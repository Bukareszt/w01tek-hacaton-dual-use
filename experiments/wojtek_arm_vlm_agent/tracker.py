#!/usr/bin/env python3
"""People tracker: YOLO11 + ByteTrack over an MJPEG (or V4L2) source, served as MJPEG + JSON.

Runs on the GPU host. Pulls the camera's MJPEG stream (camera_stream.py on the robot),
tracks all COCO classes with YOLO11 + ByteTrack, and serves:

    /            HTML page with the annotated stream
    /stream      annotated MJPEG
    /snap        one annotated JPEG
    /tracks      JSON: current tracks [{id, bbox:[x1,y1,x2,y2], center:[cx,cy], conf, age_s}]
    /frame       raw (unannotated) JPEG of the latest processed frame; header X-Tracks carries
                 the JSON tracks computed on exactly that frame
    /status      JSON: fps, device, source state, track count

Usage:
    tracker.py --source http://127.0.0.1:8092/stream/wrist --port 8093
    tracker.py --source /dev/video0 --port 8093            (direct V4L2)
"""

import argparse
import json
import socket
import struct
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

BOUNDARY = b"--frame"
COLORS = [(66, 133, 244), (52, 168, 83), (251, 188, 5), (234, 67, 53),
          (171, 71, 188), (0, 172, 193), (255, 112, 67), (120, 144, 156)]


class Source:
    """Keeps only the latest frame from an MJPEG HTTP stream or a V4L2 device."""

    def __init__(self, url, width=None, height=None):
        self.url = url
        self.width, self.height = width, height
        self.frame = None
        self.ts = 0.0
        self.state = "connecting"
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                if self.url.startswith("http"):
                    self._read_mjpeg()
                else:
                    self._read_v4l2()
            except Exception as e:  # noqa: BLE001
                self.state = f"error: {e}"
            time.sleep(1.0)

    def _publish(self, frame):
        with self.lock:
            self.frame = frame
            self.ts = time.time()
        self.state = "live"

    def _read_v4l2(self):
        cap = cv2.VideoCapture(self.url, cv2.CAP_V4L2)
        if self.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open {self.url}")
        while True:
            ok, frame = cap.read()
            if not ok:
                cap.release()
                raise RuntimeError("read failed")
            self._publish(frame)

    def _read_mjpeg(self):
        """Parse multipart/x-mixed-replace by its Content-Length headers: each frame is
        consumed the moment it fully arrives (no block-size waits, no marker scanning)."""
        resp = urllib.request.urlopen(self.url, timeout=10)
        while True:
            length = None
            line = resp.readline()
            if not line:
                raise RuntimeError("stream ended")
            if not line.strip():
                continue  # blank / boundary separators between parts
            while line.strip():
                if line.lower().startswith(b"content-length:"):
                    length = int(line.split(b":", 1)[1])
                line = resp.readline()
                if not line:
                    raise RuntimeError("stream ended")
            if length is None:
                continue  # boundary line without headers yet
            jpg = resp.read(length)
            if len(jpg) != length:
                raise RuntimeError("short frame")
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                self._publish(frame)

    def latest(self):
        with self.lock:
            return self.frame, self.ts


class Tracker:
    def __init__(self, source, model_path, device, imgsz, conf, tracker_cfg, trail_len, jpeg_q, classes=None):
        from ultralytics import YOLO

        self.source = source
        self.model = YOLO(model_path)
        self.names = dict(self.model.names)          # COCO: 80 classes
        self.classes = classes
        self.device, self.imgsz, self.conf = device, imgsz, conf
        self.tracker_cfg = tracker_cfg
        self.jpeg_q = jpeg_q
        self.trails = {}            # id -> deque of centers
        self.first_seen = {}        # id -> timestamp
        self.trail_len = trail_len
        self.tracks = []
        self.jpg = None
        self.raw_jpg = None
        self.tracks_ts = 0.0
        self.fps = 0.0
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.frame_idx = 0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_ts = 0.0
        stamps = deque(maxlen=40)   # recent frame times -> fps over a ~2 s window
        while True:
            frame, ts = self.source.latest()
            if frame is None or ts == last_ts:
                time.sleep(0.005)
                continue
            last_ts = ts
            results = self.model.track(frame, persist=True, classes=self.classes, conf=self.conf,
                                       imgsz=self.imgsz, device=self.device,
                                       tracker=self.tracker_cfg, verbose=False)
            tracks = self._extract(results[0], ts)
            annotated = self._draw(frame, tracks)
            now = time.time()
            stamps.append(now)
            fps = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
            ok, enc = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
            ok_raw, enc_raw = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
            with self.cond:
                self.tracks = tracks
                self.tracks_ts = ts
                if ok_raw:
                    self.raw_jpg = enc_raw.tobytes()
                self.fps = fps
                if ok:
                    self.jpg = enc.tobytes()
                self.frame_idx += 1
                self.cond.notify_all()

    def _extract(self, res, ts):
        tracks = []
        boxes = res.boxes
        if boxes is None or boxes.id is None:
            self._prune(set())
            return tracks
        ids = boxes.id.int().tolist()
        xyxy = boxes.xyxy.tolist()
        confs = boxes.conf.tolist()
        clss = boxes.cls.int().tolist()
        for tid, (x1, y1, x2, y2), c, k in zip(ids, xyxy, confs, clss):
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            self.trails.setdefault(tid, deque(maxlen=self.trail_len)).append((int(cx), int(cy)))
            self.first_seen.setdefault(tid, ts)
            tracks.append({"id": tid, "bbox": [round(x1), round(y1), round(x2), round(y2)],
                           "center": [round(cx), round(cy)], "conf": round(c, 3),
                           "cls": self.names.get(k, str(k)), "cls_id": k,
                           "age_s": round(ts - self.first_seen[tid], 2)})
        self._prune({t["id"] for t in tracks})
        return tracks

    def _prune(self, live_ids):
        for tid in list(self.trails):
            if tid not in live_ids:
                # keep trail briefly so short occlusions do not wipe history
                if len(self.trails[tid]) and self.trails[tid].maxlen:
                    self.trails[tid].popleft()
                if not self.trails[tid]:
                    del self.trails[tid]
                    self.first_seen.pop(tid, None)

    def _draw(self, frame, tracks):
        out = frame.copy()
        for t in tracks:
            col = COLORS[t["id"] % len(COLORS)]
            x1, y1, x2, y2 = t["bbox"]
            cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
            label = f"{t.get('cls', 'obj')} #{t['id']} {t['conf']:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(out, (x1, max(0, y1 - th - 8)), (x1 + tw + 6, y1), col, -1)
            cv2.putText(out, label, (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 2, cv2.LINE_AA)
            pts = list(self.trails.get(t["id"], []))
            for a, b in zip(pts, pts[1:]):
                cv2.line(out, a, b, col, 2)
            cv2.circle(out, tuple(t["center"]), 4, col, -1)
        hud = f"{len(tracks)} people  {self.fps:4.1f} fps  {self.device}"
        cv2.putText(out, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    def wait_frame(self, last_idx, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_idx != last_idx, timeout=timeout)
            return self.jpg, self.frame_idx


def pusher(tracker, target):
    """Push each annotated frame to stream_relay.py (4-byte len, 8-byte float64 ts, JPEG)."""
    host, port = target.rsplit(":", 1)
    while True:
        try:
            s = socket.create_connection((host, int(port)), timeout=5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(5)
            print(f"pushing annotated frames to {target}", flush=True)
            idx = tracker.frame_idx
            while True:
                jpg, idx = tracker.wait_frame(idx)
                if jpg is None:
                    continue
                s.sendall(struct.pack(">Id", len(jpg), time.time()) + jpg)
        except OSError as e:
            print(f"push to {target} failed: {e}; retrying", flush=True)
            time.sleep(1)


PAGE = b"""<!doctype html><html><head><title>people tracker</title>
<style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif;display:flex;flex-direction:column;
align-items:center;gap:8px;padding:8px}img{max-width:100%;background:#222}pre{margin:0;font:12px monospace;
color:#9c9;white-space:pre-wrap}</style></head><body>
<img src="/stream"><pre id="t">loading tracks...</pre>
<script>setInterval(async()=>{try{const r=await fetch('/tracks');document.getElementById('t').textContent=
JSON.stringify(await r.json(),null,1)}catch(e){}},500)</script></body></html>"""


def make_handler(tracker, source):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):  # quiet
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif p == "/tracks":
                with tracker.lock:
                    self._json({"ts": tracker.tracks_ts, "frame_idx": tracker.frame_idx, "tracks": tracker.tracks})
            elif p == "/frame":
                with tracker.lock:
                    raw, meta = tracker.raw_jpg, json.dumps({"ts": tracker.tracks_ts, "frame_idx": tracker.frame_idx,
                                                             "tracks": tracker.tracks})
                if raw is None:
                    return self._json({"error": "no frame"}, 503)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("X-Tracks", meta)
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            elif p == "/status":
                _, ts = source.latest()
                with tracker.lock:
                    self._json({"source": source.state, "source_age_s": round(time.time() - ts, 2) if ts else None,
                                "fps": round(tracker.fps, 1), "device": tracker.device,
                                "n_tracks": len(tracker.tracks), "frames": tracker.frame_idx})
            elif p == "/snap":
                jpg, _ = tracker.wait_frame(-1)
                if jpg is None:
                    return self._json({"error": "no frame"}, 503)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
            elif p == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                idx = -1
                try:
                    while True:
                        jpg, idx = tracker.wait_frame(idx)
                        if jpg is None:
                            continue
                        self.wfile.write(BOUNDARY + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._json({"error": "not found"}, 404)

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="http://127.0.0.1:8092/stream/wrist",
                    help="MJPEG URL or /dev/videoN")
    ap.add_argument("--model", default="yolo11s.pt")
    ap.add_argument("--device", default="0", help="CUDA index, 'cpu', or 'mps'")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--classes", default="all", help="'all' (80 COCO classes) or comma-separated names/ids, e.g. person,dog")
    ap.add_argument("--tracker", default="bytetrack.yaml", help="bytetrack.yaml or botsort.yaml")
    ap.add_argument("--trail", type=int, default=40, help="trail length in frames")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--push", default=None, help="HOST:PORT of stream_relay.py to push annotated frames to")
    args = ap.parse_args()

    source = Source(args.source, args.width, args.height)
    classes = None
    if args.classes != "all":
        from ultralytics import YOLO as _Y
        names = {v: k for k, v in _Y(args.model).names.items()}
        classes = [names[c] if c in names else int(c) for c in args.classes.split(",")]
    tracker = Tracker(source, args.model, args.device, args.imgsz, args.conf, args.tracker,
                      args.trail, args.jpeg_quality, classes)
    if args.push:
        threading.Thread(target=pusher, args=(tracker, args.push), daemon=True).start()
    srv = ThreadingHTTPServer((args.bind, args.port), make_handler(tracker, source))
    srv.daemon_threads = True
    print(f"serving http://{args.bind}:{args.port}/  source={args.source} model={args.model} "
          f"device={args.device}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
