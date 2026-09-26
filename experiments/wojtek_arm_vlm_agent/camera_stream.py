#!/usr/bin/env python3
"""Lowest-latency MJPEG relay for a USB camera.

- Pulls the camera's own MJPG frames (no decode / re-encode) when the camera supports it.
- Single capture thread, 1-frame driver queue, always serves the NEWEST frame only:
  a slow client skips frames instead of lagging behind.
- TCP_NODELAY, no HTTP buffering, multipart MJPEG that any browser <img> can show.

    camera_stream.py --dev /dev/video0 --size 640x480 --fps 30 --port 8095 --bind 0.0.0.0
    camera_stream.py --push VIEWER_HOST:8096         # also push frames to stream_relay.py on a viewer machine

Endpoints: /  (page)   /stream (MJPEG)   /snap (one JPEG)   /status (json)
Push framing (TCP): 4-byte big-endian length, 8-byte float64 capture time, JPEG bytes.
"""

import argparse
import json
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class Camera:
    def __init__(self, dev, w, h, fps, quality):
        self.dev, self.w, self.h, self.fps, self.quality = dev, w, h, fps, quality
        self.jpg = None
        self.seq = 0
        self.ts = 0.0
        self.passthrough = False
        self.measured_fps = 0.0
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        # Accept /dev/v4l/by-id/... symlinks (stable across replugs); V4L2 needs the real node.
        dev = os.path.realpath(self.dev)
        if not os.path.exists(dev):
            raise RuntimeError(f"{self.dev} not present")
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open {self.dev} ({dev})")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Ask for the raw compressed buffer. If the driver honours it, frames are JPEG bytes.
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        ok, f = cap.read()
        if not ok:
            raise RuntimeError("no frame")
        raw = f.ravel()
        self.passthrough = bool(raw.size > 4 and raw[0] == 0xFF and raw[1] == 0xD8)
        if not self.passthrough:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        print(f"opened {self.dev} {int(cap.get(3))}x{int(cap.get(4))}@{cap.get(5):.0f} "
              f"passthrough={self.passthrough}", flush=True)
        return cap

    def _run(self):
        while True:
            try:
                cap = self._open()
                n, t0 = 0, time.time()
                while True:
                    ok, f = cap.read()
                    if not ok:
                        raise RuntimeError("read failed")
                    if self.passthrough:
                        jpg = f.tobytes()
                    else:
                        ok, enc = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                        if not ok:
                            continue
                        jpg = enc.tobytes()
                    with self.cond:
                        self.jpg, self.ts = jpg, time.time()
                        self.seq += 1
                        self.cond.notify_all()
                    n += 1
                    if n % 60 == 0:
                        self.measured_fps = 60 / (time.time() - t0)
                        t0 = time.time()
            except Exception as e:  # noqa: BLE001
                print(f"camera error: {e}; retrying", flush=True)
                time.sleep(1)

    def wait(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.jpg, self.seq


PAGE = b"""<!doctype html><html><head><title>fast cam</title><style>body{margin:0;background:#000;
display:flex;justify-content:center;align-items:center;height:100vh}img{max-width:100%;max-height:100%}
</style></head><body><img src="/stream"></body></html>"""


def make_handler(cam):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif p == "/status":
                body = json.dumps({"passthrough": cam.passthrough, "fps": round(cam.measured_fps, 1),
                                   "age_s": round(time.time() - cam.ts, 3) if cam.ts else None,
                                   "frames": cam.seq}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif p == "/snap":
                jpg, _ = cam.wait(-1)
                self.send_response(200 if jpg else 503)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg or b"")))
                self.end_headers()
                self.wfile.write(jpg or b"")
            elif p == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                seq = cam.seq  # start from the next frame, never a stale one
                try:
                    while True:
                        jpg, seq = cam.wait(seq)
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_response(404)
                self.send_header("Content-Length", "0")
                self.end_headers()

    return H


def pusher(cam, target):
    """Push the newest frame to a relay over one TCP connection; reconnect forever."""
    host, port = target.rsplit(":", 1)
    while True:
        try:
            s = socket.create_connection((host, int(port)), timeout=5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(5)
            print(f"pushing to {target}", flush=True)
            seq = cam.seq
            while True:
                jpg, seq = cam.wait(seq)
                if jpg is None:
                    continue
                s.sendall(struct.pack(">Id", len(jpg), cam.ts) + jpg)
        except OSError as e:
            print(f"push to {target} failed: {e}; retrying", flush=True)
            time.sleep(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dev", default="/dev/video0")
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=70, help="JPEG quality, only used without passthrough")
    ap.add_argument("--port", type=int, default=8095)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--push", default=None, help="HOST:PORT of stream_relay.py to push frames to")
    ap.add_argument("--stale-exit", type=float, default=5.0, help="exit (for supervisor restart) after this many s without a frame")
    ap.add_argument("--startup-timeout", type=float, default=30.0, help="exit if no frame at all arrives within this many s")
    a = ap.parse_args()
    w, h = (int(x) for x in a.size.lower().split("x"))
    cam = Camera(a.dev, w, h, a.fps, a.quality)

    def watchdog():
        # A V4L2 read on an unplugged camera can block forever, and a capture object that
        # failed once can keep failing to reopen the device after a replug. Exit so the
        # supervisor (docker --restart) relaunches a fresh process: after --stale-exit s
        # without frames, or after --startup-timeout s if no frame ever arrived.
        t0 = time.time()
        while True:
            time.sleep(1)
            now = time.time()
            if cam.ts and now - cam.ts > a.stale_exit:
                print(f"no frame for {a.stale_exit}s: exiting for restart", flush=True)
                os._exit(3)
            if not cam.ts and now - t0 > a.startup_timeout:
                print(f"no frame within {a.startup_timeout}s of start: exiting for restart", flush=True)
                os._exit(3)

    threading.Thread(target=watchdog, daemon=True).start()
    if a.push:
        threading.Thread(target=pusher, args=(cam, a.push), daemon=True).start()
    srv = ThreadingHTTPServer((a.bind, a.port), make_handler(cam))
    srv.daemon_threads = True
    print(f"serving http://{a.bind}:{a.port}/stream", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
