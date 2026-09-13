#!/usr/bin/env python3
"""People commander v2: a reactive, tool-driven VLM agent for the SO-101 camera arm. No memory.

    tracker.py  --/frame-->  commander  --/aim-->  arm_follower.py
                                      |
                                      +--> VLM agent (Ollama tool calling): look / turn / sweep /
                                           inspect / track / aim_pan / find / cycle / stop / home / wait
                                      +--> annotated MJPEG pushed to stream_relay.py on the laptop
                                      +--> HTTP UI + JSON API (port 8100)

Every observation tool returns a fresh picture plus the detections in it, so the model never
reasons about stale state. Loops (sweeping, finding, cycling) run deterministically in code.

API:
    GET  /               UI                         GET /state   JSON state
    GET  /aim            what the follower should do: {"id": N} | {"pan","tilt",...} | {"id": null}
    GET  /crop/<id>      crop of a visible person    GET /snap    annotated JPEG   GET /stream  MJPEG
    POST /agent {"text"} run a natural-language command (GET /agent = transcript)
    POST /instruction {"text"}   standing filter rule ("" = everyone)
    POST /control {"mode": "off"|"once"|"loop", "next": true, "dwell_s": f, "lock_s": f, "timeout_s": f}
    POST /aim {"id": N} | {"id": null} | {"pan": deg, "tilt": deg} | {"posture": true|"save"}
"""

import argparse
import base64
import json
import os
import socket
import struct
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

SYSTEM_PROMPT = (
    "You are a strict visual filter for a robot camera. Each image is a crop around ONE detected "
    "object or person: the MAIN SUBJECT is the largest, most central one in the crop. Judge only the "
    "main subject; ignore other things partially visible at the edges or in the background. "
    "The operator gives one selection rule. Answer match=true ONLY if the main subject clearly and "
    "unambiguously satisfies the rule as literally stated (e.g. 'white shirt' means the shirt itself "
    "is white, not light grey, beige or a white jacket over a dark shirt). If the relevant attribute "
    "is not visible, occluded, too small to see, or you are unsure, answer match=false. "
    "Give a reason of at most 8 words naming the decisive evidence."
)

RESULT_SCHEMA = {
    "type": "object",
    "properties": {
        "results": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "person": {"type": "integer"},
                    "match": {"type": "boolean"},
                    "reason": {"type": "string"},
                },
                "required": ["person", "match", "reason"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["results"],
    "additionalProperties": False,
}


def load_env_file(path):
    if not path or not os.path.exists(path):
        return
    for line in open(path):
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def iou(a, b):
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def crop_person(frame, box, px=256):
    """JPEG crop around one detection, padded 10%/5%, longest side <= px."""
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = box
    mx, my = int(0.1 * (x2 - x1)), int(0.05 * (y2 - y1))
    x1, y1, x2, y2 = max(0, x1 - mx), max(0, y1 - my), min(w, x2 + mx), min(h, y2 + my)
    c = frame[y1:y2, x1:x2]
    if c.size == 0:
        return None
    scale = px / max(c.shape[:2])
    if scale < 1:
        c = cv2.resize(c, (max(1, int(c.shape[1] * scale)), max(1, int(c.shape[0] * scale))),
                       interpolation=cv2.INTER_AREA)
    ok, enc = cv2.imencode(".jpg", c, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return enc.tobytes() if ok else None


class Classifier:
    """Batches crops of unclassified people into one Claude request; caches verdicts by track id."""

    def __init__(self, model, recheck_s, max_batch, crop_px, backend="auto", ollama_url="http://127.0.0.1:11434"):
        self.backend = ("ollama" if ":" in model else "anthropic") if backend == "auto" else backend
        self.ollama_url = ollama_url
        if self.backend == "anthropic":
            import anthropic

            self.anthropic = anthropic
            self.client = anthropic.Anthropic()
        self.model = model
        self.effort_ok = "haiku" not in model  # Haiku 4.5 rejects output_config.effort
        self.fallbacks_ok = True
        self.recheck_s, self.max_batch, self.crop_px = recheck_s, max_batch, crop_px
        self.verdicts = {}      # tid -> {match, reason, ts, box, source}
        self.crops = {}         # tid -> jpg bytes (last crop sent)
        self.last_ms = None
        self.calls = 0
        self.errors = 0
        self.last_error = None
        self.busy = False
        self.lock = threading.Lock()

    def crop(self, frame, box):
        return crop_person(frame, box, self.crop_px)

    def needs_check(self, tid, now):
        v = self.verdicts.get(tid)
        return v is None or (now - v["ts"]) > self.recheck_s

    def classify(self, instruction, items):
        """items: list of (tid, jpg). Returns {tid: (match, reason)}."""
        if self.backend == "ollama":
            return self.classify_ollama(instruction, items)
        content = []
        for i, (tid, jpg) in enumerate(items, 1):
            content.append({"type": "text", "text": f"Person {i}:"})
            content.append({"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                        "data": base64.standard_b64encode(jpg).decode()}})
        content.append({"type": "text", "text":
                        f'Selection rule: "{instruction}"\nFor each numbered image judge its main subject only. '
                        f'Return one result per image, numbered 1..{len(items)}; when in doubt, match=false.'})
        kwargs = dict(model=self.model, max_tokens=1024, system=SYSTEM_PROMPT,
                      messages=[{"role": "user", "content": content}],
                      output_config={"format": {"type": "json_schema", "schema": RESULT_SCHEMA}})
        if self.effort_ok:
            kwargs["output_config"]["effort"] = "low"
        t0 = time.perf_counter()
        if self.fallbacks_ok:
            try:
                resp = self.client.beta.messages.create(betas=["server-side-fallback-2026-07-01"],
                                                        fallbacks="default", **kwargs)
            except (self.anthropic.BadRequestError, TypeError) as e:
                print(f"fallbacks unsupported here ({e}); continuing without", flush=True)
                self.fallbacks_ok = False
                resp = self.client.messages.create(**kwargs)
        else:
            resp = self.client.messages.create(**kwargs)
        self.last_ms = (time.perf_counter() - t0) * 1000
        if resp.stop_reason == "refusal":
            raise RuntimeError("model refused the request")
        text = next(b.text for b in resp.content if b.type == "text")
        data = json.loads(text)
        out = {}
        for r in data.get("results", []):
            idx = int(r["person"]) - 1
            if 0 <= idx < len(items):
                out[items[idx][0]] = (bool(r["match"]), str(r.get("reason", "")))
        return out

    def describe_ollama(self, jpg):
        """Short appearance description of one person crop (for the long-term registry)."""
        body = {"model": self.model, "stream": False, "keep_alive": "30m",
                "options": {"temperature": 0, "num_predict": 48},
                "messages": [{"role": "user", "content":
                              "Describe the main person in this crop in at most 12 words, for recognising them "
                              "later: apparent gender, hair, top garment colour and type, notable accessories "
                              "(glasses, hat, bag). Plain text, no preamble.",
                              "images": [base64.standard_b64encode(jpg).decode()]}]}
        req = urllib.request.Request(self.ollama_url + "/api/chat", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as r:
            return json.loads(r.read())["message"]["content"].strip().strip('"')[:120]

    def classify_ollama(self, instruction, items):
        """Local VLM through Ollama: one request per crop (image tokens dominate anyway),
        JSON-schema constrained output, temperature 0."""
        schema = {"type": "object", "properties": {"match": {"type": "boolean"}, "reason": {"type": "string"}},
                  "required": ["match", "reason"]}
        out = {}
        t0 = time.perf_counter()
        for tid, jpg in items:
            body = {"model": self.model, "stream": False, "format": schema, "keep_alive": "30m",
                    "options": {"temperature": 0, "num_predict": 60},
                    "messages": [{"role": "system", "content": SYSTEM_PROMPT},
                                 {"role": "user", "content": f'Selection rule: "{instruction}"\nDoes the MAIN SUBJECT of this '
                                  'crop clearly satisfy the rule? When in doubt answer false. '
                                  'Answer JSON: match (true/false) and reason (max 8 words).',
                                  "images": [base64.standard_b64encode(jpg).decode()]}]}
            req = urllib.request.Request(self.ollama_url + "/api/chat", data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=60) as r:
                resp = json.loads(r.read())
            try:
                d = json.loads(resp["message"]["content"])
                out[tid] = (bool(d["match"]), str(d.get("reason", ""))[:60])
            except (KeyError, ValueError, TypeError) as e:
                print(f"ollama: unparsable answer for #{tid}: {resp.get('message', {}).get('content', '')[:80]!r} ({e})",
                      flush=True)
        self.last_ms = (time.perf_counter() - t0) * 1000 / max(1, len(items))
        return out

    def run_batch(self, instruction, frame, tracks):
        now = time.time()
        pending = [t for t in tracks if self.needs_check(t["id"], now)]
        if not pending:
            return
        pending.sort(key=lambda t: self.verdicts.get(t["id"], {}).get("ts", 0.0))
        items = []
        for t in pending[:self.max_batch]:
            jpg = self.crop(frame, t["bbox"])
            if jpg:
                items.append((t["id"], jpg))
                self.crops[t["id"]] = jpg
        if not items:
            return
        self.busy = True
        try:
            res = self.classify(instruction, items)
            self.calls += 1
            with self.lock:
                for t in pending[:self.max_batch]:
                    if t["id"] in res:
                        m, reason = res[t["id"]]
                        self.verdicts[t["id"]] = {"match": m, "reason": reason, "ts": time.time(),
                                                  "box": t["bbox"], "source": "vlm"}
        except Exception as e:  # noqa: BLE001
            self.errors += 1
            self.last_error = f"{type(e).__name__}: {e}"[:200]
            print(f"classify error: {self.last_error}", flush=True)
            time.sleep(1.0)
        finally:
            self.busy = False

    def inherit(self, tracks, recent_gone):
        """A freshly assigned id whose box overlaps a just-vanished id inherits its verdict."""
        with self.lock:
            for t in tracks:
                if t["id"] in self.verdicts:
                    continue
                best, best_iou = None, 0.5
                for old_id, (box, ts) in recent_gone.items():
                    o = iou(t["bbox"], box)
                    if o > best_iou:
                        best, best_iou = old_id, o
                if best is not None and best in self.verdicts:
                    v = dict(self.verdicts[best])
                    v["source"], v["box"] = "inherited", t["bbox"]
                    v["ts"] = time.time() - self.recheck_s * 0.5   # recheck sooner than a fresh one
                    self.verdicts[t["id"]] = v
                    if best in self.crops:
                        self.crops[t["id"]] = self.crops[best]

    def forget_all(self):
        with self.lock:
            self.verdicts.clear()

    def prune(self, live_ids, max_age=30.0):
        now = time.time()
        with self.lock:
            for tid in list(self.verdicts):
                if tid not in live_ids and now - self.verdicts[tid]["ts"] > max_age:
                    del self.verdicts[tid]
                    self.crops.pop(tid, None)




class Commander:
    """Live state (frames, tracks, arm), the visible-only sequencer, motion primitives for the
    agent's tools, and the annotated output stream. No memory of anything out of view."""

    def __init__(self, a):
        self.a = a
        self.instruction = ""
        self.instruction_classes = None     # optional class filter for the standing rule / cycle
        self.mode = "off"                   # off | once | loop
        self.manual_target = None
        self.manual_goto = None             # absolute joint goals while the agent moves the arm
        self.home = None
        self.run_done = set()
        self.no_cand_since = None
        self.dwell_s, self.lock_s, self.timeout_s = a.dwell, a.lock, a.timeout
        self.clf = (Classifier(a.model, a.recheck, a.batch, a.crop_px, a.backend, a.ollama_url)
                    if a.model != "none" else None)
        self.w, self.h = a.width, a.height
        self.target = None
        self.phase = "idle"
        self.phase_since = time.time()
        self.lock_since = None
        self.skip_next = False
        self.busy = None                    # what the agent is doing with the arm right now
        self.region = None                  # {"label","box":[x1,y1,x2,y2] normalized,"ts"} aimed-at image region
        self.frame = None
        self.tracks = []
        self.tracks_ts = 0.0
        self.frame_idx = -1
        self.jpg = None
        self.seq = 0
        self.cond = threading.Condition()
        self.recent_gone = {}
        self._prev = []
        self.tracker_state = "connecting"
        self.arm = {"pos": None, "limits": None, "ts": 0.0, "mode": None}
        self.agent = None
        threading.Thread(target=self._arm_loop, daemon=True).start()
        threading.Thread(target=self._frame_loop, daemon=True).start()
        threading.Thread(target=self._classify_loop, daemon=True).start()
        if a.push:
            threading.Thread(target=self._pusher, args=(a.push,), daemon=True).start()

    # ---------- inputs ----------
    def _arm_loop(self):
        while True:
            try:
                with urllib.request.urlopen(self.a.follower + "/", timeout=0.5) as r:
                    st = json.loads(r.read())
                if st.get("pos"):
                    self.arm = {"pos": st["pos"], "limits": st.get("limits") or self.arm["limits"],
                                "ts": time.time(), "mode": st.get("mode")}
                time.sleep(1 / 30)
            except Exception:  # noqa: BLE001
                self.arm["mode"] = "unreachable"
                time.sleep(0.5)

    def _fetch_frame(self):
        with urllib.request.urlopen(self.a.tracker + "/frame", timeout=1.0) as r:
            meta = json.loads(r.headers.get("X-Tracks", "{}"))
            frame = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        return frame, meta

    def _frame_loop(self):
        last_idx = -1
        while True:
            try:
                frame, meta = self._fetch_frame()
                self.tracker_state = "live"
            except Exception as e:  # noqa: BLE001
                self.tracker_state = f"error: {e}"
                time.sleep(0.5)
                continue
            if frame is None or meta.get("frame_idx", -1) == last_idx:
                time.sleep(0.005)
                continue
            last_idx = meta["frame_idx"]
            tracks = [t for t in meta.get("tracks", []) if t["conf"] >= self.a.min_conf]
            now = time.time()
            ids = {t["id"] for t in tracks}
            for t in self._prev:
                if t["id"] not in ids:
                    self.recent_gone[t["id"]] = (t["bbox"], now)
            self.recent_gone = {k: v for k, v in self.recent_gone.items() if now - v[1] < 1.5}
            if self.clf:
                self.clf.inherit(tracks, self.recent_gone)
                self.clf.prune(ids)
            with self.cond:
                self.frame, self.tracks, self.tracks_ts, self.frame_idx = frame, tracks, meta.get("ts", now), last_idx
                self._prev = tracks
            self._sequence(tracks, now)
            jpg = self._draw(frame, tracks)
            with self.cond:
                self.jpg = jpg
                self.seq += 1
                self.cond.notify_all()

    def _classify_loop(self):
        while True:
            if not self.clf or not self.instruction or self.frame is None:
                time.sleep(0.1)
                continue
            with self.cond:
                frame, tracks, instruction = self.frame, list(self.tracks), self.instruction
            tracks = [t for t in tracks if self.class_ok(t)]
            self.clf.run_batch(instruction, frame, tracks)
            time.sleep(0.05)

    # ---------- helpers ----------
    def class_ok(self, t, classes=None):
        classes = self.instruction_classes if classes is None else classes
        return not classes or t.get("cls", "person") in classes

    def matches(self, t):
        if not self.class_ok(t):
            return False
        if not self.instruction:
            return True
        v = self.clf.verdicts.get(t["id"]) if self.clf else None
        return bool(v and v["match"])

    def err(self, t):
        x1, y1, x2, y2 = t["bbox"]
        frac = 0.4 if t.get("cls", "person") == "person" else 0.5
        cx, ay = (x1 + x2) / 2, y1 + frac * (y2 - y1)
        return (cx - self.w / 2) / (self.w / 2), (ay - self.h / 2) / (self.h / 2)

    def snapshot(self):
        """(frame, tracks, frame_idx) of the newest processed frame."""
        with self.cond:
            return self.frame, list(self.tracks), self.frame_idx

    def wait_new_frame(self, timeout=1.0):
        with self.cond:
            idx = self.frame_idx
            self.cond.wait_for(lambda: self.frame_idx != idx, timeout=timeout)
            return self.frame, list(self.tracks), self.frame_idx

    # ---------- sequencer (visible people only) ----------
    def _continuity(self, tracks, now):
        """The tracker often re-numbers the same body. If the target id vanished within the last
        second, adopt the detection of the same class that overlaps its last box most."""
        by_id = {t["id"]: t for t in tracks}
        if self.target is not None and self.target in by_id:
            t = by_id[self.target]
            self._tgt_last = (t["bbox"], t.get("cls", "person"), now)
            return
        last = getattr(self, "_tgt_last", None)
        if self.target is None or not last or now - last[2] > self.a.continuity_s:
            return
        box, cls, _ = last
        best, best_iou = None, 0.2
        for t in tracks:
            if t.get("cls", "person") != cls:
                continue
            o = iou(box, t["bbox"])
            if o > best_iou:
                best, best_iou = t, o
        if best is not None:
            print(f"continuity: target #{self.target} -> #{best['id']} (iou {best_iou:.2f})", flush=True)
            if self.manual_target == self.target:
                self.manual_target = best["id"]
            if self.target in self.run_done:
                self.run_done.add(best["id"])
            self.target = best["id"]
            if self.clf and self.instruction and self.target in self.clf.verdicts and best["id"] not in self.clf.verdicts:
                self.clf.verdicts[best["id"]] = dict(self.clf.verdicts[self.target])
            self._tgt_last = (best["bbox"], cls, now)

    def _sequence(self, tracks, now):
        if now - self.tracks_ts > self.a.stale_s:
            self.target, self.phase, self.lock_since = None, "stale-video", None
            return
        self._continuity(tracks, now)
        if self.manual_goto is not None:          # the agent is moving the arm / holding a region
            self.target = None
            self.phase = self.busy or (f"aimed at {self.region['label']}" if self.region else "moving")
            return
        by_id = {t["id"]: t for t in tracks if self.matches(t)}
        if self.mode == "off":
            self.target = self.manual_target
            if self.target is None:
                self.phase = f"aimed at {self.region['label']}" if self.region else "idle"
            elif self.target in {t["id"] for t in tracks}:
                self.phase = self._track_phase({t["id"]: t for t in tracks}, now)
            else:
                self.phase = "target-lost"
            return
        candidates = {i: t for i, t in by_id.items()
                      if not (self.mode == "once" and i in self.run_done) and t.get("age_s", 1.0) >= self.a.min_age}
        if self.target in by_id and self.target not in candidates and not self.skip_next:
            candidates[self.target] = by_id[self.target]
        if candidates:
            self.no_cand_since = None
        if self.target not in candidates or self.skip_next:
            if self.skip_next and self.target is not None:
                self.run_done.add(self.target)
            self.skip_next = False
            self._advance(candidates, now)
            return
        ph = self._track_phase(candidates, now)
        if ph in ("timeout", "served"):
            self.run_done.add(self.target)
            rest = {i: t for i, t in candidates.items() if i != self.target}
            if ph == "timeout" or self.mode == "once" or len(candidates) > 1:
                self._advance(rest, now)

    def _track_phase(self, by_id, now):
        t = by_id[self.target]
        ex, ey = self.err(t)
        centered = abs(ex) < self.a.lock_err and abs(ey) < self.a.lock_err
        if self.phase not in ("aiming", "locked"):
            self.phase, self.phase_since, self.lock_since = "aiming", now, None
        if self.phase == "aiming":
            if centered:
                if self.lock_since is None:
                    self.lock_since = now
                elif now - self.lock_since >= self.lock_s:
                    self.phase, self.phase_since = "locked", now
            else:
                self.lock_since = None
            if self.mode != "off" and now - self.phase_since > self.timeout_s:
                return "timeout"
        elif self.phase == "locked":
            if not centered and self.mode == "off":
                self.phase, self.phase_since, self.lock_since = "aiming", now, None
            elif now - self.phase_since >= self.dwell_s:
                return "served"
        return self.phase

    def _advance(self, by_id, now):
        if not by_id:
            self.target, self.lock_since = None, None
            if self.mode == "once" and self.run_done:
                self.no_cand_since = self.no_cand_since or now
                self.phase = "finishing"
                if now - self.no_cand_since >= self.a.done_grace:
                    self.mode, self.phase, self.manual_target = "off", "done", None
                    print(f"cycle once finished: served {sorted(self.run_done)}", flush=True)
            else:
                self.phase = "searching"
            return
        order = sorted(by_id)
        nxt = next((i for i in order if self.target is not None and i > self.target), order[0])
        self.target, self.phase, self.phase_since, self.lock_since = nxt, "aiming", now, None

    # ---------- commands ----------
    def set_instruction(self, text, classes=None):
        self.instruction = (text or "").strip()
        self.instruction_classes = classes or None
        if self.clf:
            self.clf.forget_all()
        self.run_done = set()
        if self.mode != "off":
            self.target, self.phase, self.lock_since = None, "searching", None
        print(f"rule: {self.instruction!r} classes={self.instruction_classes}", flush=True)

    def start(self, mode):
        self.mode, self.run_done, self.no_cand_since = mode, set(), None
        self.manual_target, self.manual_goto = None, None
        self.target, self.phase, self.lock_since = None, "searching", None

    def stop(self):
        self.mode, self.manual_target, self.manual_goto, self.target, self.phase = "off", None, None, None, "idle"
        self.busy, self.region = None, None

    def region_goal(self, box):
        """Joint goals that center a normalized [x1,y1,x2,y2] image box (same law as the follower)."""
        pos = self.arm["pos"]
        if not pos:
            return None
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        ex, ey = (cx - 0.5) * 2, (cy - 0.5) * 2
        return pos["pan"] + ex * self.a.hfov / 2, pos["tilt"] + ey * self.a.vfov / 2

    def hold_region(self, label, box):
        """Keep pointing where we are and show the region on the video (non-detector target)."""
        self.mode, self.manual_target, self.target = "off", None, None
        self.region = {"label": label, "box": list(box), "ts": time.time()}
        pos = self.arm["pos"]
        if pos:
            self.manual_goto = {"pan": pos["pan"], "tilt": pos["tilt"], "speed": self.a.goto_speed}
        self.busy = None
        self.phase, self.phase_since = f"aimed at {label}", time.time()

    def track(self, tid):
        self.region = None
        self.mode, self.manual_goto, self.busy = "off", None, None
        self.manual_target, self.target = tid, tid
        _, tracks, _ = self.snapshot()
        t = next((x for x in tracks if x["id"] == tid), None)
        if t is not None:                        # seed continuity with the box we were asked to track
            self._tgt_last = (t["bbox"], t.get("cls", "person"), time.time())
        self.phase, self.phase_since, self.lock_since = "aiming", time.time(), None

    def goto(self, pan=None, tilt=None, speed=None, posture=False, label="moving"):
        pos = self.arm["pos"] or {}
        g = {"speed": speed or self.a.goto_speed}
        g["pan"] = pos.get("pan", 0.0) if pan is None else float(pan)
        if posture and self.home:
            g.update({"lift": self.home["lift"], "elbow": self.home["elbow"],
                      "tilt": self.home["tilt"] if tilt is None else float(tilt), "speed": self.a.posture_speed})
        else:
            g["tilt"] = pos.get("tilt", 0.0) if tilt is None else float(tilt)
        lim = self.arm["limits"]
        if lim:
            g["pan"] = float(np.clip(g["pan"], -lim["pan"] + 1, lim["pan"] - 1))
            g["tilt"] = float(np.clip(g["tilt"], -lim["tilt"] + 1, lim["tilt"] - 1))
        self.mode, self.manual_target, self.target = "off", None, None
        self.manual_goto, self.busy = g, label
        return g

    def wait_settled(self, goal, timeout=6.0, tol=2.5):
        """Block until the arm is within tol deg of the goal (pan/tilt) or timeout."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            pos = self.arm["pos"]
            if pos and all(abs(pos[k] - goal[k]) < tol for k in ("pan", "tilt") if k in goal):
                time.sleep(0.35)             # let the picture stop moving
                return True
            time.sleep(0.05)
        return False

    def release(self):
        """Stop driving the arm manually (keeps the current position)."""
        self.manual_goto, self.busy = None, None

    def save_home(self):
        pos = self.arm["pos"]
        if pos:
            self.home = {"lift": pos["lift"], "elbow": pos["elbow"], "tilt": pos["tilt"]}
        return self.home

    # ---------- outputs ----------
    def _draw(self, frame, tracks):
        out = frame.copy()
        t = next((x for x in tracks if x["id"] == self.target), None)
        now = time.time()
        if t is not None:
            self._last_box = (t["bbox"], t.get("cls", "target"), t["id"], now)
        elif self.target is not None and getattr(self, "_last_box", None) and self._last_box[2] == self.target \
                and now - self._last_box[3] < 1.0:
            # id flickered for a frame or two: keep the last box (dim) instead of blinking
            bx, cls, tid, _ = self._last_box
            x1, y1, x2, y2 = bx
            cv2.rectangle(out, (x1, y1), (x2, y2), (140, 140, 140), 2)
            cv2.putText(out, f"{cls} #{tid} (reacquiring)", (x1 + 3, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (200, 200, 200), 2, cv2.LINE_AA)
        if t is not None:
            x1, y1, x2, y2 = t["bbox"]
            locked = self.phase == "locked"
            col = (60, 220, 60) if locked else (40, 40, 255)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(out, (x1, y1), (x2, y2), col, 3)
            # corner ticks make the box read as "the target" even when it is large
            for (px, py, sx, sy) in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
                cv2.line(out, (px, py), (px + sx * 18, py), col, 5)
                cv2.line(out, (px, py), (px, py + sy * 18), col, 5)
            cv2.line(out, (x1, cy), (x2, cy), col, 2)
            cv2.line(out, (cx, y1), (cx, y2), col, 2)
            cv2.circle(out, (cx, cy), 12, col, 2)
            status = "LOCKED" if locked else "AIMING"
            pos, lim = self.arm["pos"], self.arm["limits"]
            if pos and lim and not locked:
                ex, ey = self.err(t)
                if (abs(pos["pan"]) > lim["pan"] - 2.5 and np.sign(ex) == np.sign(pos["pan"]) and abs(ex) > self.a.lock_err) or \
                   (abs(pos["tilt"]) > lim["tilt"] - 2.5 and np.sign(ey) == np.sign(pos["tilt"]) and abs(ey) > self.a.lock_err):
                    status = "AT JOINT LIMIT"
            label = f"{t.get('cls', 'target').upper()} #{t['id']}  {status}  {t['conf']:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            ly = y1 - 10 if y1 - th - 14 > 0 else y2 + th + 12          # above the box, or below if no room
            cv2.rectangle(out, (x1, ly - th - 8), (x1 + tw + 12, ly + 6), col, -1)
            cv2.putText(out, label, (x1 + 6, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            # banner top-right
            ban = f"TRACKING {t.get('cls', 'target')} #{t['id']}"
            (bw, bh), _ = cv2.getTextSize(ban, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            cv2.rectangle(out, (self.w - bw - 18, 6), (self.w - 6, bh + 18), col, -1)
            cv2.putText(out, ban, (self.w - bw - 12, bh + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        elif self.region is not None:
            rb = self.region["box"]
            x1, y1, x2, y2 = int(rb[0] * self.w), int(rb[1] * self.h), int(rb[2] * self.w), int(rb[3] * self.h)
            col = (0, 200, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), col, 3)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.drawMarker(out, (cx, cy), col, cv2.MARKER_CROSS, 40, 2)
            label = f"AIMED: {self.region['label']}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            ly = y1 - 10 if y1 - th - 14 > 0 else y2 + th + 12
            cv2.rectangle(out, (x1, ly - th - 8), (x1 + tw + 12, ly + 6), col, -1)
            cv2.putText(out, label, (x1 + 6, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
        elif self.a.draw_all:
            for x in tracks:
                x1, y1, x2, y2 = x["bbox"]
                cv2.rectangle(out, (x1, y1), (x2, y2), (120, 120, 120), 1)
        n_people = sum(1 for x in tracks if x.get("cls", "person") == "person")
        hud = f"{n_people} people, {len(tracks) - n_people} objects | {self.phase}"
        if self.busy:
            hud += f" | {self.busy}"
        if self.instruction:
            hud += f" | rule: {self.instruction}"
        (hw, hh), _ = cv2.getTextSize(hud, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(out, (4, 4), (hw + 14, hh + 14), (20, 20, 20), -1)
        cv2.putText(out, hud, (8, hh + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1, cv2.LINE_AA)
        ok, enc = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return enc.tobytes() if ok else None

    def wait(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.jpg, self.seq

    def _pusher(self, target):
        host, port = target.rsplit(":", 1)
        while True:
            try:
                s = socket.create_connection((host, int(port)), timeout=5)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(5)
                print(f"pushing annotated frames to {target}", flush=True)
                seq = self.seq
                while True:
                    jpg, seq = self.wait(seq)
                    if jpg:
                        s.sendall(struct.pack(">Id", len(jpg), time.time()) + jpg)
            except OSError as e:
                print(f"push to {target} failed: {e}; retrying", flush=True)
                time.sleep(1)

    def state(self):
        frame, tracks, _ = self.snapshot()
        det = []
        for t in tracks:
            v = self.clf.verdicts.get(t["id"]) if (self.clf and self.instruction) else None
            ex, ey = self.err(t)
            det.append({"id": t["id"], "cls": t.get("cls", "person"), "bbox": t["bbox"], "conf": t["conf"],
                        "err": [round(ex, 2), round(ey, 2)], "is_target": t["id"] == self.target,
                        "match": (None if not self.instruction else (v["match"] if v else None)),
                        "reason": (v["reason"] if v else ""), "served": t["id"] in self.run_done})
        counts = {}
        for t in tracks:
            counts[t.get("cls", "person")] = counts.get(t.get("cls", "person"), 0) + 1
        now = time.time()
        return {"instruction": self.instruction, "instruction_classes": self.instruction_classes, "mode": self.mode,
                "target": self.target, "phase": self.phase, "phase_age_s": round(now - self.phase_since, 1),
                "busy": self.busy, "region": self.region, "dwell_s": self.dwell_s, "lock_s": self.lock_s, "timeout_s": self.timeout_s,
                "tracker": ("stale" if now - self.tracks_ts > self.a.stale_s else self.tracker_state),
                "detections": det, "counts": counts, "home": self.home,
                "arm": {"pos": self.arm["pos"], "limits": self.arm["limits"], "mode": self.arm["mode"],
                        "age_s": round(now - self.arm["ts"], 1) if self.arm["ts"] else None},
                "fov": [self.a.hfov, self.a.vfov],
                "vlm": ({"model": f"{self.clf.backend}:{self.clf.model}", "last_ms": self.clf.last_ms and round(self.clf.last_ms),
                         "calls": self.clf.calls, "errors": self.clf.errors, "last_error": self.clf.last_error,
                         "busy": self.clf.busy} if self.clf else None),
                "ts": now}


# =====================================================================================
# Agent: the VLM's eyes and hands
# =====================================================================================

AGENT_SYSTEM = (
    "You operate a robot camera arm. You see only through your tools and you have NO memory of "
    "anything out of view: when in doubt, look. Act, don't ask questions. Never answer with an "
    "intention like 'I will track it': call the tool, then report what you DID in one or two short "
    "sentences. Earlier turns are history only; the picture may have changed, so look again.\n"
    "Do exactly what the command asks and nothing more: 'look up 20' is ONE look_up call followed by "
    "a short description of the picture; 'what do you see' is ONE look. Every look/turn/look_* tool "
    "already returns the picture: after it, answer. find already turns the arm AND starts tracking: "
    "after found=true do not call aim_pan or track, just report. Never repeat a call that just failed "
    "with the same arguments.\n"
    "Tools: look (picture + numbered detections of ALL objects, filter with classes), look_left / "
    "look_right / look_up / look_down (degrees, default 30, then a new picture), turn (relative "
    "pan/tilt, then a new picture), inspect (big crop of one detection), track (follow a detection "
    "id), aim_at (point the camera at anything you can describe in the CURRENT picture, e.g. a "
    "banner, a door, a QR code - no detection needed), find (look around step by step - look, turn, "
    "look - until the described thing is in view, then aim at it or track it), cycle (visit every "
    "visible detection matching a description, once or in a loop), stop, home, wait.\n"
    "Typical: 'target the man in the white shirt' -> look; if he is in the picture, track his id; "
    "otherwise find('man in white shirt'). 'What do you see?' -> look, then describe. "
    "Detection classes are the 80 COCO classes (person, dog, cup, laptop, chair, ...)."
)

AGENT_TOOLS = [
    {"type": "function", "function": {"name": "look", "description":
        "Take a fresh picture: returns detections (id, class, position, size, targeted) and shows the "
        "annotated frame. classes: optional list of COCO class names to keep (default: all).",
        "parameters": {"type": "object", "properties": {"classes": {"type": "array", "items": {"type": "string"}}}}}},
    {"type": "function", "function": {"name": "look_left", "description": "Turn the camera left by some degrees (default 30), then look.",
        "parameters": {"type": "object", "properties": {"degrees": {"type": "number"}}}}},
    {"type": "function", "function": {"name": "look_right", "description": "Turn the camera right by some degrees (default 30), then look.",
        "parameters": {"type": "object", "properties": {"degrees": {"type": "number"}}}}},
    {"type": "function", "function": {"name": "look_up", "description": "Tilt the camera up by some degrees (default 20), then look.",
        "parameters": {"type": "object", "properties": {"degrees": {"type": "number"}}}}},
    {"type": "function", "function": {"name": "look_down", "description": "Tilt the camera down by some degrees (default 20), then look.",
        "parameters": {"type": "object", "properties": {"degrees": {"type": "number"}}}}},
    {"type": "function", "function": {"name": "turn", "description":
        "Turn the camera by a relative amount (degrees; positive pan = right, positive tilt = down), "
        "wait until it settles, then the same as look.",
        "parameters": {"type": "object", "properties": {"pan_deg": {"type": "number"}, "tilt_deg": {"type": "number"},
                                                        "classes": {"type": "array", "items": {"type": "string"}}}}}},
    {"type": "function", "function": {"name": "aim_at", "description":
        "Point the camera at something visible in the CURRENT picture described in words (any object, "
        "sign, banner, screen, door...). Centers it and holds; if it is a detected object it is tracked.",
        "parameters": {"type": "object", "properties": {"description": {"type": "string"}}, "required": ["description"]}}},
    {"type": "function", "function": {"name": "inspect", "description":
        "Show a large crop of one detection for details (glasses, logo, what they hold).",
        "parameters": {"type": "object", "properties": {"id": {"type": "integer"}}, "required": ["id"]}}},
    {"type": "function", "function": {"name": "track", "description":
        "Aim at a detection and keep following it until told otherwise.",
        "parameters": {"type": "object", "properties": {"id": {"type": "integer"}}, "required": ["id"]}}},
    {"type": "function", "function": {"name": "find", "description":
        "Look around for something described in words: look, decide where to turn from what is seen, "
        "turn, look again, until it is in view; then center it (and track it if it is a detected object). "
        "Works for anything the camera can see, not only detected classes. classes narrows detections.",
        "parameters": {"type": "object", "properties": {"description": {"type": "string"},
                                                        "classes": {"type": "array", "items": {"type": "string"}}},
                       "required": ["description"]}}},
    {"type": "function", "function": {"name": "cycle", "description":
        "Visit every VISIBLE detection that matches a description, one after another. mode once = one "
        "pass then stop, loop = keep cycling. description '' = everything of the given classes.",
        "parameters": {"type": "object", "properties": {"description": {"type": "string"},
                                                        "mode": {"type": "string", "enum": ["once", "loop"]},
                                                        "classes": {"type": "array", "items": {"type": "string"}}},
                       "required": ["mode"]}}},
    {"type": "function", "function": {"name": "stop", "description": "Stop tracking/cycling, hold position.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {"name": "home", "description": "Return to the saved observation posture.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {"name": "wait", "description": "Wait some seconds (max 10), then look.",
        "parameters": {"type": "object", "properties": {"seconds": {"type": "number"}}, "required": ["seconds"]}}},
]

PICK_TILE_SCHEMA = {"type": "object", "properties": {"tile": {"type": ["integer", "null"]}, "confidence": {"type": "number"},
                                                     "reason": {"type": "string"}},
                    "required": ["tile", "confidence", "reason"]}
FIND_STEP_SCHEMA = {"type": "object",
                    "properties": {"found_id": {"type": ["integer", "null"]}, "confidence": {"type": "number"},
                                   "turn": {"type": "string", "enum": ["left", "right", "up", "down", "none"]},
                                   "degrees": {"type": "number"}, "reason": {"type": "string"}},
                    "required": ["found_id", "confidence", "turn", "degrees", "reason"]}
GROUND_SCHEMA = {"type": "object",
                 "properties": {"found": {"type": "boolean"}, "box": {"type": "array", "items": {"type": "number"}},
                                "confidence": {"type": "number"}, "reason": {"type": "string"}},
                 "required": ["found", "box", "confidence", "reason"]}
PICK_ID_SCHEMA = {"type": "object", "properties": {"id": {"type": ["integer", "null"]}, "confidence": {"type": "number"},
                                                   "reason": {"type": "string"}},
                  "required": ["id", "confidence", "reason"]}


def annotate(frame, tracks, target=None):
    out = frame.copy()
    for t in tracks:
        x1, y1, x2, y2 = t["bbox"]
        col = (40, 40, 255) if t["id"] == target else ((60, 220, 60) if t.get("cls", "person") == "person" else (0, 200, 255))
        cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
        label = f"#{t['id']} {t.get('cls', '')}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(out, (x1, max(0, y1 - th - 8)), (x1 + tw + 6, y1), col, -1)
        cv2.putText(out, label, (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def contact_sheet(tiles, cols=3, tile_w=426, tile_h=320):
    """tiles: list of (label, bgr). Grid image with the label drawn on each tile."""
    cells = []
    for label, img in tiles:
        c = cv2.resize(img, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
        cv2.rectangle(c, (0, 0), (tile_w - 1, 34), (0, 0, 0), -1)
        cv2.putText(c, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cells.append(c)
    while len(cells) % cols:
        cells.append(np.zeros((tile_h, tile_w, 3), np.uint8))
    rows = [cv2.hconcat(cells[i:i + cols]) for i in range(0, len(cells), cols)]
    return cv2.vconcat(rows)


def jpeg(img, q=80):
    ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return enc.tobytes() if ok else None


ACTION_WORDS = ("target", "track", "follow", "aim at", "aim", "find", "search", "locate", "cycle", "go through",
                "stop", "hold", "home", "look for", "point at", "center on", "show me the")
TERMINAL_TOOLS = {"track", "find", "aim_at", "cycle", "stop", "home"}


class Agent:
    _CLASSES = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
                "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
                "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
                "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
                "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
                "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
                "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                "teddy bear", "hair drier", "toothbrush"}

    def __init__(self, cmd, model, ollama_url, max_steps=10):
        self.cmd, self.model, self.url, self.max_steps = cmd, model, ollama_url, max_steps
        self.running = False
        self.transcript = []
        self.history = []
        self.lock = threading.Lock()

    # ----- observation helpers -----
    def observe(self, classes=None, note=""):
        frame, tracks, _ = self.cmd.wait_new_frame(1.0)
        if frame is None:
            return {"error": "no video"}, []
        tracks = [t for t in tracks if self.cmd.class_ok(t, classes)]
        tracks.sort(key=lambda t: -(t["bbox"][2] - t["bbox"][0]) * (t["bbox"][3] - t["bbox"][1]))
        det = []
        for t in tracks[:12]:
            x1, y1, x2, y2 = t["bbox"]
            cx = (x1 + x2) / 2 / self.cmd.w
            pos = "left" if cx < 0.35 else ("right" if cx > 0.65 else "center")
            h = (y2 - y1) / self.cmd.h
            size = "large (close)" if h > 0.6 else ("medium" if h > 0.3 else "small (far)")
            det.append({"id": t["id"], "class": t.get("cls", "person"), "position": pos, "size": size,
                        "conf": t["conf"], "targeted": t["id"] == self.cmd.target})
        arm = self.cmd.arm["pos"] or {}
        result = {"detections": det, "n_total": len(tracks), "pan_deg": arm.get("pan"), "tilt_deg": arm.get("tilt"),
                  "target": self.cmd.target, "phase": self.cmd.phase}
        if note:
            result["note"] = note
        img = jpeg(annotate(frame, tracks[:12], self.cmd.target), 75)
        return result, [("picture (boxes labelled #id class)", img)] if img else []

    # ----- model calls -----
    def _chat(self, messages, tools=True, fmt=None, num_predict=400):
        body = {"model": self.model, "stream": False, "keep_alive": "30m",
                "options": {"temperature": 0, "num_predict": num_predict}, "messages": messages}
        if tools:
            body["tools"] = AGENT_TOOLS
        if fmt:
            body["format"] = fmt
        req = urllib.request.Request(self.url + "/api/chat", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=180) as r:
            return json.loads(r.read())["message"]

    def ask_json(self, prompt, images, schema):
        msg = self._chat([{"role": "user", "content": prompt + " Keep 'reason' under 12 words.",
                           "images": [base64.standard_b64encode(j).decode() for j in images]}],
                         tools=False, fmt=schema, num_predict=300)
        txt = msg.get("content") or "{}"
        try:
            return json.loads(txt)
        except ValueError:
            import re
            out = {}
            for key in ("tile", "id"):
                m = re.search(r'"%s"\s*:\s*(null|-?\d+)' % key, txt)
                if m:
                    out[key] = None if m.group(1) == "null" else int(m.group(1))
            m = re.search(r'"confidence"\s*:\s*([0-9.]+)', txt)
            out["confidence"] = float(m.group(1)) if m else 0.0
            out["reason"] = "(truncated answer)"
            return out

    @staticmethod
    def class_of(description):
        """'a laptop' / 'the person' / 'chairs' -> COCO class name, else None."""
        d = description.lower().strip(" .!?")
        for art in ("a ", "an ", "the ", "some ", "any "):
            if d.startswith(art):
                d = d[len(art):]
        d = d.rstrip("s") if d.endswith("s") and d[:-1] in Agent._CLASSES else d
        return d if d in Agent._CLASSES else None

    # ----- grounding: where is X in this picture? -----
    def ground(self, description, jpg):
        ans = self.ask_json(
            f"Locate: \"{description}\". If it is visible, give its bounding box as [x1,y1,x2,y2] normalized 0-1 "
            "relative to the image width/height (x=0 left, x=1 right, y=0 top, y=1 bottom). If not visible: "
            "found=false, box=[].", [jpg], GROUND_SCHEMA)
        box = ans.get("box") or []
        if not ans.get("found") or len(box) != 4:
            return None, float(ans.get("confidence") or 0), ans.get("reason", "")
        box = [float(v) for v in box]
        if max(box) > 1.5:                                  # the model used a 0-1000 grid or pixels
            scale = 1000.0 if max(box) <= 1000 else float(max(self.cmd.w, self.cmd.h))
            box = [v / scale for v in box]
        box = [min(1.0, max(0.0, v)) for v in box]
        if box[2] - box[0] < 0.01 or box[3] - box[1] < 0.01:
            return None, 0.0, "degenerate box"
        return box, float(ans.get("confidence") or 0), ans.get("reason", "")

    def _det_for_box(self, box, tracks):
        """A detection whose box overlaps the grounded region well (so we can track it instead)."""
        w, h = self.cmd.w, self.cmd.h
        pb = [box[0] * w, box[1] * h, box[2] * w, box[3] * h]
        best, best_iou = None, 0.35
        for t in tracks:
            o = iou(pb, t["bbox"])
            if o > best_iou:
                best, best_iou = t, o
        return best

    def aim_region(self, description, box, tracks, pic):
        """Center a grounded region: track its detection if there is one, else point and hold."""
        c = self.cmd
        det = self._det_for_box(box, tracks)
        if det is not None:
            c.release()
            c.track(det["id"])
            return {"found": True, "id": det["id"], "class": det.get("cls"), "how": "tracking detection",
                    "note": f"now tracking #{det['id']}; nothing more to do"}, [("found", pic)]
        goal = c.region_goal(box)
        if goal is None:
            return {"found": False, "error": "arm position unknown"}, []
        g = c.goto(goal[0], goal[1], speed=c.a.goto_speed, label=f"aiming at {description}")
        c.wait_settled(g, timeout=8.0)
        frame, _, _ = c.wait_new_frame(1.0)
        jpg2 = jpeg(frame, 75) if frame is not None else None
        box2, conf2, _ = self.ground(description, jpg2) if jpg2 else (None, 0.0, "")
        if box2 is not None:
            cx, cy = (box2[0] + box2[2]) / 2, (box2[1] + box2[3]) / 2
            if abs(cx - 0.5) > 0.12 or abs(cy - 0.5) > 0.12:           # one refinement step
                goal = c.region_goal(box2)
                g = c.goto(goal[0], goal[1], speed=c.a.goto_speed, label=f"aiming at {description}")
                c.wait_settled(g, timeout=6.0)
                frame, _, _ = c.wait_new_frame(1.0)
                jpg2 = jpeg(frame, 75) if frame is not None else jpg2
                box3, _, _ = self.ground(description, jpg2) if jpg2 else (None, 0, "")
                box2 = box3 or box2
        c.hold_region(description, box2 or [0.4, 0.4, 0.6, 0.6])
        return {"found": True, "how": "aimed at region", "region": [round(v, 2) for v in (box2 or box)],
                "pan_deg": round(c.arm["pos"]["pan"]) if c.arm["pos"] else None,
                "note": f"camera centered on {description} and holding; nothing more to do"}, \
            [("aimed", jpg2 or pic)]

    def do_aim_at(self, description):
        c = self.cmd
        frame, tracks, _ = c.wait_new_frame(1.0)
        if frame is None:
            return {"found": False, "error": "no video"}, []
        pic = jpeg(frame, 75)
        box, conf, reason = self.ground(description, pic)
        if box is None or conf < 0.4:
            return {"found": False, "reason": reason or "not in the current picture", "hint": "use find to look around"}, [("view", pic)]
        return self.aim_region(description, box, tracks, pic)

    # ----- compound: find (look, decide, turn, look ... no sweeping) -----
    def _uncovered(self, covered, lo, hi):
        """Pan intervals inside [lo, hi] not yet covered (list of (a, b))."""
        segs, cur = [], lo
        for a, b in sorted(covered):
            if b <= cur:
                continue
            if a > cur:
                segs.append((cur, min(a, hi)))
            cur = max(cur, b)
            if cur >= hi:
                break
        if cur < hi:
            segs.append((cur, hi))
        return [(a, b) for a, b in segs if b - a > 8]

    def do_find(self, description, classes=None):
        """Manual-style search driven by the VLM: at each step ground the description in the current
        picture (anything visible counts, not only detector classes); if absent, ask which way to turn
        and by how much, turn, look again. Coverage is tracked in code so it never re-checks a view
        and never pushes into a joint stop. No sweeping."""
        c = self.cmd
        lim, pos = c.arm["limits"], c.arm["pos"]
        if not lim or not pos:
            return {"found": False, "error": "arm position unknown"}, []
        half = c.a.hfov / 2
        lo, hi = -lim["pan"] + 2, lim["pan"] - 2
        covered = {}                                             # tilt band -> [(pan_lo, pan_hi)]
        plain = self.class_of(description)
        if plain and not classes:
            classes = [plain]
        last_pic = None
        c.busy = "searching"
        try:
            for step in range(c.a.find_steps):
                frame, tracks, _ = c.wait_new_frame(1.0)
                if frame is None:
                    continue
                pos = c.arm["pos"]
                cand = [t for t in tracks if c.class_ok(t, classes)]
                band = int(round(pos["tilt"] / 25.0))
                covered.setdefault(band, []).append((pos["pan"] - half * 0.8, pos["pan"] + half * 0.8))
                pic = jpeg(annotate(frame, cand), 75)
                last_pic = pic
                if plain and cand:                               # plain class: the detector already answers
                    best = max(cand, key=lambda t: (t["bbox"][2] - t["bbox"][0]) * (t["bbox"][3] - t["bbox"][1]))
                    self.transcript.append({"t": "step", "text": f"step {step + 1}: pan {pos['pan']:+.0f} tilt {pos['tilt']:+.0f} -> {plain} #{best['id']} in view"})
                    c.release()
                    c.track(best["id"])
                    return {"found": True, "id": best["id"], "steps": step + 1, "note": f"now tracking #{best['id']}; nothing more to do"}, [("found", pic)]
                # 1) is it in this picture? (grounding: works for anything visible)
                box, conf, reason = self.ground(description, jpeg(frame, 75))
                if box is not None and conf >= 0.5:
                    self.transcript.append({"t": "step", "text": f"step {step + 1}: pan {pos['pan']:+.0f} tilt {pos['tilt']:+.0f} -> in view ({reason[:60]}), aiming"})
                    res, imgs = self.aim_region(description, box, cand if classes else tracks, pic)
                    res["steps"] = step + 1
                    return res, imgs
                # 2) a candidate cut off at a side edge: turn toward it before deciding
                edge_seen = getattr(self, "_edge_seen", set())
                edge = next((t for t in cand if (t["bbox"][0] <= 3 or t["bbox"][2] >= c.w - 3) and t["id"] not in edge_seen), None)
                if edge is not None:
                    edge_seen.add(edge["id"])
                    self._edge_seen = edge_seen
                    side = -1 if edge["bbox"][0] <= 3 else 1
                    self.transcript.append({"t": "step", "text": f"step {step + 1}: #{edge['id']} cut off at the {'left' if side < 0 else 'right'} edge, turning to see it"})
                    g = c.goto(float(np.clip(pos["pan"] + side * 18, lo, hi)), pos["tilt"], speed=c.a.goto_speed, label="searching")
                    c.wait_settled(g, timeout=6.0)
                    continue
                # 3) not here: which way? (the model chooses from what it sees; code keeps it honest)
                unc = self._uncovered(covered.get(band, []), lo, hi)
                unc_txt = ", ".join(f"{a:+.0f}..{b:+.0f}" for a, b in unc) or "none at this tilt"
                ans = self.ask_json(
                    f"You are looking for: \"{description}\" and it is NOT in this picture ({reason[:60]}). Camera pan "
                    f"{pos['pan']:+.0f} deg (limits {lo:+.0f}..{hi:+.0f}), tilt {pos['tilt']:+.0f} deg (+ = down, range "
                    f"{-lim['tilt'] + 2:+.0f}..{lim['tilt'] - 2:+.0f}). Pan ranges NOT yet checked at this tilt: {unc_txt}. "
                    "From what you see (walls, openings, tables, where such a thing would be mounted or placed), choose "
                    "ONE move: turn left/right/up/down by 10-60 degrees. Prefer unchecked ranges; if the target is likely "
                    "higher or lower than the current view, use up/down. "
                    "Answer JSON {found_id: null, confidence: 0, turn, degrees, reason}.", [pic], FIND_STEP_SCHEMA)
                turn, deg = ans.get("turn", "none"), float(np.clip(float(ans.get("degrees") or 30), 10, 60))
                self.transcript.append({"t": "step", "text": f"step {step + 1}: pan {pos['pan']:+.0f} tilt {pos['tilt']:+.0f} -> "
                                        f"turn {turn} {deg:.0f} ({ans.get('reason', '')[:70]})"})
                new_pan, new_tilt = pos["pan"], pos["tilt"]
                if turn == "left":
                    new_pan -= deg
                elif turn == "right":
                    new_pan += deg
                elif turn == "up":
                    new_tilt -= deg
                elif turn == "down":
                    new_tilt += deg
                new_pan = float(np.clip(new_pan, lo, hi))
                new_tilt = float(np.clip(new_tilt, -lim["tilt"] + 2, lim["tilt"] - 2))
                if turn in ("left", "right", "none"):
                    stuck = abs(new_pan - pos["pan"]) < 5
                    in_cov = any(a <= new_pan <= b for a, b in covered.get(band, []))
                    if stuck or in_cov or turn == "none":
                        if unc:                                  # steer to the nearest unexplored pan range
                            a, b = min(unc, key=lambda r: min(abs(r[0] - pos["pan"]), abs(r[1] - pos["pan"])))
                            new_pan = float(np.clip((a + b) / 2, lo, hi))
                        else:                                    # this tilt is exhausted: change tilt band
                            new_tilt = float(np.clip(pos["tilt"] + (-25 if band >= 0 else 25), -lim["tilt"] + 2, lim["tilt"] - 2))
                            if abs(new_tilt - pos["tilt"]) < 5:
                                break
                elif abs(new_tilt - pos["tilt"]) < 5:            # up/down into a tilt stop: pick an unexplored pan instead
                    if unc:
                        a, b = unc[0]
                        new_pan = float(np.clip((a + b) / 2, lo, hi))
                    else:
                        break
                g = c.goto(new_pan, new_tilt, speed=c.a.goto_speed, label=f"searching {new_pan:+.0f}/{new_tilt:+.0f}")
                c.wait_settled(g, timeout=8.0)
            return {"found": False, "reason": "looked around without seeing it", "steps": c.a.find_steps}, \
                ([("last view", last_pic)] if last_pic else [])
        finally:
            c.release()

    # ----- tool dispatch -----
    def _call(self, name, args):
        c = self.cmd
        classes = args.get("classes") or None
        if name == "look":
            return self.observe(classes)
        if name in ("look_left", "look_right", "look_up", "look_down"):
            d = float(args.get("degrees") or (30 if name in ("look_left", "look_right") else 20))
            args = {"pan_deg": (-d if name == "look_left" else d if name == "look_right" else 0),
                    "tilt_deg": (-d if name == "look_up" else d if name == "look_down" else 0)}
            name = "turn"
        if name == "turn":
            pos = c.arm["pos"] or {"pan": 0.0, "tilt": 0.0}
            dp = float(np.clip(float(args.get("pan_deg") or 0), -60, 60))
            dt = float(np.clip(float(args.get("tilt_deg") or 0), -40, 40))
            g = c.goto(pos["pan"] + dp, pos["tilt"] + dt, label="turning")
            settled = c.wait_settled(g)
            c.release()
            return self.observe(classes, note="" if settled else "did not fully settle (joint limit?)")
        if name == "aim_at":
            return self.do_aim_at(str(args.get("description", "")))
        if name == "inspect":
            frame, tracks, _ = c.snapshot()
            t = next((x for x in tracks if x["id"] == int(args["id"])), None)
            if t is None or frame is None:
                return {"ok": False, "error": "id not visible; call look"}, []
            return {"ok": True, "id": t["id"], "class": t.get("cls")}, [(f"crop of #{t['id']}", crop_person(frame, t["bbox"], 512))]
        if name == "track":
            tid = int(args["id"])
            _, tracks, _ = c.snapshot()
            if not any(t["id"] == tid for t in tracks):
                vis = [f"#{t['id']} {t.get('cls', '')}" for t in sorted(tracks, key=lambda t: -(t["bbox"][2] - t["bbox"][0]) * (t["bbox"][3] - t["bbox"][1]))[:10]]
                return {"ok": False, "error": f"id {tid} is not visible; visible now: {', '.join(vis) or 'nothing'}. "
                                              "Pick one of these ids."}, []
            c.track(tid)
            return {"ok": True, "tracking": tid}, []
        if name == "find":
            return self.do_find(str(args.get("description", "")), classes)
        if name == "cycle":
            c.set_instruction(str(args.get("description") or ""), classes)
            c.start("loop" if args.get("mode") == "loop" else "once")
            return {"ok": True, "mode": c.mode, "rule": c.instruction, "classes": classes}, []
        if name == "stop":
            c.stop()
            return {"ok": True}, []
        if name == "home":
            if not c.home:
                return {"ok": False, "error": "no saved posture (use Save posture in the UI)"}, []
            g = c.goto(None, None, posture=True, label="going home")
            c.wait_settled(g, timeout=15.0)
            c.release()
            return {"ok": True}, []
        if name == "wait":
            time.sleep(float(np.clip(float(args.get("seconds", 2)), 0, 10)))
            return self.observe(classes)
        return {"ok": False, "error": f"unknown tool {name}"}, []

    # ----- loop -----
    def run(self, text):
        with self.lock:
            if self.running:
                return False
            self.running = True
        threading.Thread(target=self._run, args=(text,), daemon=True).start()
        return True

    def _run(self, text):
        self.transcript = [{"t": "user", "text": text}]
        messages = [{"role": "system", "content": AGENT_SYSTEM}, {"role": "user", "content": text}]
        low = text.lower().strip(" .!")
        action_cmd = any(w in low for w in ACTION_WORDS)
        last_sig, repeats, nudges = None, 0, 0

        def finish(answer):
            self.transcript.append({"t": "answer", "text": answer})

        # fast paths: no model needed
        if low in ("stop", "hold", "halt", "freeze", "stop tracking", "stop it"):
            self.cmd.stop()
            self.transcript.append({"t": "call", "name": "stop", "args": {}})
            finish("Stopped, holding position.")
            self.running = False
            return
        if low in ("home", "go home"):
            self.transcript.append({"t": "call", "name": "home", "args": {}})
            result, _ = self._call("home", {})
            finish("Back at the home posture." if result.get("ok") else result.get("error", "cannot go home"))
            self.cmd.release()
            self.running = False
            return

        def final_from_model(extra=None):
            if extra:
                messages.append({"role": "user", "content": extra})
            msg2 = self._chat(messages, tools=False, num_predict=200)
            finish((msg2.get("content") or "").strip() or "(no answer)")

        try:
            for _ in range(self.max_steps):
                msg = self._chat(messages)
                calls = msg.get("tool_calls") or []
                if not calls:
                    answer = (msg.get("content") or "").strip() or "(no answer)"
                    lw = answer.lower()
                    intent = any(k in lw for k in ("i will", "i'll", "let me", "going to", "i am going"))
                    if action_cmd and nudges < 2:      # an action was asked and none completed yet: insist
                        nudges += 1
                        self.transcript.append({"t": "nudge", "text": answer})
                        messages.append({"role": "assistant", "content": answer})
                        messages.append({"role": "user", "content": "The command asks for an action and it has not been "
                                         "completed. Do it now with the tools (track/find/cycle/stop) using ids from the "
                                         "latest look, then report."})
                        continue
                    finish(answer)
                    return
                sig = json.dumps([(cl.get("function", {}).get("name"), cl.get("function", {}).get("arguments")) for cl in calls],
                                 sort_keys=True, default=str)
                repeats = repeats + 1 if sig == last_sig else 0
                last_sig = sig
                if repeats >= 1:
                    final_from_model("Stop repeating that call. Answer now in one or two sentences with what you found or did.")
                    return
                messages.append({"role": "assistant", "content": msg.get("content") or "", "tool_calls": calls})
                images, done = [], None
                for call in calls:
                    fn = call.get("function", {})
                    name, args = fn.get("name"), fn.get("arguments") or {}
                    if isinstance(args, str):
                        try:
                            args = json.loads(args)
                        except ValueError:
                            args = {}
                    self.transcript.append({"t": "call", "name": name, "args": args})
                    if name in TERMINAL_TOOLS and not action_cmd:
                        result, imgs = {"ok": False, "error": "the command only asked to look; not executing an action"}, []
                    else:
                        try:
                            result, imgs = self._call(name, args)
                        except Exception as e:  # noqa: BLE001
                            result, imgs = {"ok": False, "error": f"{type(e).__name__}: {e}"}, []
                    self.transcript.append({"t": "result", "name": name, "result": result})
                    messages.append({"role": "tool", "tool_name": name, "content": json.dumps(result)})
                    images += [(lbl, j) for lbl, j in imgs if j]
                    # terminal actions: the harness composes the answer, the model is done
                    if name == "track" and result.get("ok"):
                        _, tr, _ = self.cmd.snapshot()
                        cls = next((t.get("cls", "") for t in tr if t["id"] == result["tracking"]), "")
                        done = f"Tracking #{result['tracking']} {cls}.".replace(" .", ".")
                    elif name in ("find", "aim_at") and result.get("found"):
                        if result.get("id") is not None:
                            done = f"Found it. Tracking #{result['id']} {result.get('class') or ''}.".replace(" .", ".")
                        else:
                            done = f"Found it. Camera centered on it and holding (pan {result.get('pan_deg')}°)."
                    elif name == "cycle" and result.get("ok"):
                        done = f"Cycling ({result['mode']}) through {result.get('rule') or 'everything'}" + \
                               (f" [{', '.join(result['classes'])}]." if result.get("classes") else ".")
                    elif name == "stop" and result.get("ok"):
                        done = "Stopped, holding position."
                    elif name == "home" and result.get("ok"):
                        done = "Back at the home posture."
                if images:
                    labels = "; ".join(f"image {i + 1}: {lbl}" for i, (lbl, _) in enumerate(images))
                    messages.append({"role": "user", "content": f"Attached: {labels}.",
                                     "images": [base64.standard_b64encode(j).decode() for _, j in images]})
                    self.transcript.append({"t": "images", "labels": [lbl for lbl, _ in images]})
                if done:
                    finish(done)
                    return
                if not action_cmd:
                    # observation command: one look is the whole job; now describe the picture
                    final_from_model("Now answer the command in one to three sentences based on the picture.")
                    return
            self.transcript.append({"t": "error", "text": f"stopped after {self.max_steps} steps"})
        except Exception as e:  # noqa: BLE001
            self.transcript.append({"t": "error", "text": f"{type(e).__name__}: {e}"})
        finally:
            self.cmd.release()
            self.running = False


# =====================================================================================
# UI
# =====================================================================================

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Target Commander</title>
<style>
:root{--bg:#0d0f12;--panel:#161a20;--line:#262c36;--fg:#e6e9ee;--dim:#8a93a2;--ok:#3ddc84;--tgt:#ff4d4d;--warn:#ffcc33}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 -apple-system,Segoe UI,Inter,sans-serif;height:100vh;display:grid;grid-template-columns:360px 1fr 340px}
#chat{background:var(--panel);border-right:1px solid var(--line);padding:16px;display:flex;flex-direction:column;gap:10px;min-height:0}
#cmdlog{flex:1;overflow:auto;font-size:13px;line-height:1.5;display:flex;flex-direction:column;gap:6px}
.msg{padding:8px 10px;border-radius:10px;white-space:pre-wrap;word-break:break-word}.msg.you{background:#22303f;align-self:flex-end}.msg.arm{background:#15412b}.msg.call{color:var(--warn);font-size:12px;padding:2px 10px}.msg.res{color:var(--dim);font-size:12px;padding:0 10px 4px}.msg.err{color:var(--tgt)}.msg.img{color:var(--dim);font-size:12px;padding:0 10px}
main{display:flex;align-items:center;justify-content:center;background:#000}main img{max-width:100%;max-height:100vh;cursor:crosshair}
aside{background:var(--panel);border-left:1px solid var(--line);padding:16px;display:flex;flex-direction:column;gap:14px;overflow:auto}
h1{font-size:13px;margin:0;letter-spacing:.06em;text-transform:uppercase;color:var(--dim)}
textarea{width:100%;background:#0b0d10;color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:10px;font:inherit;resize:vertical}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}button{background:#22303f;color:var(--fg);border:1px solid #2f4257;border-radius:8px;padding:8px 12px;font:inherit;cursor:pointer}
button.primary{background:#2b6cb0;border-color:#3b82f6}button.stop{background:#5a2330;border-color:#a33}button.active{outline:2px solid var(--ok)}
.status{font-size:13px;padding:8px 10px;border:1px solid var(--line);border-radius:8px;background:#0b0d10}.hint{color:var(--dim);font-size:12px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font-size:13px}.kv b{color:var(--dim);font-weight:500}
.det{display:flex;gap:8px;align-items:center;padding:6px 8px;border:1px solid var(--line);border-radius:8px;background:#11151b;cursor:pointer;font-size:13px}
.det.target{border-color:var(--tgt)}.det img{width:36px;height:48px;object-fit:cover;border-radius:4px;background:#000}
.tag{font-size:11px;padding:2px 6px;border-radius:999px;background:#222a35;color:var(--dim)}.tag.ok{background:#15412b;color:var(--ok)}.tag.tgt{background:#4a1d1d;color:var(--tgt)}
details summary{cursor:pointer;color:var(--dim);font-size:13px}
</style></head><body>
<section id="chat">
<h1>Command <span class="hint" style="text-transform:none;letter-spacing:0">— talk to the arm</span></h1>
<div id="cmdlog"></div>
<textarea id="cmdtext" rows="3" placeholder='"target the man in the white shirt" · "find the banner with the QR code" · "aim at the door" · "what do you see?" · "look left 40" · "cycle through everyone wearing glasses" · "stop"'></textarea>
<div class="row"><button class="primary" id="cmdbtn" onclick="sendCmd()">Send</button><span class="hint" id="cmdstate"></span></div>
</section>
<main><img id="video" alt="stream" onclick="videoClick(event)"></main>
<aside>
<h1>Status</h1>
<div class="status" id="status"></div>
<div class="row"><button class="stop" onclick="ctl({mode:'off'})">Stop</button><button onclick="post('/aim',{posture:'save'})">Save posture</button><button onclick="post('/aim',{posture:true})">Go home</button></div>
<h1>Look <span class="hint" style="text-transform:none;letter-spacing:0">— manual, <span id="stepv">20</span>°</span></h1>
<div class="row" style="justify-content:center"><button onclick="look(-1,0)" title="left">◀</button><div style="display:flex;flex-direction:column;gap:6px"><button onclick="look(0,-1)" title="up">▲</button><button onclick="look(0,1)" title="down">▼</button></div><button onclick="look(1,0)" title="right">▶</button><input type="range" id="step" min="5" max="60" step="5" value="20" style="width:90px" oninput="document.getElementById('stepv').textContent=this.value"></div>
<canvas id="pan" width="308" height="120" style="width:100%;background:#0b0d10;border:1px solid var(--line);border-radius:8px"></canvas>
<h1>Find</h1>
<div class="row"><input id="findtext" placeholder='e.g. "person in a red jacket", "a laptop"' style="flex:1;background:#0b0d10;color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:8px;font:inherit"><button class="primary" onclick="quickFind()">Find</button></div>
<h1>Cycle rule</h1>
<textarea id="rule" rows="2" placeholder='e.g. "people in white shirts", "cups" (leave empty = everything)'></textarea>
<div class="row"><button class="primary" onclick="applyRule()">Apply</button><button id="b-once" onclick="ctl({mode:'once'})">Cycle once</button><button id="b-loop" onclick="ctl({mode:'loop'})">Loop</button><button onclick="ctl({next:true})">Skip</button></div>
<label class="hint">Dwell on locked target: <span id="dwellv"></span>s<input type="range" id="dwell" min="0.2" max="5" step="0.1" style="width:100%" onchange="ctl({dwell_s:+this.value})"></label>
<details><summary>Detections <span id="detsum"></span> — click to track</summary><div id="dets" style="display:flex;flex-direction:column;gap:6px;margin-top:8px"></div></details>
<h1>Vision model</h1><div class="kv" id="vlm"></div><div class="hint" id="vlmerr"></div>
<div class="hint">Video: <span id="vsrc"></span></div>
</aside>
<script>
const q=new URLSearchParams(location.search);const VIDEO=q.get('video')||'http://127.0.0.1:8099/stream';
const video=document.getElementById('video');video.src=VIDEO;document.getElementById('vsrc').textContent=VIDEO;
let last=null,editing=false;const rule=document.getElementById('rule');rule.onfocus=()=>editing=true;rule.onblur=()=>editing=false;
async function post(path,body){await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});refresh();}
function applyRule(){post('/instruction',{text:rule.value})}function ctl(b){post('/control',b)}function aim(id){post('/aim',{id})}
function look(dx,dy){const st=+document.getElementById('step').value;post('/aim',{dpan:dx*st,dtilt:dy*st});}
function quickFind(){const t=document.getElementById('findtext').value.trim();if(!t)return;cmdtext.value='find '+t;sendCmd();}
document.getElementById('findtext').onkeydown=e=>{if(e.key==='Enter')quickFind();};
function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function videoClick(e){if(!last)return;const r=video.getBoundingClientRect();const sx=(video.naturalWidth||640)/r.width,sy=(video.naturalHeight||480)/r.height;
 const x=(e.clientX-r.left)*sx,y=(e.clientY-r.top)*sy;let best=null,area=1e12;
 for(const p of last.detections){const[b0,b1,b2,b3]=p.bbox;if(x>=b0&&x<=b2&&y>=b1&&y<=b3){const a=(b2-b0)*(b3-b1);if(a<area){area=a;best=p;}}}
 if(best)aim(best.id);}
const cmdtext=document.getElementById('cmdtext');cmdtext.onkeydown=e=>{if(e.key==='Enter'&&!e.shiftKey){e.preventDefault();sendCmd();}};
async function sendCmd(){const t=cmdtext.value.trim();if(!t)return;cmdtext.value='';await fetch('/agent',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({text:t})});pollAgent();}
let agentTimer=null;
async function pollAgent(){try{const a=await (await fetch('/agent')).json();
 document.getElementById('cmdstate').textContent=a.running?'working…':(a.model?('idle · '+a.model):'');document.getElementById('cmdbtn').disabled=a.running;
 document.getElementById('cmdlog').innerHTML=a.transcript.map(e=>{
  if(e.t==='user')return `<div class="msg you">${esc(e.text)}</div>`;
  if(e.t==='call')return `<div class="msg call">↳ ${esc(e.name)}(${esc(JSON.stringify(e.args))})</div>`;
  if(e.t==='result'){const r=JSON.stringify(e.result);return `<div class="msg res">= ${esc(r.length>240?r.slice(0,240)+'…':r)}</div>`;}
  if(e.t==='images')return `<div class="msg img">📷 ${esc(e.labels.join(', '))}</div>`;
  if(e.t==='nudge')return `<div class="msg res">(model said: ${esc(e.text)} — asked to act)</div>`;
  if(e.t==='step')return `<div class="msg call">🔎 ${esc(e.text)}</div>`;
  if(e.t==='answer')return `<div class="msg arm">${esc(e.text)}</div>`;
  return `<div class="msg err">${esc(e.text)}</div>`;}).join('')+(a.running?'<div class="msg res">working…</div>':'');
 const log=document.getElementById('cmdlog');log.scrollTop=log.scrollHeight;
 if(a.running){clearTimeout(agentTimer);agentTimer=setTimeout(pollAgent,600);}}catch(e){}}
function drawPan(s){const c=document.getElementById('pan'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);
 const lim=(s.arm.limits&&s.arm.limits.pan)||110,pos=s.arm.pos,cx=c.width/2,cy=c.height-6,R=cy-8;
 const A=a=>(a/lim)*Math.PI*0.5;g.strokeStyle='#262c36';g.beginPath();g.arc(cx,cy,R,Math.PI,2*Math.PI);g.stroke();
 for(const a of[-lim,-60,-30,0,30,60,lim]){const x=cx+R*Math.sin(A(a)),y=cy-R*Math.cos(A(a));g.fillStyle='#4a5261';g.font='10px sans-serif';g.fillText(Math.round(a)+'°',x-8,y-4);}
 if(pos){const h=s.fov[0]/2;g.fillStyle='rgba(59,130,246,.25)';g.beginPath();g.moveTo(cx,cy);for(let a=pos.pan-h;a<=pos.pan+h;a+=2){g.lineTo(cx+R*Math.sin(A(a)),cy-R*Math.cos(A(a)));}g.closePath();g.fill();
  g.strokeStyle='#3b82f6';g.lineWidth=2;g.beginPath();g.moveTo(cx,cy);g.lineTo(cx+R*Math.sin(A(pos.pan)),cy-R*Math.cos(A(pos.pan)));g.stroke();g.lineWidth=1;
  g.fillStyle='#e6e9ee';g.fillText(`pan ${pos.pan}°  tilt ${pos.tilt}°`,8,14);}}
function kv(el,obj){el.innerHTML=Object.entries(obj).map(([k,v])=>`<b>${k}</b><span>${v??'—'}</span>`).join('')}
async function refresh(){try{const s=await (await fetch('/state')).json();last=s;
 if(!editing)rule.value=s.instruction;
 for(const m of['once','loop'])document.getElementById('b-'+m).classList.toggle('active',s.mode===m);
 const tgt=s.target===null?'none':('#'+s.target);
 document.getElementById('status').innerHTML=`<b>${s.busy?esc(s.busy):(s.mode==='off'?(s.target!==null?'tracking':'idle'):(s.mode==='once'?'cycle (once)':'cycle (loop)'))}</b> · target ${tgt} · ${esc(s.phase)} (${s.phase_age_s}s)<br><span class="hint">video ${esc(s.tracker)} · arm ${esc(s.arm.mode||'?')} · ${Object.entries(s.counts).map(([k,v])=>v+' '+k).join(', ')||'nothing detected'}</span>`;
 const d=document.getElementById('dwell');if(document.activeElement!==d)d.value=s.dwell_s;document.getElementById('dwellv').textContent=s.dwell_s;
 drawPan(s);
 document.getElementById('detsum').textContent='('+s.detections.length+')';
 const html=s.detections.map(p=>`<div class="det ${p.is_target?'target':''}" onclick="aim(${p.id})"><img src="/crop/${p.id}?v=${Math.floor(Date.now()/3000)}" onerror="this.style.visibility='hidden'"><div style="flex:1"><b>#${p.id}</b> ${esc(p.cls)} ${p.is_target?'<span class="tag tgt">target</span>':''} ${p.match===true?'<span class="tag ok">match</span>':''}<div class="hint">conf ${p.conf}${p.reason?' · '+esc(p.reason):''}</div></div></div>`).join('');
 const box=document.getElementById('dets');if(box.dataset.html!==html){box.dataset.html=html;box.innerHTML=html||'<div class="hint">nothing detected</div>';}
 if(s.vlm){kv(document.getElementById('vlm'),{model:s.vlm.model,latency:s.vlm.last_ms?s.vlm.last_ms+' ms':'—',calls:s.vlm.calls,errors:s.vlm.errors});document.getElementById('vlmerr').textContent=s.vlm.last_error||'';}
}catch(e){}}
setInterval(refresh,500);refresh();pollAgent();
</script></body></html>"""


def make_handler(cmd):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, "application/json", json.dumps(obj).encode())

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":
                self._send(200, "text/html; charset=utf-8", PAGE.encode())
            elif p == "/state":
                self._json(cmd.state())
            elif p == "/aim":
                if cmd.manual_goto is not None:
                    self._json(dict(cmd.manual_goto, id=None, phase=cmd.phase, mode=cmd.mode))
                else:
                    self._json({"id": cmd.target, "phase": cmd.phase, "mode": cmd.mode})
            elif p.startswith("/crop/"):
                frame, tracks, _ = cmd.snapshot()
                try:
                    t = next((x for x in tracks if x["id"] == int(p[6:])), None)
                except ValueError:
                    t = None
                jpg = crop_person(frame, t["bbox"]) if (t and frame is not None) else None
                self._send(200 if jpg else 404, "image/jpeg", jpg or b"")
            elif p == "/agent":
                ag = cmd.agent
                self._json({"running": bool(ag and ag.running), "transcript": ag.transcript if ag else [],
                            "model": ag.model if ag else None})
            elif p == "/snap":
                jpg, _ = cmd.wait(-1)
                self._send(200 if jpg else 503, "image/jpeg", jpg or b"")
            elif p == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                seq = cmd.seq
                try:
                    while True:
                        jpg, seq = cmd.wait(seq)
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            p = self.path.split("?")[0]
            n = int(self.headers.get("Content-Length", 0))
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json({"error": "bad json"}, 400)
            if p == "/instruction":
                cmd.set_instruction(body.get("text", ""), body.get("classes"))
                self._json({"ok": True, "instruction": cmd.instruction})
            elif p == "/aim":
                if body.get("posture") == "save":
                    return self._json({"ok": True, "home": cmd.save_home()})
                if body.get("posture"):
                    cmd.goto(None, None, posture=True, label="going home")
                    threading.Timer(15.0, cmd.release).start()
                elif body.get("pan") is not None:
                    g = cmd.goto(float(body["pan"]), body.get("tilt"), label="turning")
                    threading.Timer(8.0, cmd.release).start()
                elif body.get("dpan") is not None or body.get("dtilt") is not None:
                    pos = cmd.arm["pos"] or {"pan": 0.0, "tilt": 0.0}
                    g = cmd.goto(pos["pan"] + float(body.get("dpan") or 0), pos["tilt"] + float(body.get("dtilt") or 0),
                                 label="turning")
                    threading.Timer(6.0, cmd.release).start()
                elif body.get("id") is None:
                    cmd.stop()
                else:
                    cmd.track(int(body["id"]))
                self._json({"ok": True, "target": cmd.target})
            elif p == "/control":
                if body.get("mode") in ("once", "loop"):
                    cmd.start(body["mode"])
                elif body.get("mode") == "off":
                    cmd.stop()
                for k in ("dwell_s", "lock_s", "timeout_s"):
                    if k in body:
                        setattr(cmd, k, float(body[k]))
                if body.get("next"):
                    cmd.skip_next = True
                self._json({"ok": True})
            elif p == "/agent":
                if cmd.agent is None:
                    return self._json({"error": "agent disabled"}, 400)
                started = cmd.agent.run(str(body.get("text", "")).strip())
                self._json({"ok": started, "busy": not started}, 200 if started else 409)
            else:
                self._json({"error": "not found"}, 404)

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tracker", default="http://127.0.0.1:8093")
    ap.add_argument("--follower", default="http://127.0.0.1:8094")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--model", default="qwen3-vl:30b-a3b-instruct",
                    help="VLM for the standing rule: an Ollama model (name with ':'), a Claude model id, or 'none'")
    ap.add_argument("--backend", choices=["auto", "anthropic", "ollama"], default="auto")
    ap.add_argument("--ollama-url", default="http://127.0.0.1:11434")
    ap.add_argument("--agent-model", default=None, help="Ollama model with tools+vision for the agent (default: --model)")
    ap.add_argument("--agent-steps", type=int, default=10)
    ap.add_argument("--env-file", default=None, help="optional KEY=VALUE file (e.g. ANTHROPIC_API_KEY for the Claude backend)")
    ap.add_argument("--recheck", type=float, default=8.0)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--crop-px", type=int, default=256)
    ap.add_argument("--min-conf", type=float, default=0.35)
    ap.add_argument("--continuity-s", type=float, default=2.5,
                    help="s after the target id vanishes during which an overlapping same-class detection is adopted")
    ap.add_argument("--min-age", type=float, default=0.3)
    ap.add_argument("--lock-err", type=float, default=0.12)
    ap.add_argument("--lock", type=float, default=0.3)
    ap.add_argument("--dwell", type=float, default=1.0)
    ap.add_argument("--timeout", type=float, default=4.0)
    ap.add_argument("--done-grace", type=float, default=2.0)
    ap.add_argument("--stale-s", type=float, default=1.0)
    ap.add_argument("--hfov", type=float, default=47.0)
    ap.add_argument("--vfov", type=float, default=36.0)
    ap.add_argument("--find-steps", type=int, default=8, help="max look-and-turn steps of the guided search")
    ap.add_argument("--goto-speed", type=float, default=50.0)
    ap.add_argument("--posture-speed", type=float, default=15.0)
    ap.add_argument("--draw-all", action="store_true", help="also outline non-target detections on the video")
    ap.add_argument("--push", default=None)
    ap.add_argument("--port", type=int, default=8100)
    ap.add_argument("--bind", default="127.0.0.1")
    a = ap.parse_args()
    load_env_file(a.env_file)
    cmd = Commander(a)
    agent_model = a.agent_model or (a.model if ":" in a.model else "qwen3-vl:30b-a3b-instruct")
    cmd.agent = Agent(cmd, agent_model, a.ollama_url, a.agent_steps) if agent_model != "none" else None
    srv = ThreadingHTTPServer((a.bind, a.port), make_handler(cmd))
    srv.daemon_threads = True
    print(f"commander v2 on http://{a.bind}:{a.port}/  tracker={a.tracker} rule-model={a.model} agent={agent_model}",
          flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
