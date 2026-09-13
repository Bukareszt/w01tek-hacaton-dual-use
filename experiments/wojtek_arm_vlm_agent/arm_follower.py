#!/usr/bin/env python3
"""Make the SO-101 follow a person: visual servoing on the wrist camera using tracker.py.

Reads /tracks from the tracker (default http://127.0.0.1:8093), selects one person
(sticky on track id, otherwise the largest box) and moves two joints so the person's
box center stays in the image center:

    horizontal error -> shoulder_pan
    vertical error   -> wrist_flex (camera tilt)

No excursion or rate limits: the goal is only clamped to the calibrated joint range.
If the current person disappears, the largest visible person is followed immediately.
Holds when nobody is visible; torque off on exit.
Default is --dry-run (prints what it would do). Use --execute to move the arm.

    arm_follower.py --dry-run
    arm_follower.py --execute --probe      # first run: learns joint signs automatically
    arm_follower.py --execute --sign-pan -1 --sign-tilt 1
"""

import argparse
import json
import os
import signal
from collections import deque
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen

import numpy as np

try:
    from feetech_bus import FeetechBus
except ImportError:  # running from another directory
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from feetech_bus import FeetechBus

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
PAN, TILT, LIFT, ELBOW = "shoulder_pan", "wrist_flex", "shoulder_lift", "elbow_flex"
GOTO_KEYS = {"pan": PAN, "tilt": TILT, "lift": LIFT, "elbow": ELBOW}

STATE = {"mode": "init", "target_id": None, "err": None, "pan": None, "tilt": None,
         "sign_pan": None, "sign_tilt": None, "n_tracks": 0, "updated": 0.0}
STATE_LOCK = threading.Lock()


def set_state(**kw):
    with STATE_LOCK:
        STATE.update(kw)
        STATE["updated"] = time.time()


def status_server(port, bind="127.0.0.1"):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            with STATE_LOCK:
                body = json.dumps(STATE).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer((bind, port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()


def read_tracks(url, max_age):
    tracks, _ = read_tracks_ts(url, max_age)
    return tracks


def read_tracks_ts(url, max_age):
    """(tracks, capture_ts); [] when the newest frame is older than max_age."""
    with urlopen(url + "/tracks", timeout=1.0) as r:
        d = json.loads(r.read())
    if time.time() - d["ts"] > max_age:
        return [], d["ts"]
    return d["tracks"], d["ts"]


def pos_at(hist, t):
    """Joint positions at time t, interpolated from a (time, pos) history."""
    if not hist:
        return None
    if t <= hist[0][0]:
        return hist[0][1]
    for (t0, p0), (t1, p1) in zip(hist, list(hist)[1:]):
        if t0 <= t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return {k: p0[k] + f * (p1[k] - p0[k]) for k in p0}
    return hist[-1][1]


def read_status(url):
    with urlopen(url + "/status", timeout=1.0) as r:
        return json.loads(r.read())


class TargetSelector:
    """Sticks to the current track id while it is visible; otherwise switches at once
    to the largest visible person."""

    def __init__(self):
        self.tid = None

    def pick(self, tracks, min_conf, keep_conf=None):
        keep_conf = min_conf if keep_conf is None else keep_conf
        for t in tracks:  # hysteresis: keep the current person down to keep_conf
            if t["id"] == self.tid and t["conf"] >= keep_conf:
                return t
        cand = [t for t in tracks if t["conf"] >= min_conf]
        if not cand:
            return None  # keep tid so the same person is preferred if they reappear
        t = max(cand, key=lambda t: (t["bbox"][2] - t["bbox"][0]) * (t["bbox"][3] - t["bbox"][1]))
        if t["id"] != self.tid:
            print(f"switching target -> id {t['id']}", flush=True)
        self.tid = t["id"]
        return t


def port_users(port):
    """PIDs other than ours that have the serial device open (via /proc)."""
    real = os.path.realpath(port)
    pids = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") == real:
                    pids.append(int(pid))
                    break
        except OSError:
            continue
    return pids


def norm_error(track, w, h, edge_push=0.35, aim_frac=0.4):
    """Normalized aim-point offset in [-1, 1].

    Vertically the aim point is the UPPER BODY (aim_frac down the box), not the box center:
    a close person's box is cut off at the bottom and its center jumps whenever the feet
    enter or leave the frame, which made the tilt oscillate at frame rate. The upper body
    stays put. Horizontally a box cut off by a side edge has a biased center, so the error is
    pushed toward that edge. A box spanning the whole frame on an axis gives zero error."""
    x1, y1, x2, y2 = track["bbox"]
    cx = (x1 + x2) / 2
    if track.get("cls", "person") != "person":
        aim_frac = 0.5                      # objects: aim at the box centre
    ay = y1 + aim_frac * (y2 - y1)
    ex, ey = (cx - w / 2) / (w / 2), (ay - h / 2) / (h / 2)
    if x1 <= 2 and x2 >= w - 2:
        ex = 0.0
    elif x1 <= 2:
        ex = min(ex, -edge_push)
    elif x2 >= w - 2:
        ex = max(ex, edge_push)
    if y1 <= 2 and y2 >= h - 2:
        ey = 0.0
    elif y1 <= 2:                      # head cut off at the top: gently look up
        ey = min(ey, -0.25)
    return ex, ey


class Arm:
    def __init__(self, port, execute, calibration=None):
        self.execute = execute
        self.port = port
        self.bus = FeetechBus(port, calibration)          # torch-free; LeRobot calibration file + units
        self.calibration = self.bus.calibration
        self.enabled = False
        self.half_range = {}

    def connect(self):
        self.bus.connect()
        if not self.bus.is_calibrated:
            raise RuntimeError("Stored calibration does not match motors")
        for name, cal in self.calibration.items():
            self.half_range[name] = (cal.range_max - cal.range_min) * 360 / 4095 / 2
        set_state(limits={"pan": round(self.half_range[PAN], 1), "tilt": round(self.half_range[TILT], 1),
                          "lift": round(self.half_range[LIFT], 1)})
        self.pos = self.read()
        self.goal = dict(self.pos)
        self.start = dict(self.pos)
        if self.execute:
            if any(self.bus.sync_read("Torque_Enable", normalize=False).values()):
                # Torque left on (e.g. previous controller was SIGKILLed). Only refuse if
                # some other process really holds the serial port.
                others = port_users(self.port)
                if others:
                    raise RuntimeError(f"Arm already powered and port held by pids {others}")
                print("warning: torque was already on with no other controller; taking over", flush=True)
            self.bus.sync_write("Goal_Position", self.goal)  # avoid old-goal jump
            self.bus.enable_torque()
            self.enabled = True

    def set_servo_smoothing(self, accel, p_coeff):
        """Gentler ramps inside the servo itself (registers are RAM, reset on power cycle)."""
        for name in (PAN, TILT, LIFT):
            if accel > 0:
                self.bus.write("Acceleration", name, accel)
            if p_coeff > 0:
                self.bus.write("P_Coefficient", name, p_coeff)
                self.bus.write("D_Coefficient", name, 32)

    def read(self):
        v = self.bus.sync_read("Present_Position")
        self.pos = {j: float(v[j]) for j in JOINTS}
        return self.pos

    def clamp(self, name, value, margin=1.0):
        """Only the calibrated joint range: beyond it the servo just hits its stop."""
        lim = self.half_range[name] - margin
        return float(np.clip(value, -lim, lim))

    def command(self, deltas, max_step=0.0):
        """deltas: {joint: desired change in degrees}. Writes goal (optional rate cap)."""
        for name, d in deltas.items():
            if max_step > 0:
                d = float(np.clip(d, -max_step, max_step))
            self.goal[name] = self.clamp(name, self.goal[name] + d)
        if self.execute:
            self.read()
            self.bus.sync_write("Goal_Position", self.goal)

    def set_goal(self, goals, min_move=0.0):
        """Absolute joint goals in degrees; a joint whose goal moves less than min_move is left alone."""
        changed = False
        for name, g in goals.items():
            g = self.clamp(name, g)
            if abs(g - self.goal[name]) >= min_move:
                self.goal[name] = g
                changed = True
        if self.execute and changed:
            self.bus.sync_write("Goal_Position", self.goal)
        return changed

    def close(self):
        try:
            if self.enabled:
                for attempt in range(3):
                    try:
                        # an interrupted transaction can leave the SDK's "port in use" flag set
                        ph = getattr(self.bus, "port_handler", None)
                        if ph is not None and getattr(ph, "is_using", False):
                            ph.is_using = False
                        self.bus.disable_torque()
                        break
                    except ConnectionError as e:
                        print(f"disable_torque attempt {attempt + 1} failed: {e}", flush=True)
                        time.sleep(0.2)
        finally:
            try:
                self.bus.disconnect(disable_torque=False)
            except Exception as e:  # noqa: BLE001
                print(f"disconnect: {e}", flush=True)


def probe_sign(arm, joint, tracker_url, selector, a, w, h):
    """Nudge one joint and see which way the target moves in the image. Returns +1/-1."""
    def measure():
        xs, ys = [], []
        for _ in range(8):
            t = selector.pick(read_tracks(tracker_url, a.max_track_age), a.min_conf)
            if t:
                ex, ey = norm_error(t, w, h)
                xs.append(ex)
                ys.append(ey)
            time.sleep(0.1)
        if len(xs) < 4:
            raise RuntimeError("Lost target during probe")
        return float(np.median(xs)), float(np.median(ys))

    # Nudge away from the nearer range limit so a saturated joint can still be probed.
    pos = arm.read()[joint]
    direction = -1.0 if pos > 0 else 1.0
    before = measure()
    for _ in range(int(a.probe_deg)):
        arm.command({joint: direction})
        time.sleep(1 / a.hz)
    time.sleep(0.8)
    after = measure()
    for _ in range(int(a.probe_deg)):
        arm.command({joint: -direction})
        time.sleep(1 / a.hz)
    axis = 0 if joint == PAN else 1
    shift = (after[axis] - before[axis]) * direction   # shift per +joint motion
    print(f"probe {joint}: {direction * a.probe_deg:+.0f} deg => target shift {after[axis] - before[axis]:+.3f}; "
          f"per +deg motion: {shift:+.3f}", flush=True)
    if abs(shift) < 0.05:
        raise RuntimeError(f"Probe on {joint} produced no clear shift ({shift:+.3f}); is the person moving?")
    # Positive joint motion moved the target toward +error => use negative gain to reduce error.
    return -1 if shift > 0 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tracker", default="http://127.0.0.1:8093")
    ap.add_argument("--port", default=os.environ.get("WOJTEK_ARM_SERIAL", "/dev/ttyACM0"),
                    help="SO-101 serial device (env WOJTEK_ARM_SERIAL; prefer a /dev/serial/by-id path)")
    ap.add_argument("--calibration", default=None,
                    help="LeRobot calibration JSON (default: env WOJTEK_ARM_CALIBRATION or LeRobot's so101.json)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--hz", type=float, default=50, help="control loop rate (polls the tracker; acts on new frames)")
    ap.add_argument("--control", choices=["absolute", "velocity"], default="absolute",
                    help="absolute: goal = joint angle that centers the person (fast, no hunting); "
                         "velocity: the older ramped velocity controller")
    ap.add_argument("--gain-pan", type=float, default=22.0,
                    help="deg of pan per unit of normalized x error (~half the horizontal FOV; probe measured ~23.5)")
    ap.add_argument("--gain-tilt", type=float, default=17.0, help="deg of tilt per unit of normalized y error (~half VFOV)")
    ap.add_argument("--goal-ema", type=float, default=0.6, help="goal low-pass per frame (1 = none)")
    ap.add_argument("--min-move", type=float, default=0.5, help="deg: ignore goal changes smaller than this (kills jitter)")
    ap.add_argument("--soft-zone", type=float, default=0.15, help="normalized error below which the correction is scaled down")
    ap.add_argument("--soft-gain", type=float, default=0.5, help="fraction of the correction applied at zero error")
    ap.add_argument("--lead", type=float, default=0.06, help="s of target-motion prediction added to the goal (0 = off)")
    ap.add_argument("--lead-max", type=float, default=3.0, help="deg cap on the feedforward lead")
    ap.add_argument("--lead-deadzone", type=float, default=6.0, help="deg/s of target motion below which no lead is applied")
    ap.add_argument("--cam-latency", type=float, default=0.05, help="s from exposure to the tracker's frame timestamp")
    ap.add_argument("--kp-pan", type=float, default=90.0, help="commanded deg/s at full-scale error")
    ap.add_argument("--kp-tilt", type=float, default=60.0, help="commanded deg/s at full-scale error")
    ap.add_argument("--max-vel", type=float, default=35.0, help="deg/s ceiling")
    ap.add_argument("--accel", type=float, default=70.0, help="deg/s^2 ramp for speeding up and slowing down")
    ap.add_argument("--ema", type=float, default=0.35, help="error low-pass: weight of the newest sample (1 = off)")
    ap.add_argument("--servo-accel", type=int, default=120, help="Feetech Acceleration register for pan/tilt (0 = leave)")
    ap.add_argument("--servo-p", type=int, default=24, help="Feetech P_Coefficient for pan/tilt (0 = leave)")
    ap.add_argument("--sign-pan", type=int, default=None, choices=[-1, 1])
    ap.add_argument("--sign-tilt", type=int, default=None, choices=[-1, 1])
    ap.add_argument("--sign-lift", type=int, default=None, choices=[-1, 1],
                    help="shoulder_lift takes over vertical tracking when wrist_flex hits its range (omit = wrist only)")
    ap.add_argument("--lift-range", type=float, default=25.0, help="max deg shoulder_lift may move from its start")
    ap.add_argument("--probe", action="store_true", help="learn signs by nudging joints (needs --execute)")
    ap.add_argument("--probe-deg", type=float, default=8.0)
    ap.add_argument("--deadband", type=float, default=0.06, help="soft deadband: gain fades to zero inside it")
    ap.add_argument("--min-conf", type=float, default=0.5, help="confidence to acquire a new person")
    ap.add_argument("--keep-conf", type=float, default=0.3, help="confidence to keep the current person")
    ap.add_argument("--max-track-age", type=float, default=0.5)
    ap.add_argument("--status-port", type=int, default=8094)
    ap.add_argument("--status-bind", default="127.0.0.1", help="0.0.0.0 when the commander runs on another host")
    ap.add_argument("--aim-url", default=None,
                    help="e.g. http://127.0.0.1:8100/aim - a commander decides what to do: {\"id\": N} aim at a "
                         "track, {\"pan\": deg, \"tilt\": deg, \"speed\": deg/s} go to absolute joint goals, "
                         "{\"id\": null} hold; replaces the built-in target selection")
    ap.add_argument("--kp-goto", type=float, default=2.5, help="deg/s per deg of joint error in goto mode")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--execute", action="store_true")
    g.add_argument("--dry-run", action="store_true", default=True)
    a = ap.parse_args()
    if a.probe and not a.execute:
        ap.error("--probe requires --execute")

    status_server(a.status_port, a.status_bind)
    st = read_status(a.tracker)
    if st["source"] != "live":
        raise RuntimeError(f"tracker source not live: {st}")
    w, h = a.width, a.height
    selector = TargetSelector()

    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    arm = Arm(a.port, a.execute, a.calibration)
    arm.connect()
    if a.execute:
        arm.set_servo_smoothing(a.servo_accel, a.servo_p)
    print(f"start pose: { {j: round(v, 1) for j, v in arm.start.items()} }  execute={a.execute}", flush=True)
    sign_pan, sign_tilt = a.sign_pan, a.sign_tilt
    try:
        if a.probe:
            set_state(mode="probing")
            print("waiting for a target to probe joint signs...", flush=True)
            while not stopped and selector.pick(read_tracks(a.tracker, a.max_track_age), a.min_conf) is None:
                time.sleep(0.2)
            if not stopped:
                sign_pan = probe_sign(arm, PAN, a.tracker, selector, a, w, h)
                sign_tilt = probe_sign(arm, TILT, a.tracker, selector, a, w, h)
                print(f"learned signs: --sign-pan {sign_pan} --sign-tilt {sign_tilt}", flush=True)
        if sign_pan is None or sign_tilt is None:
            raise RuntimeError("joint signs unknown: pass --sign-pan/--sign-tilt or run with --probe")
        set_state(sign_pan=sign_pan, sign_tilt=sign_tilt)

        last_print = 0.0
        dt = 1 / a.hz
        vel = {PAN: 0.0, TILT: 0.0, LIFT: 0.0, ELBOW: 0.0}      # commanded deg/s (goto mode), ramped by --accel
        hist = deque(maxlen=int(a.hz * 2))                        # (time, measured pos) for latency compensation
        last_frame_ts = 0.0
        fgoal = None                                              # filtered absolute goal (pan, tilt)
        bvel = (0.0, 0.0)                                         # target bearing velocity, deg/s (feedforward)
        ferr = None                      # EMA-filtered (ex, ey, track id)

        def soft(e):
            """Deadband that fades gain in smoothly instead of switching it on/off."""
            m = max(0.0, abs(e) - a.deadband) / (1.0 - a.deadband)
            return m if e > 0 else -m

        def ramp(v, v_target):
            step = a.accel * dt
            return float(np.clip(v_target, v - step, v + step))

        while not stopped:
            tick = time.perf_counter()
            try:
                tracks, frame_ts = read_tracks_ts(a.tracker, a.max_track_age)
            except Exception as e:  # noqa: BLE001
                tracks, frame_ts = [], 0.0
                print(f"tracker read failed: {e}", flush=True)
            if a.execute:
                arm.read()
            hist.append((time.time(), dict(arm.pos)))
            goto = None
            if a.aim_url:
                try:
                    with urlopen(a.aim_url, timeout=0.2) as r:
                        aim = json.loads(r.read())
                except Exception:  # noqa: BLE001 - commander down: hold
                    aim = {}
                aim_id = aim.get("id")
                if any(aim.get(k) is not None for k in GOTO_KEYS):
                    goto = aim
                t = next((x for x in tracks if x["id"] == aim_id), None) if aim_id is not None else None
                selector.tid = aim_id
            else:
                t = selector.pick(tracks, a.min_conf, a.keep_conf)
            posr = {"pan": round(arm.pos[PAN], 1), "tilt": round(arm.pos[TILT], 1), "lift": round(arm.pos[LIFT], 1),
                    "elbow": round(arm.pos["elbow_flex"], 1)}
            if t is None and goto is not None:
                # Absolute joint goals (scan / recall): P on measured position, capped speed, ramped.
                ferr = None
                speed = float(goto.get("speed") or a.max_vel)
                want = {}
                for key, j in GOTO_KEYS.items():
                    g = goto.get(key)
                    want[j] = 0.0 if g is None else float(np.clip(a.kp_goto * (arm.clamp(j, float(g)) - arm.pos[j]),
                                                                  -speed, speed))
                for j in GOTO_KEYS.values():
                    vel[j] = ramp(vel[j], want[j])
                arm.command({j: vel[j] * dt for j in GOTO_KEYS.values()})
                set_state(mode="goto", target_id=None, err=None, n_tracks=len(tracks), pos=posr,
                          pan=round(arm.goal[PAN], 2), tilt=round(arm.goal[TILT], 2))
            elif t is None:
                ferr, fgoal = None, None
                if any(abs(v) > 0.01 for v in vel.values()):     # finish decelerating after a goto
                    for j in (PAN, TILT, LIFT, ELBOW):
                        vel[j] = ramp(vel[j], 0.0)
                    arm.command({j: vel[j] * dt for j in (PAN, TILT, LIFT, ELBOW)})
                set_state(mode="holding", target_id=selector.tid, err=None, n_tracks=len(tracks), pos=posr,
                          pan=round(arm.goal[PAN], 2), tilt=round(arm.goal[TILT], 2))
            elif a.control == "absolute":
                # One absolute goal per camera frame: the joint angles that put the person in the
                # center, computed from where the arm WAS when that frame was captured.
                for j in vel:
                    vel[j] = 0.0
                if frame_ts != last_frame_ts:
                    last_frame_ts = frame_ts
                    ex, ey = norm_error(t, w, h)
                    p_at = pos_at(hist, frame_ts - a.cam_latency) or arm.pos
                    # Near the center apply only part of the correction: with ~100 ms of pipeline
                    # latency a full correction overshoots and hunts by +-0.5 deg; a fractional one
                    # converges monotonically (asymptotically) instead.
                    fx_ = a.soft_gain + (1 - a.soft_gain) * min(1.0, abs(ex) / a.soft_zone)
                    fy_ = a.soft_gain + (1 - a.soft_gain) * min(1.0, abs(ey) / a.soft_zone)
                    g_pan = p_at[PAN] + sign_pan * a.gain_pan * ex * fx_
                    g_tilt = p_at[TILT] + sign_tilt * a.gain_tilt * ey * fy_
                    if fgoal is None or fgoal[2] != t["id"]:
                        fgoal, bvel = (g_pan, g_tilt, t["id"], frame_ts), (0.0, 0.0)
                    else:
                        k = a.goal_ema
                        dtf = max(1e-3, frame_ts - fgoal[3])
                        # bearing velocity of the person (filtered) -> lead the goal to cancel latency
                        vp, vt = (g_pan - fgoal[0]) / dtf, (g_tilt - fgoal[1]) / dtf
                        bvel = (0.25 * vp + 0.75 * bvel[0], 0.25 * vt + 0.75 * bvel[1])
                        fgoal = (k * g_pan + (1 - k) * fgoal[0], k * g_tilt + (1 - k) * fgoal[1], t["id"], frame_ts)
                    # only lead for real motion (dead zone) so noise never feeds back into the goal
                    lp = bvel[0] if abs(bvel[0]) > a.lead_deadzone else 0.0
                    lt = bvel[1] if abs(bvel[1]) > a.lead_deadzone else 0.0
                    lead_p = float(np.clip(lp * a.lead, -a.lead_max, a.lead_max))
                    lead_t = float(np.clip(lt * a.lead, -a.lead_max, a.lead_max))
                    goals = {PAN: fgoal[0] + lead_p, TILT: fgoal[1] + lead_t}
                    # wrist out of range in the needed direction: hand the remainder to shoulder_lift
                    if a.sign_lift is not None:
                        lim = arm.half_range[TILT] - 1.0
                        excess = fgoal[1] - float(np.clip(fgoal[1], -lim, lim))
                        if abs(excess) > 0.5:
                            g_lift = p_at[LIFT] + a.sign_lift * sign_tilt * excess
                            lo, hi = arm.start[LIFT] - a.lift_range, arm.start[LIFT] + a.lift_range
                            goals[LIFT] = float(np.clip(g_lift, lo, hi))
                    arm.set_goal(goals, a.min_move)
                    set_state(mode="following", target_id=t["id"], err=[round(ex, 3), round(ey, 3)], pos=posr,
                              n_tracks=len(tracks), pan=round(arm.goal[PAN], 2), tilt=round(arm.goal[TILT], 2))
                    if time.time() - last_print > 1.0:
                        last_print = time.time()
                        print(f"id={t['id']} err=({ex:+.2f},{ey:+.2f}) goal pan={arm.goal[PAN]:.1f} "
                              f"tilt={arm.goal[TILT]:.1f} lift={arm.goal[LIFT]:.1f} pos pan={arm.pos[PAN]:.1f}", flush=True)
            else:
                ex, ey = norm_error(t, w, h)
                if ferr is None or ferr[2] != t["id"]:
                    ferr = (ex, ey, t["id"])                  # new target: no filter memory
                else:
                    ferr = (a.ema * ex + (1 - a.ema) * ferr[0], a.ema * ey + (1 - a.ema) * ferr[1], t["id"])
                fx, fy = ferr[0], ferr[1]
                # sign=-1 means "+joint moves the target toward +error", so sign*kp*err reduces it.
                v_pan = float(np.clip(sign_pan * a.kp_pan * soft(fx), -a.max_vel, a.max_vel))
                v_tilt = float(np.clip(sign_tilt * a.kp_tilt * soft(fy), -a.max_vel, a.max_vel))
                # Wrist saturated in the needed direction? Hand the vertical motion to shoulder_lift.
                v_lift = 0.0
                if a.sign_lift is not None and v_tilt != 0.0:
                    lim = arm.half_range[TILT] - 1.5
                    at_limit = (v_tilt > 0 and arm.goal[TILT] >= lim) or (v_tilt < 0 and arm.goal[TILT] <= -lim)
                    if at_limit:
                        v_lift = a.sign_lift * float(np.clip(a.kp_tilt * soft(fy), -a.max_vel, a.max_vel))
                        lift_off = arm.goal[LIFT] - arm.start[LIFT]
                        if (v_lift > 0 and lift_off >= a.lift_range) or (v_lift < 0 and lift_off <= -a.lift_range):
                            v_lift = 0.0
                        v_tilt = 0.0
                vel[PAN], vel[TILT], vel[LIFT] = ramp(vel[PAN], v_pan), ramp(vel[TILT], v_tilt), ramp(vel[LIFT], v_lift)
                vel[ELBOW] = ramp(vel[ELBOW], 0.0)
                dpan, dtilt = vel[PAN] * dt, vel[TILT] * dt
                arm.command({PAN: dpan, TILT: dtilt, LIFT: vel[LIFT] * dt, ELBOW: vel[ELBOW] * dt})
                set_state(mode="following", target_id=t["id"], err=[round(ex, 3), round(ey, 3)], pos=posr,
                          n_tracks=len(tracks), pan=round(arm.goal[PAN], 2), tilt=round(arm.goal[TILT], 2))
                if time.time() - last_print > 1.0:
                    last_print = time.time()
                    print(f"id={t['id']} err=({ex:+.2f},{ey:+.2f}) v=({vel[PAN]:+.1f},{vel[TILT]:+.1f},{vel[LIFT]:+.1f}) "
                          f"goal pan={arm.goal[PAN]:.1f} tilt={arm.goal[TILT]:.1f} lift={arm.goal[LIFT]:.1f}", flush=True)
            time.sleep(max(0, 1 / a.hz - (time.perf_counter() - tick)))
    finally:
        set_state(mode="stopped")
        arm.close()
        print("torque off, disconnected", flush=True)


if __name__ == "__main__":
    main()
