#!/usr/bin/env python3
"""Deck gateway -- the robot-side half of the deck panel.

    ros2 run wojtek_deck deck_gateway        # then open http://<robot>:8090

One HTTP port (parameter `port`, default 8090) carries three things:

  GET /              the panel (web/index.html and the files next to it)
  GET /ws            the command websocket: JSON both ways, see below
  GET /stream.mjpg   a camera as MJPEG (multipart), for the page's <img> and
                     for any detector on the handheld that wants the same
                     frames (OpenCV opens the URL directly). ?cam=front is
                     the body's colour camera and the default; ?cam=tower is
                     the tower camera the targeting gimbal aims. Each camera
                     is subscribed only while somebody watches it.
  GET /det/          the detector's assets: the YOLOX network and the
                     onnxruntime-web runtime the page runs it with. Big
                     downloaded binaries, so they live in a store outside
                     the package (fetch_assets.sh), not in web/.

The page reads its charts from foxglove_bridge, not from here: this process
only carries what has to run on the robot, which is the dead-man. The
handheld sits on the far side of a wifi link; a dead-man on the handheld
cannot zero anything once that link is gone, and policy_node latches the
last /cmd_vel it saw. So the gate lives here (drive.py) and publishes
/cmd_vel itself: sticks stream in as normalized frames, and when they stop
the gate zeroes the motion for two seconds, then goes silent.

Websocket protocol (text frames, JSON):
  server -> page
    {"t":"hello", cmd_low, cmd_high, height_range, height_default,
                  bridge_port, policy}
    {"t":"avail", "svc": {key: bool}}          which services answer
    {"t":"svc", key, value, success, message}  a service call's verdict
    {"t":"status", drive: idle|live|deadman|follow, height, cam_hz,
                   tower_hz, follow, clients}
  page -> server
    {"t":"cmd", vx, vy, yaw, [height]}         normalized sticks, >= 10 Hz
    {"t":"stop"}                               explicit stop
    {"t":"height", "delta": +-0.005}           step the held stance height
    {"t":"call", key, [value]}                 arm/enable (bool) and the
                                               Trigger services below
    {"t":"call", "key":"restart_stack"}        restart the robot's control
                                               stack (the systemd unit in
                                               the stack_unit parameter);
                                               refused unless the robot is
                                               lying, because the stack
                                               assumes the folded pose when
                                               it starts
    {"t":"track", cx, cy, w, h, fw, fh, label, age, cam}
    {"t":"unlock"}                             the page's lock-in (web/lock.js),
                                               10 Hz while a target is held,
                                               `cam` the camera it was tapped
                                               on

The lock-in is the follow chain's first link. A `track` is republished as
std_msgs/String on /wojtek/track/target -- the page's own JSON plus a
`stamp` of the robot-clock moment it arrived -- and it arms the follow
source in the drive gate. An `unlock` publishes {"unlock": true, "stamp"}
once and disarms it. A plain String carries the JSON because the robot's
package list is fixed and offline, with no vision_msgs in it, and
/wojtek/nav_command already travels the same way.

The follow node that reads those tracks answers on /wojtek/follow/cmd_vel,
which this process subscribes to and feeds to the gate as a second drive
source. The pad always wins; drive.py holds that rule. Nothing here talks
to the gimbal: the follow node owns /targeting/enable_tracking.

Threading is the web_console pattern: rclpy spins in a background thread;
the ROS side hands data to the asyncio side with call_soon_threadsafe and
the asyncio side calls into the node (publishers, async service calls),
which rclpy allows from any thread.
"""
import asyncio
import io
import os
import shutil
import signal
import threading
import time
import json
from pathlib import Path

import rclpy
from aiohttp import web
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from wojtek_policy.policy_source import load_meta
from wojtek_deck.drive import DriveGate
from wojtek_deck.stream import next_frame

# JPEG encoder, cheapest first. OpenCV encodes through libjpeg-turbo and
# releases the GIL while it works, so the asyncio side (the drive tick that
# publishes /cmd_vel) keeps running underneath. Pillow holds the GIL for
# most of the encode: on the RPi that stalled the tick to 6 Hz and, with
# the camera node next to it, starved the control stack until the MD80
# drives dropped to idle (2026-09-12). Pillow stays as the fallback.
try:
    import cv2
except ImportError:
    cv2 = None
try:
    from PIL import Image as PILImage
except ImportError:  # soft dep: no encoder at all = empty camera stream
    PILImage = None

# Fallbacks when no policy reference is set (or it fails to load) -- same
# values and role as in gamepad_teleop / web_console.
DEFAULT_CMD_LOW = (-0.6, -0.4, -0.7)
DEFAULT_CMD_HIGH = (0.6, 0.4, 0.7)
DEFAULT_HEIGHT_RANGE = (0.09, 0.17)
DEFAULT_HEIGHT = 0.125

# Same topic + encoding wojtek_pc/camera_spec.py pins for the sim camera and
# the real perception stack publishes. Repeated here (not imported) because
# wojtek_pc is PC-only and never reaches the robot.
DEFAULT_COLOR_TOPIC = "/camera/camera/color/image_raw"

# The tower camera: a plain UVC webcam on the gimbal, published raw and
# best-effort by the targeting experiment's usb_camera_node at 640x480 in
# bgr8. The name is repeated here rather than imported, for the same reason
# the colour topic is: nothing in ros/ may depend on an experiment.
DEFAULT_TOWER_TOPIC = "/targeting_camera/targeting_camera/image_raw"

# Raw encodings either camera may arrive in. Three bytes a pixel, and the
# only difference is which way round they are.
RAW_ENCODINGS = ("rgb8", "bgr8")

CAMERAS = ("front", "tower")   # ?cam= on /stream.mjpg; front is the default

# What a page `track` carries to the follow node. A whitelist, because the
# gateway copies the page's own JSON onto a robot topic: cx, cy, w and h are
# the box in the pixels of an fw by fh frame, `age` is how long ago a
# detection last matched it, and `cam` is the camera it was tapped on.
TRACK_FIELDS = ("cx", "cy", "w", "h", "fw", "fh", "label", "age", "cam")

DRIVE_TICK_HZ = 20.0     # /cmd_vel publish rate (same as the other teleops)
STATUS_HZ = 2.0          # status frames to the page
CMD_TIMEOUT_S = 0.5      # dead-man: sticks older than this = link gone
SILENCE_AFTER_S = 2.0    # zeroing burst length before going silent

# "Lying" for the stack restart: every joint within this of the folded pose,
# which is where the encoders read zero after a boot in that pose.
LYING_MAX_RAD = 0.35

SETBOOL_SERVICES = ("arm", "enable")
TRIGGER_SERVICES = ("zero", "stand_up", "lie_down", "reset",
                    "trick_paw_wave", "trick_bow", "trick_sit", "trick_shake")


def assets_store():
    """Where the detector's downloaded files live on this machine.

    The same shape as wojtek_policy's policy_store, and for the same reason:
    they are fetched binaries that must not be committed, so they sit in a
    gitignored directory next to the workspace's `src/` -- ros/deck_assets in
    a checkout, ~/wojtek_ws/deck_assets on the robot. WOJTEK_DECK_ASSETS wins
    when set, which is how a container names it, because a build there is not
    always --symlink-install and the walk up from this file finds no src/.

    Returns None when it cannot tell; the caller then serves the panel
    without a detector rather than refusing to start.
    """
    env = os.environ.get("WOJTEK_DECK_ASSETS", "").strip()
    if env:
        return Path(env).expanduser()
    # A symlink install resolves this file into src/; a copying install
    # (the robot's) into install/. Either way the store sits next to them.
    for parent in Path(__file__).resolve().parents:
        if parent.name in ("src", "install"):
            return parent.parent / "deck_assets"
    return None


def encode_jpeg(msg, quality):
    """One raw Image message (rgb8 or bgr8) -> JPEG bytes."""
    if cv2 is not None:
        import numpy as np
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        bgr = raw if msg.encoding == "bgr8" else cv2.cvtColor(
            raw, cv2.COLOR_RGB2BGR)
        ok, out = cv2.imencode(".jpg", bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return out.tobytes()
    # Pillow's raw decoder knows "RGB" and "BGR", not the ROS encoding names.
    raw_mode = "BGR" if msg.encoding == "bgr8" else "RGB"
    img = PILImage.frombuffer("RGB", (msg.width, msg.height), msg.data,
                              "raw", raw_mode, 0, 1)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class CameraTap:
    """One camera's path from ROS to the MJPEG stream.

    The front camera and the tower camera are the same path twice, so each
    is one of these: its own subscription, its own rate limit, its own
    counters. One camera going quiet then says nothing about the other, and
    the page can watch either.

    The subscription exists only while somebody is on that camera's stream.
    Receiving a raw image costs a third of a core on the RPi whether or not
    a frame gets encoded, the gateway is resident in the robot service, and
    there are two cameras on one USB2 bus: an unwatched picture must cost
    nothing.
    """

    def __init__(self, node, name, topic, compressed, quality, period):
        self.node = node
        self.name = name              # "front" or "tower"
        self.topic = topic
        self.compressed = bool(compressed)   # take the camera node's own JPEG
        self.quality = int(quality)
        self.period = float(period)   # seconds between encoded frames
        self.on_frame = None          # set by the node: (name, bytes) -> None
        self.want_frames = None       # set by the node: (name) -> bool
        self.frames_seen = 0          # camera messages received
        self.frames_encoded = 0       # ... of which reached a viewer
        self._sub = None
        self._last_encode = 0.0       # monotonic time of the last one out
        self._stamps = []             # wall times of the last frames out
        self._last_cb = None

    def log(self):
        return self.node.get_logger()

    def set_wanted(self, wanted):
        if wanted and self._sub is None:
            # The cameras publish best-effort; a default-QoS subscription
            # would match nothing, so mirror the sensor-data profile.
            if self.compressed:
                self._sub = self.node.create_subscription(
                    CompressedImage, self.topic + "/compressed",
                    self._on_compressed, qos_profile_sensor_data)
            else:
                self._sub = self.node.create_subscription(
                    Image, self.topic, self._on_raw, qos_profile_sensor_data)
            self.log().info(
                f"{self.name} camera: viewer arrived, subscribing to "
                f"{self.topic}{'/compressed' if self.compressed else ''}")
        elif not wanted and self._sub is not None:
            self.node.destroy_subscription(self._sub)
            self._sub = None
            self._stamps = []
            self.log().info(
                f"{self.name} camera: last viewer gone, unsubscribed")

    def hz(self):
        now = time.monotonic()
        recent = [t for t in self._stamps if now - t < 2.0]
        return len(recent) / 2.0

    def _due(self):
        """Is this frame wanted, and is it time for one? Rate limit included."""
        if self.on_frame is None or not self.want_frames(self.name):
            return False
        now = time.monotonic()
        if now - self._last_encode < self.period:
            return False   # over the stream rate: dropped before it costs
        return True

    def _sent(self, now):
        self._last_encode = now
        self._stamps = [t for t in self._stamps if now - t < 2.0]
        self._stamps.append(now)
        self.frames_encoded += 1

    def _on_compressed(self, msg):
        """A JPEG from the camera node: pass it through, nothing to encode."""
        self.frames_seen += 1
        if not self._due():
            return
        if "jpeg" not in msg.format.lower():
            self.log().warning(
                f"compressed {self.name} camera format {msg.format!r} is not "
                "JPEG; set the camera's compressed format to jpeg", once=True)
            return
        self._sent(time.monotonic())
        if self.frames_encoded == 1:
            self.log().info(
                f"{self.name} camera: first frame passed through "
                f"({len(msg.data) // 1024} KB JPEG from the camera node)")
        self.on_frame(self.name, bytes(msg.data))

    def _on_raw(self, msg):
        self.frames_seen += 1
        if not self._due():
            return
        if msg.encoding not in RAW_ENCODINGS:
            self.log().warning(
                f"unsupported {self.name} camera encoding {msg.encoding!r} "
                f"(want one of {', '.join(RAW_ENCODINGS)})", once=True)
            return
        try:
            jpeg = encode_jpeg(msg, self.quality)
        except Exception as e:  # noqa: BLE001 -- a bad frame must not kill
            # the spin thread (see web_console for the same rule)
            self.log().warning(
                f"dropping {self.name} camera frame: {e}", once=True)
            return
        now = time.monotonic()
        if self._last_cb is not None and now - self._last_cb > 1.0:
            self.log().warning(
                f"{self.name} camera callback starved: {now - self._last_cb:.1f} "
                f"s since the previous frame (seen {self.frames_seen})")
        self._last_cb = now
        self._sent(now)
        if self.frames_encoded == 1:
            self.log().info(
                f"{self.name} camera: first frame encoded ({msg.width}x"
                f"{msg.height}, {len(jpeg) // 1024} KB JPEG)")
        t0 = time.monotonic()
        self.on_frame(self.name, jpeg)
        dt = time.monotonic() - t0
        if dt > 0.05:
            self.log().warning(
                f"handing a {self.name} frame to the server took {dt*1000:.0f} ms")


class GatewayNode(Node):
    """ROS half: service clients, the /cmd_vel publisher, the camera taps."""

    def __init__(self, emit, want_frames):
        super().__init__("deck_gateway")
        self.emit = emit                # (dict) -> None, safe from ROS thread
        self.want_frames = want_frames  # (cam) -> bool: anyone on that stream?
        self.on_follow = None           # set by the server: (vx, vy, yaw) -> None

        self.declare_parameter("policy", "")
        self.declare_parameter("port", 8090)
        # Told to the page so it knows where foxglove_bridge listens.
        self.declare_parameter("bridge_port", 8765)
        self.declare_parameter("color_topic", DEFAULT_COLOR_TOPIC)
        # The tower camera, /stream.mjpg?cam=tower. Raw and best-effort: the
        # targeting camera node is Python and publishes no compressed topic.
        self.declare_parameter("tower_topic", DEFAULT_TOWER_TOPIC)
        self.declare_parameter("jpeg_quality", 80)
        # Most frames a second that get encoded for the stream. The camera
        # may run faster; the rest are dropped before they cost anything.
        # Encoding is the gateway's whole CPU bill, and on the RPi that
        # bill is paid by the same four cores as the control loop.
        self.declare_parameter("stream_hz", 10.0)
        # Take the camera node's own JPEG (image_transport's compressed
        # plugin, <color_topic>/compressed) instead of the raw image. The
        # camera node encodes in C++ and only while somebody subscribes;
        # the gateway then receives ~40 KB a frame instead of 0.9 MB and
        # encodes nothing. false = the raw image and the encoder below,
        # for a robot without the plugin.
        self.declare_parameter("compressed", True)
        # Where the detector's files are. Empty means "work it out", which
        # is right everywhere except a container that named it differently.
        self.declare_parameter("assets_dir", "")

        self._cli = {k: self.create_client(SetBool, f"wojtek/{k}")
                     for k in SETBOOL_SERVICES}
        self._cli.update({k: self.create_client(Trigger, f"wojtek/{k}")
                          for k in TRIGGER_SERVICES})
        self._pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        # The lock-in, on its way to the follow node. JSON in a String: see
        # the module docstring for why it is not a typed message.
        self._pub_track = self.create_publisher(String, "wojtek/track/target", 10)
        # The follow node's answer. It reaches the drive gate as a second
        # source and drives only while a lock is on and the pad is quiet.
        self.create_subscription(Twist, "wojtek/follow/cmd_vel",
                                 self._on_follow_cmd, 10)

        # The control stack's systemd unit, for the panel's restart button.
        # Empty disables the button (the simulation has no such unit).
        self.declare_parameter("stack_unit", "wojtek-robot.service")
        # Where the joints are, for the "is it lying" check before a
        # restart: the largest distance from zero, and when it was seen.
        self.joint_max_rad = None
        self._joint_stamp = 0.0
        self.create_subscription(JointState, "joint_states", self._on_joints, 10)

        self.cmd_low = list(DEFAULT_CMD_LOW)
        self.cmd_high = list(DEFAULT_CMD_HIGH)
        self.height_range = list(DEFAULT_HEIGHT_RANGE)
        self.height_default = DEFAULT_HEIGHT
        self.policy_name = ""
        self._load_meta()

        quality = int(self.get_parameter("jpeg_quality").value)
        period = 1.0 / max(0.1, float(self.get_parameter("stream_hz").value))
        self._encoder = None
        if cv2 is not None or PILImage is not None:
            self._encoder = "OpenCV" if cv2 is not None else "Pillow"
            self.get_logger().info(f"camera JPEG encoder: {self._encoder}")
        else:
            self.get_logger().warning(
                "no JPEG encoder (neither OpenCV nor Pillow) -- camera "
                "streams will stay empty (apt install python3-opencv)")
        # One tap per camera, subscribed only while watched.
        self.taps = {
            "front": CameraTap(
                self, "front", str(self.get_parameter("color_topic").value),
                bool(self.get_parameter("compressed").value), quality, period),
            "tower": CameraTap(
                self, "tower", str(self.get_parameter("tower_topic").value),
                False, quality, period),
        }
        for tap in self.taps.values():
            tap.want_frames = self.want_frames

    def _load_meta(self):
        ref = self.get_parameter("policy").value
        if not ref:
            self.get_logger().warning(
                "no policy reference set; driving with default command limits")
            return
        try:
            meta, source = load_meta(ref)
        except Exception as e:  # resolver/network/file -- stay drivable
            self.get_logger().warning(
                f"could not load policy contract {ref!r} ({e}); driving "
                "with default command limits")
            return
        self.cmd_low = [float(v) for v in meta["command_low"][:3]]
        self.cmd_high = [float(v) for v in meta["command_high"][:3]]
        if len(meta["command_low"]) >= 4:
            self.height_range = [
                float(meta["command_low"][3]), float(meta["command_high"][3])
            ]
        if meta.get("command_fill"):
            self.height_default = float(meta["command_fill"][0])
        self.policy_name = str(meta.get("run_name", ""))
        self.get_logger().info(
            f"command box from {meta['run_name']} ({source})")

    # -- camera (asyncio thread) ---------------------------------------------
    def set_camera_wanted(self, cam, wanted):
        """Subscribe to one camera while a viewer is on its stream.

        Called from the stream handler, which runs on the asyncio side.
        rclpy allows creating and destroying a subscription from any thread.
        """
        tap = self.taps.get(cam)
        if tap is None:
            return
        if wanted and self._encoder is None and not tap.compressed:
            return   # nothing here could encode a raw frame, so do not pay
        tap.set_wanted(wanted)

    def tap(self, cam):
        return self.taps[cam]

    # -- the lock-in (asyncio thread) ----------------------------------------
    def clock_s(self):
        """This node's clock in seconds, the stamp every track carries.

        The handheld's clock is its own and can be minutes out. The gimbal
        node judges a target by its age against the robot's clock, so the
        moment that counts is the one this process read on arrival.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_track(self, msg):
        """One page `track` -> std_msgs/String on /wojtek/track/target.

        Only the fields the follow node reads are copied across, so a page
        sending anything else cannot put it on a robot topic.
        """
        out = {k: msg.get(k) for k in TRACK_FIELDS}
        if out["cam"] not in CAMERAS:
            out["cam"] = "front"
        out["stamp"] = self.clock_s()
        self._pub_track.publish(String(data=json.dumps(out)))

    def publish_unlock(self):
        """The lock ended: one message that says so, on the same topic."""
        self._pub_track.publish(String(data=json.dumps(
            {"unlock": True, "stamp": self.clock_s()})))

    def _on_follow_cmd(self, msg):
        """The follow node's answer, on the ROS thread.

        It goes to the server, which hands it to the gate on the asyncio
        thread with the clock the pad frames are stamped with. The gate is
        touched from one thread only, which is what keeps the dead-man's
        arithmetic honest.
        """
        if self.on_follow is not None:
            self.on_follow(msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_joints(self, msg):
        if msg.position:
            self.joint_max_rad = max(abs(p) for p in msg.position)
            self._joint_stamp = time.monotonic()

    def lying(self):
        """(ok, reason): may the control stack be restarted right now?"""
        if self.joint_max_rad is None or time.monotonic() - self._joint_stamp > 1.0:
            return False, "no fresh joint states"
        if self.joint_max_rad > LYING_MAX_RAD:
            return False, (f"robot is not lying: a joint is "
                           f"{self.joint_max_rad:.2f} rad from folded")
        return True, ""

    def stack_unit(self):
        unit = str(self.get_parameter("stack_unit").value).strip()
        return unit if unit and shutil.which("systemctl") else ""

    # -- commands (asyncio thread) ---------------------------------------------
    def availability(self):
        avail = {k: c.service_is_ready() for k, c in self._cli.items()}
        avail["restart_stack"] = bool(self.stack_unit())
        return avail

    def call(self, key, value=None):
        cli = self._cli[key]
        if not cli.service_is_ready():
            self.emit({"t": "svc", "key": key, "value": value,
                       "success": False, "message": "service unavailable"})
            return
        req = (SetBool.Request(data=bool(value)) if key in SETBOOL_SERVICES
               else Trigger.Request())
        fut = cli.call_async(req)

        def done(f, key=key, value=value):
            try:
                resp = f.result()
                self.emit({"t": "svc", "key": key, "value": value,
                           "success": bool(resp.success),
                           "message": resp.message})
            except Exception as e:  # noqa: BLE001 -- surface any RPC failure
                self.emit({"t": "svc", "key": key, "value": value,
                           "success": False, "message": str(e)})
        fut.add_done_callback(done)

    def publish_cmd(self, vx, vy, yaw, height):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(yaw)
        # Standing-height command; policy_node treats 0 as "use the default".
        t.linear.z = float(height)
        self._pub_cmd.publish(t)


@web.middleware
async def cross_origin_isolation(request, handler):
    """Let the page use shared memory.

    A browser only hands a page shared memory once the page promises it is
    not sharing a process with anything it did not ask for. These two
    headers are that promise. The detector runs on one thread today
    (det_worker.js says why), so nothing needs the memory yet; the headers
    stay because they are the one thing a multithreaded runtime will ask
    for, and they cost nothing. Everything the panel loads is same-origin,
    so nothing else has to change: the camera stream, the scripts and the
    bridge websocket are unaffected.

    A response that has already started (the MJPEG stream, the websocket)
    sent its headers inside the handler, so this only decorates the rest --
    which is fine, because the promise has to be on the page itself.
    """
    resp = await handler(request)
    resp.headers["Cross-Origin-Opener-Policy"] = "same-origin"
    resp.headers["Cross-Origin-Embedder-Policy"] = "require-corp"
    # The page's own files change with every deploy, and a browser left to
    # guess from Last-Modified will happily keep yesterday's deck.js for
    # hours. no-cache means "ask first": the browser still keeps a copy and
    # revalidates it with the ETag, which is one small round trip per file.
    # The detector assets are pinned by hash and never change under the
    # same name, so those are left to cache for as long as they like.
    if not request.path.startswith("/det/"):
        resp.headers.setdefault("Cache-Control", "no-cache")
    return resp


class Server:
    """asyncio half: HTTP + websocket + MJPEG, and the drive tick."""

    def __init__(self, node, web_dir, loop, assets_dir=None):
        self.node = node
        self.web_dir = web_dir
        self.assets_dir = assets_dir
        self.loop = loop
        self.clients = set()      # websocket connections
        # One set of viewer queues per camera: an asyncio.Queue per MJPEG
        # viewer, so a watcher of one picture does not subscribe the other.
        self.streams = {name: set() for name in CAMERAS}
        self.gate = DriveGate(node.cmd_low, node.cmd_high, node.height_range,
                              node.height_default, timeout_s=CMD_TIMEOUT_S,
                              silence_after_s=SILENCE_AFTER_S)
        self._last_state = None

    # -- cross-thread entry points -----------------------------------------
    def emit(self, obj):
        self.loop.call_soon_threadsafe(self._broadcast, obj)

    def push_frame(self, cam, jpeg):
        self.loop.call_soon_threadsafe(self._fanout_frame, cam, jpeg)

    def push_follow(self, vx, vy, yaw):
        self.loop.call_soon_threadsafe(self._take_follow, vx, vy, yaw)

    def want_frames(self, cam):
        return bool(self.streams.get(cam))

    def _broadcast(self, obj):
        if not self.clients:
            return
        data = json.dumps(obj)
        for ws in list(self.clients):
            if not ws.closed:
                asyncio.ensure_future(ws.send_str(data))

    def _fanout_frame(self, cam, jpeg):
        for q in self.streams.get(cam, ()):
            # Latest frame wins: a slow viewer drops frames, never lags.
            if q.full():
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(jpeg)

    def _take_follow(self, vx, vy, yaw):
        """A /wojtek/follow/cmd_vel frame, now on the asyncio thread.

        Stamped with the same clock as the pad frames, which is the only way
        the gate can tell which source spoke last.
        """
        self.gate.follow(self.loop.time(), vx, vy, yaw)

    # -- HTTP handlers -----------------------------------------------------
    async def index(self, request):
        return web.FileResponse(os.path.join(self.web_dir, "index.html"),
                                headers={"Cache-Control": "no-store"})

    async def stream(self, request):
        """One camera as MJPEG. ?cam=front (the default) or ?cam=tower."""
        cam = request.query.get("cam", "front")
        if cam not in CAMERAS:
            raise web.HTTPNotFound(
                text=f"no camera {cam!r} (have {', '.join(CAMERAS)})")
        boundary = "wojtekframe"
        resp = web.StreamResponse(status=200, headers={
            "Content-Type": f"multipart/x-mixed-replace; boundary={boundary}",
            "Cache-Control": "no-store",
        })
        await resp.prepare(request)
        q = asyncio.Queue(maxsize=1)
        viewers = self.streams[cam]
        viewers.add(q)
        self.node.set_camera_wanted(cam, True)
        try:
            # The wait is bounded (wojtek_deck/stream.py): a viewer that left
            # while the camera was silent is found at the next poll, not at
            # a write that never comes.
            while True:
                jpeg = await next_frame(q, request)
                if jpeg is None:
                    break
                await resp.write(
                    f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    + jpeg + b"\r\n")
        except (ConnectionResetError, asyncio.CancelledError,
                ConnectionError):
            pass
        finally:
            viewers.discard(q)
            if not viewers:
                self.node.set_camera_wanted(cam, False)
        return resp

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        self.clients.add(ws)
        await ws.send_str(json.dumps(self.hello()))
        await ws.send_str(json.dumps(
            {"t": "avail", "svc": self.node.availability()}))
        await ws.send_str(json.dumps(self.status()))
        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    self._on_message(json.loads(msg.data))
                except (ValueError, TypeError, KeyError):
                    continue
        finally:
            self.clients.discard(ws)
            if not self.clients:
                # Last page gone: whatever it was commanding stops now. A
                # lock it held ends the same way, and the follow node is
                # told so, because nothing else would ever tell it.
                if self.gate.follow_active:
                    self.node.publish_unlock()
                    self.node.get_logger().info("last page gone, lock dropped")
                self.gate.stop(self.loop.time())
        return ws

    def hello(self):
        return {
            "t": "hello",
            "cmd_low": self.node.cmd_low,
            "cmd_high": self.node.cmd_high,
            "height_range": self.node.height_range,
            "height_default": self.node.height_default,
            "bridge_port": int(self.node.get_parameter("bridge_port").value),
            "policy": self.node.policy_name,
        }

    def status(self):
        taps = self.node.taps
        return {
            "t": "status",
            "drive": self.gate.state,
            # Whether a lock is armed here. The page draws its own lock, so
            # this is the robot's word for it, and the two showing different
            # things is the sign that a track or an unlock went missing.
            "follow": self.gate.follow_active,
            "height": self.gate.height,
            "cam_hz": taps["front"].hz(),
            "tower_hz": taps["tower"].hz(),
            # Summed over the cameras: a diagnostic for a curl, not for the
            # page, which reads the rate of the picture it is showing.
            "frames_seen": sum(t.frames_seen for t in taps.values()),
            "frames_encoded": sum(t.frames_encoded for t in taps.values()),
            "clients": len(self.clients),
            "viewers": sum(len(v) for v in self.streams.values()),
        }

    def _on_message(self, msg):
        t = msg.get("t")
        now = self.loop.time()
        if t == "cmd":
            self.gate.command(now, msg.get("vx", 0), msg.get("vy", 0),
                              msg.get("yaw", 0), msg.get("height"))
        elif t == "stop":
            self.gate.stop(now)
        elif t == "track":
            # Ten a second while a target is held. Each one goes out on
            # /wojtek/track/target and arms the follow source; the follow
            # node's own /wojtek/follow/cmd_vel is what actually drives.
            self.node.publish_track(msg)
            # Only a tower lock can drive: a front-camera pixel carries no
            # tower bearing, and the follow node refuses such tracks. The
            # front lock still shows on the page and still goes out on the
            # topic, but the drive stays with the pad.
            if msg.get("cam") != "tower":
                self.node.get_logger().info(
                    f"lock on {msg.get('label')!r} on the front camera: "
                    "shown, not followed", throttle_duration_sec=5.0)
            elif not self.gate.follow_active:
                self.node.get_logger().info(
                    f"lock on {msg.get('label')!r} ({msg.get('cam')} camera)")
                self.gate.start_follow()
        elif t == "unlock":
            if self.gate.follow_active:
                self.node.get_logger().info("lock released")
            self.node.publish_unlock()
            self.gate.end_follow(now)
        elif t == "height":
            h = self.gate.step_height(msg.get("delta", 0.0))
            self._broadcast({"t": "status", **{k: v for k, v in
                             self.status().items() if k != "t"},
                             "height": h})
        elif t == "call":
            key = msg.get("key")
            if key in SETBOOL_SERVICES or key in TRIGGER_SERVICES:
                self.node.call(key, msg.get("value"))
            elif key == "restart_stack":
                asyncio.ensure_future(self.restart_stack())

    async def restart_stack(self):
        """Restart the control stack's systemd unit, if the robot is lying.

        The stack assumes the folded pose when it starts (real_io zeroes
        there), so a restart mid-stand would leave every joint offset
        wrong. What makes this necessary at all: after a motor power cycle
        under a running controller the drives come back idle, keep
        answering on CAN, and nothing re-enables them. The gateway is a
        separate process, so the panel stays up and shows the stack coming
        back.
        """
        def verdict(success, message):
            self._broadcast({"t": "svc", "key": "restart_stack", "value": None,
                             "success": success, "message": message})

        unit = self.node.stack_unit()
        if not unit:
            verdict(False, "no control stack unit configured")
            return
        ok, why = self.node.lying()
        if not ok:
            verdict(False, why)
            return
        self.gate.stop(self.loop.time())
        self.node.get_logger().warning(f"restarting {unit} on the panel's request")
        try:
            proc = await asyncio.create_subprocess_exec(
                "sudo", "-n", "systemctl", "restart", unit,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE)
            _, err = await asyncio.wait_for(proc.communicate(), timeout=90)
        except Exception as e:  # noqa: BLE001 -- report, never crash the loop
            verdict(False, f"restart failed: {e}")
            return
        if proc.returncode == 0:
            verdict(True, f"{unit} restarted; the stack comes up in ~30 s")
        else:
            verdict(False, f"systemctl exited {proc.returncode}: "
                           f"{err.decode(errors='replace').strip()[:120]}")

    # -- periodic tasks ----------------------------------------------------
    async def drive_tick(self):
        while True:
            out = self.gate.tick(self.loop.time())
            if out is not None:
                self.node.publish_cmd(*out)
            if self.gate.state != self._last_state:
                if self.gate.state == "deadman":
                    self.node.get_logger().warning(
                        "pad frames stopped -- zeroing /cmd_vel")
                else:
                    self.node.get_logger().info(f"drive {self.gate.state}")
                self._last_state = self.gate.state
                self._broadcast(self.status())
            await asyncio.sleep(1.0 / DRIVE_TICK_HZ)

    async def status_tick(self):
        last_avail = None
        while True:
            if self.clients:
                avail = self.node.availability()
                if avail != last_avail:
                    self._broadcast({"t": "avail", "svc": avail})
                    last_avail = avail
                self._broadcast(self.status())
            await asyncio.sleep(1.0 / STATUS_HZ)

    def app(self):
        app = web.Application(middlewares=[cross_origin_isolation])
        app.router.add_get("/", self.index)
        app.router.add_get("/ws", self.websocket)
        app.router.add_get("/stream.mjpg", self.stream)
        # The detector's assets, before the catch-all below: aiohttp tries
        # routes in the order they were added, and "/" matches everything.
        if self.assets_dir is not None:
            app.router.add_static("/det/", str(self.assets_dir),
                                  show_index=False)
        # css/js next to the page; no directory listing
        app.router.add_static("/", self.web_dir, show_index=False)
        return app


def main():
    # No rclpy signal handlers: they would shut the ROS context down under
    # the spin thread and leave the web server running with no ROS behind
    # it -- a page that loads and never gets a frame, on a port the next
    # launch cannot take. The asyncio loop below owns the signals instead
    # and takes everything down in one go.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    server = None

    def emit(obj):
        if server is not None:
            server.emit(obj)

    def want_frames(cam):
        return server is not None and server.want_frames(cam)

    node = GatewayNode(emit, want_frames)
    port = int(node.get_parameter("port").value)
    web_dir = os.path.join(get_package_share_directory("wojtek_deck"), "web")

    # The detector's files are optional. Without them the panel is the panel
    # it always was, minus the boxes, so say so once and carry on.
    param = str(node.get_parameter("assets_dir").value).strip()
    assets_dir = Path(param).expanduser() if param else assets_store()
    if assets_dir is None or not assets_dir.is_dir():
        where = (f"no detector assets in {assets_dir}" if assets_dir
                 else "cannot tell where the detector assets are (set "
                      "WOJTEK_DECK_ASSETS or the assets_dir parameter)")
        node.get_logger().warning(
            f"{where} -- the panel will run without detection. Fetch them "
            "with ros/src/wojtek_deck/fetch_assets.sh")
        assets_dir = None
    else:
        node.get_logger().info(f"detector assets from {assets_dir}")

    server = Server(node, web_dir, loop, assets_dir)
    for tap in node.taps.values():
        tap.on_frame = server.push_frame
    node.on_follow = server.push_follow

    stopping = asyncio.Event()

    def stop(why):
        if not stopping.is_set():
            node.get_logger().info(f"deck gateway stopping ({why})")
            stopping.set()

    def spin():
        # If ROS goes away underneath us (an external shutdown), the server
        # must not outlive it: nothing behind it would ever send a frame.
        try:
            rclpy.spin(node)
        except Exception:  # noqa: BLE001 -- shutdown races raise here
            pass
        finally:
            loop.call_soon_threadsafe(stop, "ROS spin ended")

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    # Ctrl-C at a terminal, SIGTERM from a launch file or systemd: one path
    # for both, and it ends in the finally below.
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop, signal.Signals(sig).name)

    async def run():
        runner = web.AppRunner(server.app(), access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", port)
        await site.start()
        node.get_logger().info(f"deck panel on http://0.0.0.0:{port}")
        ticks = asyncio.gather(server.drive_tick(), server.status_tick())
        try:
            await stopping.wait()
        finally:
            ticks.cancel()
            try:
                await ticks
            except asyncio.CancelledError:
                pass
            await runner.cleanup()

    try:
        loop.run_until_complete(run())
    finally:
        # Shut ROS down first so the spin thread wakes up and returns, then
        # wait for it. Exiting while it still sits inside the executor ends
        # in a C++ abort rather than a clean exit.
        rclpy.try_shutdown()
        spinner.join(timeout=3.0)


if __name__ == "__main__":
    main()
