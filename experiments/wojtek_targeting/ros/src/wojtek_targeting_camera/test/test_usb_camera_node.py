"""Model-free tests for the USB camera node.

There is no camera and no ROS in the environment this package is developed in,
so cv2, rclpy and sensor_msgs are stubbed and the node's own logic is exercised
against them. This covers message construction, the intrinsics defaulting and
the capture settings that matter for latency; it proves nothing about how a
real webcam behaves.
"""

import importlib
import sys
import types

import numpy as np
import pytest


# --- stubs, installed before the node module is imported ------------------

class FakeVideoCapture:
    def __init__(self, device, backend=None):
        self.device = device
        self.backend = backend
        self.props = {}
        self.opened = True
        self.frames = []
        self.released = False

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        # Report back whatever was requested, so the node's "did the camera
        # accept it" check passes unless a test says otherwise.
        return self.props.get(prop, 0)

    def read(self):
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self):
        self.released = True


def _install_stubs():
    cv2 = types.ModuleType("cv2")
    cv2.CAP_V4L2 = 200
    cv2.CAP_PROP_FRAME_WIDTH = 3
    cv2.CAP_PROP_FRAME_HEIGHT = 4
    cv2.CAP_PROP_FPS = 5
    cv2.CAP_PROP_FOURCC = 6
    cv2.CAP_PROP_BUFFERSIZE = 38
    cv2.VideoWriter_fourcc = lambda *a: sum(ord(c) for c in a)
    cv2.VideoCapture = FakeVideoCapture
    sys.modules["cv2"] = cv2

    class Logger:
        def __init__(self):
            self.messages = []

        def info(self, msg, **kw):
            self.messages.append(("info", msg))

        def warn(self, msg, **kw):
            self.messages.append(("warn", msg))

        def error(self, msg, **kw):
            self.messages.append(("error", msg))

    class Param:
        def __init__(self, value):
            self.value = value

    class Node:
        # Tests set this before constructing, to stand in for a params file.
        overrides = {}

        def __init__(self, name):
            self.name = name
            self._logger = Logger()
            self.timers = []
            self.published = []

        def declare_parameter(self, name, default):
            return Param(self.overrides.get(name, default))

        def create_publisher(self, msg_type, topic, qos):
            published = self.published

            class Pub:
                def publish(self, msg):
                    published.append((topic, msg))

            return Pub()

        def create_timer(self, period, callback):
            self.timers.append((period, callback))

        def get_logger(self):
            return self._logger

        def get_clock(self):
            class Clock:
                def now(self):
                    class Time:
                        def to_msg(self):
                            return "stamp"

                    return Time()

            return Clock()

        def destroy_node(self):
            pass

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = Node
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    rclpy.node = rclpy_node
    rclpy.qos = rclpy_qos
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    class _Msg:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")

    class Image(_Msg):
        def __init__(self):
            super().__init__()
            self.height = 0
            self.width = 0
            self.encoding = ""
            self.is_bigendian = 0
            self.step = 0
            self.data = b""

    class CameraInfo(_Msg):
        def __init__(self):
            super().__init__()
            self.height = 0
            self.width = 0
            self.distortion_model = ""
            self.d = []
            self.k = []
            self.r = []
            self.p = []

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = Image
    sensor_msgs_msg.CameraInfo = CameraInfo
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    return Node


NodeStub = _install_stubs()
usb_camera_node = importlib.import_module("wojtek_targeting_camera.usb_camera_node")


@pytest.fixture
def node_factory():
    def make(**overrides):
        NodeStub.overrides = overrides
        node = usb_camera_node.UsbCameraNode()
        NodeStub.overrides = {}
        return node

    return make


# --- capture setup --------------------------------------------------------

def test_capture_is_configured_for_low_latency(node_factory):
    """One buffer and MJPG, or the gimbal aims at a stale frame."""
    import cv2

    node = node_factory()
    props = node._capture.props

    assert props[cv2.CAP_PROP_BUFFERSIZE] == 1
    assert props[cv2.CAP_PROP_FOURCC] == cv2.VideoWriter_fourcc(*"MJPG")
    assert props[cv2.CAP_PROP_FRAME_WIDTH] == 640
    assert props[cv2.CAP_PROP_FRAME_HEIGHT] == 480
    assert node._capture.backend == cv2.CAP_V4L2


def test_timer_period_follows_framerate(node_factory):
    node = node_factory(framerate=15.0)
    period, _ = node.timers[0]
    assert period == pytest.approx(1.0 / 15.0)


def test_blank_fourcc_leaves_format_alone(node_factory):
    import cv2

    node = node_factory(fourcc="")
    assert cv2.CAP_PROP_FOURCC not in node._capture.props


def test_uncalibrated_camera_warns(node_factory):
    node = node_factory()
    warnings = [m for level, m in node._logger.messages if level == "warn"]
    assert any("fx/fy are unset" in m for m in warnings)


# --- publishing -----------------------------------------------------------

def test_tick_publishes_image_and_info(node_factory):
    node = node_factory()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[0, 0] = (1, 2, 3)
    node._capture.frames = [frame]

    node._tick()

    topics = [t for t, _ in node.published]
    assert topics == ["~/camera_info", "~/image_raw"]

    _, image = node.published[1]
    assert (image.width, image.height) == (640, 480)
    assert image.encoding == "bgr8"
    assert image.step == 640 * 3
    assert len(image.data) == 640 * 480 * 3
    # BGR is passed through untouched -- no conversion in this path.
    assert image.data[:3] == bytes((1, 2, 3))


def test_failed_read_publishes_nothing_and_counts(node_factory):
    node = node_factory()
    node._capture.frames = []

    node._tick()

    assert node.published == []
    assert node._read_failures == 1
    assert any(level == "error" for level, _ in node._logger.messages)


def test_failure_counter_resets_after_a_good_frame(node_factory):
    node = node_factory()
    node._tick()
    assert node._read_failures == 1

    node._capture.frames = [np.zeros((480, 640, 3), dtype=np.uint8)]
    node._tick()
    assert node._read_failures == 0


# --- intrinsics -----------------------------------------------------------

def test_camera_info_defaults_principal_point_to_image_centre(node_factory):
    node = node_factory()
    info = node._camera_info("stamp", 640, 480)

    assert info.k[2] == pytest.approx(320.0)
    assert info.k[5] == pytest.approx(240.0)
    assert info.k[0] == 0.0  # fx stays unset; there is no sane default
    assert info.distortion_model == "plumb_bob"
    assert len(info.d) == 5


def test_camera_info_uses_explicit_intrinsics(node_factory):
    node = node_factory(fx=600.0, fy=601.0, cx=311.0, cy=242.0)
    info = node._camera_info("stamp", 640, 480)

    assert info.k[0] == pytest.approx(600.0)
    assert info.k[4] == pytest.approx(601.0)
    assert info.k[2] == pytest.approx(311.0)
    assert info.k[5] == pytest.approx(242.0)
    # P must carry the same focal length and centre as K for an
    # already-rectified monocular camera.
    assert info.p[0] == pytest.approx(600.0)
    assert info.p[2] == pytest.approx(311.0)
    assert info.p[6] == pytest.approx(242.0)


def test_camera_info_tracks_actual_frame_size(node_factory):
    """A camera that ignored the requested geometry must not be misreported."""
    node = node_factory()
    info = node._camera_info("stamp", 320, 240)

    assert (info.width, info.height) == (320, 240)
    assert info.k[2] == pytest.approx(160.0)
    assert info.k[5] == pytest.approx(120.0)
