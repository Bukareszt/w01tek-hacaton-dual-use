"""Stubs standing in for ROS, so the follow node can be tested without one.

There is no ROS install in the environment this package is developed in, and
there is no robot either.  Everything in `core/` is ROS-free by construction
and needs none of this.  The node itself is a thin layer of callbacks and
timers over that arithmetic, and this file is what lets those callbacks be
called.

The stubs are the smallest thing that carries the fields of the real messages.
They prove nothing about whether a real `LaserTarget` has these fields; the
contract in the README and in PR 6's message definitions is what says that.
What they do prove is that the node parses a track, fills a target, gates its
service calls and publishes only when it should.

The fake clock is driven by hand.  Loss timers and coast windows are time
thresholds, and testing them against the wall clock would mean either sleeping
or accepting flakes.
"""

import sys
import types

import pytest


class FakeClock:
    now_ns = 0

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return FakeClock.now_ns

    def to_msg(self):
        return Stamp(FakeClock.now_ns // 1_000_000_000,
                     FakeClock.now_ns % 1_000_000_000)

    @classmethod
    def set(cls, seconds):
        cls.now_ns = int(seconds * 1_000_000_000)


class Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class Header:
    def __init__(self):
        self.stamp = Stamp()
        self.frame_id = ""


class Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


def _message(**fields):
    def init(self):
        self.header = Header()
        for key, value in fields.items():
            setattr(self, key, value() if callable(value) else value)

    return type("Msg", (), {"__init__": init})


class FakeFuture:
    def __init__(self, request):
        self.request = request
        self._done = False

    def done(self):
        return self._done


class FakeClient:
    """A service client that records its calls and can be made unavailable."""

    def __init__(self, srv_type, name):
        self.srv_type = srv_type
        self.name = name
        self.ready = True
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        return FakeFuture(request)


class Logger:
    def __init__(self):
        self.messages = []

    def _record(self, level, msg):
        self.messages.append((level, msg))

    def info(self, msg, **kw):
        self._record("info", msg)

    def warn(self, msg, **kw):
        self._record("warn", msg)

    def error(self, msg, **kw):
        self._record("error", msg)

    def fatal(self, msg, **kw):
        self._record("fatal", msg)


class Param:
    def __init__(self, value):
        self.value = value


class Publisher:
    def __init__(self, topic, sink):
        self.topic = topic
        self._sink = sink

    def publish(self, msg):
        self._sink.append((self.topic, msg))


class Node:
    overrides = {}

    def __init__(self, name):
        self.name = name
        self._logger = Logger()
        self.timers = []
        self.services = {}
        self.subscriptions = {}
        self.clients = {}
        self.published = []

    def declare_parameter(self, name, default):
        return Param(Node.overrides.get(name, default))

    def create_publisher(self, msg_type, topic, qos):
        return Publisher(topic, self.published)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_service(self, srv_type, name, callback):
        self.services[name] = (srv_type, callback)

    def create_client(self, srv_type, name):
        client = FakeClient(srv_type, name)
        self.clients[name] = client
        return client

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return FakeClock()

    def destroy_node(self):
        pass


def _install_ros_stub():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.try_shutdown = lambda *a, **k: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = Node
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    rclpy.node = rclpy_node
    rclpy.qos = rclpy_qos
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    CameraInfo = _message(width=0, height=0, k=lambda: [0.0] * 9)
    JointState = _message(name=list, position=list, velocity=list, effort=list)
    Image = _message(height=0, width=0, encoding="16UC1", is_bigendian=0,
                     step=0, data=bytes)
    Imu = _message(angular_velocity=Vector3, linear_acceleration=Vector3)
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = CameraInfo
    sensor_msgs_msg.JointState = JointState
    sensor_msgs_msg.Image = Image
    sensor_msgs_msg.Imu = Imu
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    String = _message(data="")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    Twist = _message(linear=Vector3, angular=Vector3)
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = Twist
    geometry_msgs_msg.Vector3 = Vector3
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    class SetBool:
        Request = _message(data=False)
        Response = _message(success=False, message="")

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = SetBool
    std_srvs.srv = std_srvs_srv
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv

    LaserTarget = _message(
        detected=False, class_name="", target_x=0.0, target_y=0.0,
        distance_m=0.0, confidence=0.0)
    TargetingStatus = _message(
        mode=0, target_fresh=False, locked=False,
        requested_pan_deg=0.0, requested_tilt_deg=0.0,
        actual_pan_deg=0.0, actual_tilt_deg=0.0,
        detection_age_s=0.0, fault_message="")
    TargetingStatus.MODE_DISABLED = 0
    TargetingStatus.MODE_SEARCHING = 1
    TargetingStatus.MODE_TRACKING = 2
    TargetingStatus.MODE_LOCKED = 3
    TargetingStatus.MODE_FAULT = 4

    msgs = types.ModuleType("wojtek_targeting_msgs")
    msgs_msg = types.ModuleType("wojtek_targeting_msgs.msg")
    msgs_msg.LaserTarget = LaserTarget
    msgs_msg.TargetingStatus = TargetingStatus
    msgs.msg = msgs_msg
    sys.modules["wojtek_targeting_msgs"] = msgs
    sys.modules["wojtek_targeting_msgs.msg"] = msgs_msg


_install_ros_stub()


@pytest.fixture
def clock():
    # Not zero: a stamp of zero is how ROS says "never stamped", and several
    # things here treat it that way.
    FakeClock.set(1000.0)
    return FakeClock


@pytest.fixture
def node(clock):
    """A follow node with stubbed ROS underneath it.

    Imported inside the fixture so the stubs are in `sys.modules` first.
    """
    from wojtek_follow.follow_node import FollowNode

    Node.overrides = {}
    return FollowNode()
