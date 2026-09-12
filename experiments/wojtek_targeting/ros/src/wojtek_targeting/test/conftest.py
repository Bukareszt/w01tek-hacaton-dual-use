"""Stubs standing in for the Dynamixel SDK and ROS.

There is no U2D2, no servo and no ROS install in the environment this package
is developed in, so the parts of it that can be tested without hardware are
tested against fakes. That covers the conversions, the limits, the
read-failure handling and the state machine -- which is most of what there is
to get wrong that is not the hardware's own behaviour. It proves nothing about
whether a real XM540 answers at 57600 baud.

The fake bus models the one thing about the real one that matters most here:
a read can fail, and a failed read must not be mistaken for a reading of zero.
"""

import sys
import types

import pytest

COMM_SUCCESS = 0
COMM_RX_TIMEOUT = -3001


class FakeServo:
    """Register file for one servo, with the control table's signedness."""

    def __init__(self, motor_id, position_raw=2048):
        self.motor_id = motor_id
        self.registers = {
            64: 0,      # torque enable
            11: 3,      # operating mode
            80: 0, 82: 0, 84: 800,
            108: 0, 112: 0,
            116: position_raw,   # goal position
            126: 0,              # present current (int16)
            128: 0,              # present velocity (int32)
            132: position_raw,   # present position (int32)
            146: 35,             # temperature
        }
        # Tests flip this to make the servo stop answering.
        self.responding = True


class FakeBus:
    """Shared state the stubbed SDK handlers read and write."""

    def __init__(self):
        self.servos = {}
        self.port_open = False
        self.baud = None
        self.sync_writes = []

    def add(self, servo):
        self.servos[servo.motor_id] = servo


BUS = FakeBus()


def _install_sdk_stub():
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = COMM_SUCCESS

    class PortHandler:
        def __init__(self, name):
            self.name = name

        def openPort(self):
            BUS.port_open = True
            return True

        def setBaudRate(self, baud):
            BUS.baud = baud
            return True

        def closePort(self):
            BUS.port_open = False

    class PacketHandler:
        def __init__(self, protocol):
            self.protocol = protocol

        def _write(self, motor_id, addr, value):
            servo = BUS.servos.get(motor_id)
            if servo is None or not servo.responding:
                return COMM_RX_TIMEOUT, 0
            servo.registers[addr] = value
            if addr == 116:
                # A real servo converges on the goal; the fake one arrives at
                # once, which is all the tests need.
                servo.registers[132] = value
            return COMM_SUCCESS, 0

        def write1ByteTxRx(self, port, motor_id, addr, value):
            return self._write(motor_id, addr, value)

        write2ByteTxRx = write1ByteTxRx
        write4ByteTxRx = write1ByteTxRx

        def _read(self, motor_id, addr):
            servo = BUS.servos.get(motor_id)
            if servo is None or not servo.responding:
                return 0, COMM_RX_TIMEOUT, 0
            return servo.registers.get(addr, 0), COMM_SUCCESS, 0

        def read1ByteTxRx(self, port, motor_id, addr):
            return self._read(motor_id, addr)

        read2ByteTxRx = read1ByteTxRx
        read4ByteTxRx = read1ByteTxRx

        def getTxRxResult(self, result):
            return f"comm result {result}"

        def getRxPacketError(self, error):
            return f"packet error {error}"

    class GroupSyncWrite:
        def __init__(self, port, packet, addr, length):
            self.addr = addr
            self.length = length
            self.params = {}

        def addParam(self, motor_id, data):
            self.params[motor_id] = bytes(data)
            return True

        def txPacket(self):
            if not self.params:
                return COMM_RX_TIMEOUT
            for motor_id, data in self.params.items():
                servo = BUS.servos.get(motor_id)
                if servo is None or not servo.responding:
                    return COMM_RX_TIMEOUT
                value = int.from_bytes(data, "little", signed=True)
                servo.registers[self.addr] = value
                if self.addr == 116:
                    servo.registers[132] = value
            BUS.sync_writes.append(dict(self.params))
            return COMM_SUCCESS

        def clearParam(self):
            self.params = {}

    class GroupSyncRead:
        def __init__(self, port, packet, addr, length):
            self.addr = addr
            self.length = length
            self.ids = []

        def addParam(self, motor_id):
            self.ids.append(motor_id)
            return True

        def txRxPacket(self):
            return COMM_SUCCESS

        def isAvailable(self, motor_id, addr, length):
            servo = BUS.servos.get(motor_id)
            return servo is not None and servo.responding

        def getData(self, motor_id, addr, length):
            servo = BUS.servos[motor_id]
            value = servo.registers.get(addr, 0)
            # The SDK hands back unsigned words; the driver is responsible for
            # reinterpreting the signed ones.
            return value & ((1 << (8 * length)) - 1)

        def clearParam(self):
            self.ids = []

    sdk.PortHandler = PortHandler
    sdk.PacketHandler = PacketHandler
    sdk.GroupSyncWrite = GroupSyncWrite
    sdk.GroupSyncRead = GroupSyncRead
    sys.modules["dynamixel_sdk"] = sdk


class FakeClock:
    """A clock the tests drive by hand.

    Lock dwell and detection age are both time thresholds, and testing them
    against the wall clock would mean either sleeping or accepting flakes.
    """

    now_ns = 0

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return FakeClock.now_ns

    def to_msg(self):
        return Stamp(FakeClock.now_ns // 1_000_000_000,
                     FakeClock.now_ns % 1_000_000_000)


class Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class Header:
    def __init__(self):
        self.stamp = Stamp()
        self.frame_id = ""


def _install_ros_stub():
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

    class Node:
        overrides = {}

        def __init__(self, name):
            self.name = name
            self._logger = Logger()
            self.timers = []
            self.services = {}
            self.subscriptions = {}
            self.published = []

        def declare_parameter(self, name, default):
            return Param(Node.overrides.get(name, default))

        def create_publisher(self, msg_type, topic, qos):
            published = self.published

            class Pub:
                def publish(self, msg):
                    published.append((topic, msg))

            return Pub()

        def create_subscription(self, msg_type, topic, callback, qos):
            self.subscriptions[topic] = callback

        def create_service(self, srv_type, name, callback):
            self.services[name] = (srv_type, callback)

        def create_timer(self, period, callback):
            self.timers.append((period, callback))

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return FakeClock()

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

    def _message(**fields):
        def init(self):
            self.header = Header()
            for key, value in fields.items():
                setattr(self, key, value() if callable(value) else value)

        return type("Msg", (), {"__init__": init})

    CameraInfo = _message(width=0, height=0, k=lambda: [0.0] * 9)
    JointState = _message(name=list, position=list, velocity=list, effort=list)
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = CameraInfo
    sensor_msgs_msg.JointState = JointState
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    class SetBool:
        Request = _message(data=False)
        Response = _message(success=False, message="")

    class Trigger:
        Request = _message()
        Response = _message(success=False, message="")

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = SetBool
    std_srvs_srv.Trigger = Trigger
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

    class SetGimbalAngles:
        Request = _message(pan_deg=0.0, tilt_deg=0.0)
        Response = _message(success=False, message="")

    msgs = types.ModuleType("wojtek_targeting_msgs")
    msgs_msg = types.ModuleType("wojtek_targeting_msgs.msg")
    msgs_srv = types.ModuleType("wojtek_targeting_msgs.srv")
    msgs_msg.LaserTarget = LaserTarget
    msgs_msg.TargetingStatus = TargetingStatus
    msgs_srv.SetGimbalAngles = SetGimbalAngles
    msgs.msg = msgs_msg
    msgs.srv = msgs_srv
    sys.modules["wojtek_targeting_msgs"] = msgs
    sys.modules["wojtek_targeting_msgs.msg"] = msgs_msg
    sys.modules["wojtek_targeting_msgs.srv"] = msgs_srv

    return Node


_install_sdk_stub()
NodeStub = _install_ros_stub()


@pytest.fixture
def clock():
    # Not zero: the controller treats an all-zero stamp as "never stamped", so
    # a clock at the epoch would make every freshly stamped target look
    # unstamped and nothing would ever track.
    FakeClock.now_ns = 1000 * 1_000_000_000
    return FakeClock


@pytest.fixture
def bus(tmp_path, monkeypatch):
    """A fresh two-servo bus on a real (empty) file, so flock works."""
    BUS.servos = {}
    BUS.sync_writes = []
    BUS.port_open = False
    port = tmp_path / "wojtek_gimbal"
    port.write_bytes(b"")
    BUS.add(FakeServo(1))
    BUS.add(FakeServo(2))
    BUS.port_path = str(port)
    return BUS
