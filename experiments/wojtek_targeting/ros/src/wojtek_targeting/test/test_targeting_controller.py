"""State machine tests, against a fake driver and a hand-driven clock.

The driver is faked rather than stubbed at the SDK level because what is under
test here is the decision logic -- when the gimbal is allowed to move and where
it is told to go -- not the bus. test_dynamixel_driver.py covers the bus.
"""

import pytest
from conftest import FakeClock, NodeStub
from std_srvs.srv import SetBool, Trigger
from wojtek_targeting_msgs.msg import LaserTarget, TargetingStatus
from wojtek_targeting_msgs.srv import SetGimbalAngles

from wojtek_targeting.dynamixel_driver import DynamixelError, MotorState
from wojtek_targeting.targeting_controller import TargetingController

SECOND = 1_000_000_000


class FakeDriver:
    def __init__(self, port, baud_rate, motors, logger=None):
        self.motors = {m.motor_id: m for m in motors}
        self.states = {m.motor_id: MotorState(position_deg=0.0, stale=False) for m in motors}
        self.goals = []
        self.connected = False
        self.torque = False
        self.shutdown_called = False
        # Tests set these to make the bus fail.
        self.fail_writes = False
        self.fail_reads = False

    def connect(self):
        self.connected = True

    def enable_torque(self):
        self.torque = True

    def read_states(self):
        if self.fail_reads:
            raise DynamixelError("read failed")
        return dict(self.states)

    @property
    def last_states(self):
        return dict(self.states)

    def write_goals(self, goals_deg):
        if self.fail_writes:
            raise DynamixelError("write failed")
        self.goals.append(dict(goals_deg))
        return dict(goals_deg)

    def shutdown(self):
        self.shutdown_called = True


@pytest.fixture
def controller(clock, request):
    overrides = getattr(request, "param", {})
    NodeStub.overrides = overrides
    node = TargetingController(driver_factory=FakeDriver)
    NodeStub.overrides = {}
    return node


def _tick(node, times=1):
    _, callback = node.timers[0]
    for _ in range(times):
        callback()


def _target(x=0.0, y=0.0, confidence=0.9, detected=True, age_ns=0):
    msg = LaserTarget()
    msg.detected = detected
    msg.confidence = confidence
    msg.target_x = x
    msg.target_y = y
    stamp_ns = FakeClock.now_ns - age_ns
    msg.header.stamp.sec = stamp_ns // SECOND
    msg.header.stamp.nanosec = stamp_ns % SECOND
    return msg


def _camera_info(fx=600.0, fy=600.0, width=640, height=480):
    from sensor_msgs.msg import CameraInfo

    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [fx, 0.0, width / 2.0, 0.0, fy, height / 2.0, 0.0, 0.0, 1.0]
    return info


def _feed(node, target=None, info=None):
    if info is not None:
        node.subscriptions["camera_info"](info)
    if target is not None:
        node.subscriptions["target"](target)


def _status(node):
    return [m for t, m in node.published if t == "status"][-1]


def _call(node, service, request):
    _, callback = node.services[service]
    srv_type = node.services[service][0]
    return callback(request, srv_type.Response())


# --- startup --------------------------------------------------------------

def test_starts_disabled_and_holding_where_it_already_is(controller):
    """Coming up must not move the robot."""
    assert controller._mode == TargetingStatus.MODE_DISABLED
    assert controller._driver.torque
    assert controller._command == (0.0, 0.0)


@pytest.mark.parametrize("controller", [{"pan_home_deg": 30.0}], indirect=True)
def test_startup_holds_the_measured_pose_not_home(controller):
    """Home is a destination, not a startup assumption."""
    assert controller._command[0] == 0.0


def test_disabled_still_writes_goals_so_the_gimbal_holds(controller):
    _tick(controller)
    assert controller._driver.goals


# --- enabling -------------------------------------------------------------

def test_enable_moves_to_searching(controller):
    request = SetBool.Request()
    request.data = True
    response = _call(controller, "enable_tracking", request)

    assert response.success
    assert controller._mode == TargetingStatus.MODE_SEARCHING


def test_enable_warns_when_the_camera_is_uncalibrated(controller):
    request = SetBool.Request()
    request.data = True
    response = _call(controller, "enable_tracking", request)
    assert "intrinsics" in response.message


def test_uncalibrated_camera_info_is_not_accepted(controller):
    _feed(controller, info=_camera_info(fx=0.0, fy=0.0))
    assert controller._intrinsics is None


# --- tracking -------------------------------------------------------------

def _enable(node):
    request = SetBool.Request()
    request.data = True
    _call(node, "enable_tracking", request)


def test_fresh_target_drives_tracking(controller):
    _enable(controller)
    _feed(controller, target=_target(x=0.5), info=_camera_info())
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_TRACKING
    # Target right of centre, pan direction +1, so pan increases.
    assert controller._command[0] > 0.0


def test_tilt_direction_inverts_the_image_convention(controller):
    """A target low in the frame must tilt down, not up."""
    _enable(controller)
    _feed(controller, target=_target(y=0.5), info=_camera_info())
    _tick(controller)

    assert controller._command[1] < 0.0


def test_low_confidence_target_is_ignored(controller):
    _enable(controller)
    _feed(controller, target=_target(x=0.5, confidence=0.1), info=_camera_info())
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_SEARCHING
    assert controller._command == (0.0, 0.0)


def test_undetected_target_is_ignored(controller):
    _enable(controller)
    _feed(controller, target=_target(x=0.5, detected=False), info=_camera_info())
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_SEARCHING


def test_stale_target_stops_the_aim_and_holds(controller):
    """The dead-man: a detector that dies must not leave the gimbal chasing."""
    _enable(controller)
    _feed(controller, target=_target(x=0.5), info=_camera_info())
    _tick(controller)
    aimed = controller._command

    FakeClock.now_ns += SECOND   # target is now a second old
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_SEARCHING
    assert controller._command == aimed   # holds, does not reset


def test_unstamped_target_is_treated_as_infinitely_old(controller):
    _enable(controller)
    target = _target(x=0.5)
    target.header.stamp.sec = 0
    target.header.stamp.nanosec = 0
    _feed(controller, target=target, info=_camera_info())
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_SEARCHING


def test_tracking_without_intrinsics_holds_and_says_why(controller):
    _enable(controller)
    _feed(controller, target=_target(x=0.5))
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_SEARCHING
    assert "intrinsics" in _status(controller).fault_message


def test_aim_is_rate_limited(controller):
    """One bad detection must not command a full-speed traverse."""
    _enable(controller)
    _feed(controller, target=_target(x=1.0), info=_camera_info(fx=50.0, fy=50.0))
    _tick(controller)

    # 45 deg/s at 40 Hz is 1.125 deg in a tick.
    assert controller._command[0] == pytest.approx(45.0 / 40.0, abs=1e-6)


def test_aim_is_clamped_to_limits(controller):
    _enable(controller)
    info = _camera_info(fx=50.0, fy=50.0)
    for _ in range(400):
        _feed(controller, target=_target(x=1.0), info=info)
        FakeClock.now_ns += SECOND // 40
        _tick(controller)

    assert controller._command[0] <= 80.0


# --- locking --------------------------------------------------------------

def test_centred_target_locks_after_the_dwell(controller):
    _enable(controller)
    info = _camera_info()
    _feed(controller, target=_target(x=0.0), info=info)
    _tick(controller)
    assert controller._mode == TargetingStatus.MODE_TRACKING

    FakeClock.now_ns += SECOND // 2   # past the 0.30 s dwell
    _feed(controller, target=_target(x=0.0))
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_LOCKED
    assert _status(controller).locked


def test_target_leaving_the_tolerance_breaks_the_lock(controller):
    _enable(controller)
    info = _camera_info()
    _feed(controller, target=_target(x=0.0), info=info)
    _tick(controller)
    FakeClock.now_ns += SECOND // 2
    _feed(controller, target=_target(x=0.0))
    _tick(controller)
    assert controller._mode == TargetingStatus.MODE_LOCKED

    _feed(controller, target=_target(x=0.8))
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_TRACKING


# --- faults ---------------------------------------------------------------

def test_write_failure_enters_fault_and_stops_commanding(controller):
    _enable(controller)
    _feed(controller, target=_target(x=0.5), info=_camera_info())
    controller._driver.fail_writes = True
    _tick(controller)
    assert controller._mode == TargetingStatus.MODE_FAULT

    controller._driver.goals.clear()
    _tick(controller)
    assert controller._driver.goals == []


def test_fault_reports_the_reason(controller):
    controller._driver.fail_writes = True
    _tick(controller)
    assert "write failed" in _status(controller).fault_message


def test_toggling_tracking_clears_a_fault(controller):
    _feed(controller, info=_camera_info())
    controller._driver.fail_writes = True
    _tick(controller)
    assert controller._mode == TargetingStatus.MODE_FAULT

    controller._driver.fail_writes = False
    off = SetBool.Request()
    off.data = False
    _call(controller, "enable_tracking", off)
    _enable(controller)
    _tick(controller)

    assert controller._mode == TargetingStatus.MODE_SEARCHING
    assert _status(controller).fault_message == ""


def test_disabling_clears_a_fault_so_the_next_one_reads_true(controller):
    controller._driver.fail_writes = True
    _tick(controller)

    off = SetBool.Request()
    off.data = False
    _call(controller, "enable_tracking", off)

    assert controller._fault_message == ""


def test_read_failure_enters_fault(controller):
    controller._driver.fail_reads = True
    _tick(controller, times=controller._state_decimation)
    assert controller._mode == TargetingStatus.MODE_FAULT


# --- manual control -------------------------------------------------------

def test_manual_angles_are_refused_while_tracking(controller):
    _enable(controller)
    request = SetGimbalAngles.Request()
    request.pan_deg = 10.0
    response = _call(controller, "set_manual_angles", request)

    assert not response.success
    assert "disable tracking" in response.message


def test_manual_angles_move_the_gimbal_when_disabled(controller):
    request = SetGimbalAngles.Request()
    request.pan_deg = 12.0
    request.tilt_deg = -8.0
    response = _call(controller, "set_manual_angles", request)

    assert response.success
    assert controller._command == (12.0, -8.0)


def test_manual_angles_are_clamped(controller):
    request = SetGimbalAngles.Request()
    request.pan_deg = 500.0
    request.tilt_deg = -500.0
    response = _call(controller, "set_manual_angles", request)

    assert response.success
    assert controller._command == (80.0, -35.0)
    assert "clamped" in response.message


def test_manual_angles_are_refused_in_fault(controller):
    controller._driver.fail_writes = True
    _tick(controller)
    response = _call(controller, "set_manual_angles", SetGimbalAngles.Request())
    assert not response.success


def test_home_is_refused_while_tracking(controller):
    _enable(controller)
    response = _call(controller, "home", Trigger.Request())
    assert not response.success


@pytest.mark.parametrize("controller", [{"pan_home_deg": 5.0, "tilt_home_deg": -3.0}],
                         indirect=True)
def test_home_moves_to_the_configured_pose(controller):
    response = _call(controller, "home", Trigger.Request())
    assert response.success
    assert controller._command == (5.0, -3.0)


@pytest.mark.parametrize("controller", [{"home_on_disable": True, "pan_home_deg": 7.0}],
                         indirect=True)
def test_disable_can_be_configured_to_go_home(controller):
    _enable(controller)
    off = SetBool.Request()
    off.data = False
    _call(controller, "enable_tracking", off)

    assert controller._command[0] == 7.0


# --- outputs --------------------------------------------------------------

def test_calibration_offsets_reach_the_servos_but_not_the_status(controller):
    """The offset trims the hardware; the status should still read in joint terms."""
    NodeStub.overrides = {"pan_offset_deg": 4.0}
    node = TargetingController(driver_factory=FakeDriver)
    NodeStub.overrides = {}

    _tick(node)

    assert node._driver.goals[-1][1] == pytest.approx(4.0)
    assert _status(node).requested_pan_deg == pytest.approx(0.0)


def test_gimbal_state_is_published_in_radians(controller):
    controller._driver.states[1] = MotorState(position_deg=90.0, stale=False)
    _tick(controller, times=controller._state_decimation)

    joint = [m for t, m in controller.published if t == "gimbal_state"][-1]
    assert joint.position[0] == pytest.approx(1.5707963, abs=1e-5)
    assert joint.name == ["targeting_pan_joint", "targeting_tilt_joint"]


def test_shutdown_releases_the_driver(controller):
    controller.destroy_node()
    assert controller._driver.shutdown_called
