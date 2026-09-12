"""The node's callbacks and its two timers, over the stubbed ROS in conftest.

Nothing here proves the node talks to a real ROS graph.  What it proves is that
a track off a websocket becomes a target and a velocity, that a malformed one
becomes a log line instead of a crash, that the gimbal's tracking switch is
flipped once per lock and not once per message, and that nothing at all is
published while the state is IDLE.
"""

import json
import math

import numpy as np
import pytest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import String
from wojtek_targeting_msgs.msg import TargetingStatus

TRACK_TOPIC = "/wojtek/track/target"
GIMBAL_TOPIC = "/targeting/gimbal_state"
STATUS_TOPIC = "/targeting/status"
TOWER_INFO_TOPIC = "/targeting_camera/targeting_camera/camera_info"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info"
IMU_TOPIC = "/imu_sensor_broadcaster/imu"
TARGET_TOPIC = "/targeting/target"
CMD_TOPIC = "/wojtek/follow/cmd_vel"
FOLLOW_STATUS_TOPIC = "/wojtek/follow/status"


def send(node, topic, msg):
    node.subscriptions[topic](msg)


def track_json(cam="tower", **kw):
    payload = {
        "cx": 320, "cy": 240, "w": 60, "h": 60, "fw": 640, "fh": 480,
        "label": "bottle", "age": 0.0, "cam": cam, "stamp": 1000.0,
    }
    payload.update(kw)
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def camera_info(width=640, height=480, fx=500.0, cx=320.0, cy=240.0):
    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
    return msg


def depth_info():
    return camera_info(width=424, height=240, fx=210.0, cx=212.0, cy=120.0)


def depth_image(distance_mm=2000, pad_bytes=0):
    """A wall at a constant depth, with an optional row stride longer than the width."""
    height, width = 240, 424
    rows = np.full((height, width + pad_bytes // 2), distance_mm, dtype=np.uint16)
    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = "16UC1"
    msg.is_bigendian = 0
    msg.step = rows.shape[1] * 2
    msg.data = rows.tobytes()
    return msg


def gimbal_state(pan_deg=0.0, tilt_deg=0.0, stamp=1000.0):
    msg = JointState()
    msg.name = ["targeting_pan_joint", "targeting_tilt_joint"]
    msg.position = [math.radians(pan_deg), math.radians(tilt_deg)]
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    return msg


def imu(wy=0.0, wz=0.0):
    msg = Imu()
    msg.angular_velocity.y = wy
    msg.angular_velocity.z = wz
    return msg


def published(node, topic):
    return [msg for name, msg in node.published if name == topic]


def enable_calls(node):
    return [request.data for request in node.clients["/targeting/enable_tracking"].calls]


def warnings(node):
    return [text for level, text in node.get_logger().messages if level == "warn"]


def status_of(node):
    return json.loads(published(node, FOLLOW_STATUS_TOPIC)[-1].data)


def lock_on(node, **kw):
    send(node, TOWER_INFO_TOPIC, camera_info())
    send(node, TRACK_TOPIC, track_json(**kw))


def aim_tick(node):
    node.timers[0][1]()


def body_tick(node):
    node.timers[1][1]()


# --- wiring ---------------------------------------------------------------

def test_the_node_subscribes_to_the_whole_contract(node):
    assert set(node.subscriptions) == {
        TRACK_TOPIC, GIMBAL_TOPIC, STATUS_TOPIC, TOWER_INFO_TOPIC,
        DEPTH_TOPIC, DEPTH_INFO_TOPIC, IMU_TOPIC}
    assert "/targeting/enable_tracking" in node.clients


def test_the_two_timers_run_at_forty_and_ten(node):
    periods = sorted(period for period, _ in node.timers)
    assert periods == pytest.approx([0.025, 0.1])


# --- the track ------------------------------------------------------------

def test_a_track_starts_the_lock_and_the_gimbal(node):
    lock_on(node)
    assert node._relay.has_track
    assert enable_calls(node) == [True]


def test_the_gimbal_is_only_switched_on_once(node):
    lock_on(node)
    send(node, TRACK_TOPIC, track_json())
    send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True]


def test_unlock_ends_the_lock_and_the_gimbal(node):
    lock_on(node)
    unlock = String()
    unlock.data = json.dumps({"unlock": True, "stamp": 1001.0})
    send(node, TRACK_TOPIC, unlock)
    assert not node._relay.has_track
    assert enable_calls(node) == [True, False]


def test_unlocking_twice_only_speaks_once(node):
    lock_on(node)
    unlock = String()
    unlock.data = json.dumps({"unlock": True, "stamp": 1001.0})
    send(node, TRACK_TOPIC, unlock)
    send(node, TRACK_TOPIC, unlock)
    assert enable_calls(node) == [True, False]


def test_a_front_camera_track_is_refused(node):
    """A front-camera pixel carries no tower bearing, so following it is a lie."""
    send(node, TOWER_INFO_TOPIC, camera_info())
    send(node, TRACK_TOPIC, track_json(cam="front"))
    assert not node._relay.has_track
    assert enable_calls(node) == []
    assert any("front" in text for text in warnings(node))


def test_a_track_with_no_camera_field_is_assumed_to_be_the_tower(node):
    send(node, TOWER_INFO_TOPIC, camera_info())
    msg = String()
    msg.data = json.dumps({"cx": 320, "cy": 240, "w": 60, "h": 60,
                           "fw": 640, "fh": 480, "label": "cup", "age": 0.0})
    send(node, TRACK_TOPIC, msg)
    assert node._relay.has_track


def test_rubbish_on_the_track_topic_is_a_log_line(node):
    msg = String()
    msg.data = "{not json"
    send(node, TRACK_TOPIC, msg)
    msg.data = "[1, 2, 3]"
    send(node, TRACK_TOPIC, msg)
    msg.data = json.dumps({"cx": 1, "cy": 2})
    send(node, TRACK_TOPIC, msg)
    msg.data = json.dumps({"cx": 1, "cy": 2, "w": 3, "h": 4, "fw": 0, "fh": 0})
    send(node, TRACK_TOPIC, msg)
    assert not node._relay.has_track
    assert len(warnings(node)) == 4


def test_a_track_with_no_stamp_is_stamped_on_arrival(node, clock):
    clock.set(1234.0)
    send(node, TOWER_INFO_TOPIC, camera_info())
    msg = String()
    msg.data = json.dumps({"cx": 320, "cy": 240, "w": 60, "h": 60,
                           "fw": 640, "fh": 480, "cam": "tower"})
    send(node, TRACK_TOPIC, msg)
    assert node._relay.track["stamp"] == pytest.approx(1234.0)


def test_the_gimbal_switch_is_not_thrown_at_a_service_that_is_not_there(node):
    node.clients["/targeting/enable_tracking"].ready = False
    lock_on(node)
    assert enable_calls(node) == []
    assert node._relay.has_track
    # The node keeps aiming, and asks again when the gimbal turns up.
    node.clients["/targeting/enable_tracking"].ready = True
    send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True]


def test_a_stop_that_never_went_out_does_not_block_the_next_start(node):
    """A request that was never sent must not be remembered as sent."""
    lock_on(node)
    node.clients["/targeting/enable_tracking"].ready = False
    unlock = String()
    unlock.data = json.dumps({"unlock": True})
    send(node, TRACK_TOPIC, unlock)
    node.clients["/targeting/enable_tracking"].ready = True
    send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True, True]


# --- the gimbal's own messages -------------------------------------------

def test_the_gimbal_state_reaches_the_relay(node, clock):
    lock_on(node)
    send(node, GIMBAL_TOPIC, gimbal_state(pan_deg=10.0, stamp=1000.0))
    assert node._relay.gimbal.latest()[1] == pytest.approx(math.radians(10.0))


def test_a_gimbal_state_without_the_expected_joints_is_a_log_line(node):
    msg = JointState()
    msg.name = ["some_other_joint"]
    msg.position = [0.0]
    send(node, GIMBAL_TOPIC, msg)
    assert warnings(node)
    assert node._relay.gimbal.latest() is None


def test_the_gimbal_mode_is_carried_into_the_status(node):
    msg = TargetingStatus()
    msg.mode = TargetingStatus.MODE_LOCKED
    send(node, STATUS_TOPIC, msg)
    body_tick(node)
    assert status_of(node)["gimbal_mode"] == TargetingStatus.MODE_LOCKED


# --- the cameras ----------------------------------------------------------

def test_an_uncalibrated_tower_camera_is_refused(node):
    send(node, TOWER_INFO_TOPIC, camera_info(fx=0.0))
    send(node, TRACK_TOPIC, track_json())
    aim_tick(node)
    assert published(node, TARGET_TOPIC) == []
    assert any("focal length" in text for text in warnings(node))


def test_the_depth_image_is_decoded_without_cv_bridge(node):
    send(node, DEPTH_INFO_TOPIC, depth_info())
    send(node, DEPTH_TOPIC, depth_image(1500))
    assert node._depth.shape == (240, 424)
    assert int(node._depth[120, 212]) == 1500


def test_a_row_stride_longer_than_the_image_is_cut_back(node):
    send(node, DEPTH_TOPIC, depth_image(1500, pad_bytes=16))
    assert node._depth.shape == (240, 424)
    assert int(node._depth[0, 423]) == 1500


def test_a_depth_image_in_the_wrong_encoding_is_refused(node):
    msg = depth_image()
    msg.encoding = "32FC1"
    send(node, DEPTH_TOPIC, msg)
    assert node._depth is None
    assert any("16UC1" in text for text in warnings(node))


def test_a_short_depth_image_is_refused(node):
    msg = depth_image()
    msg.data = msg.data[:100]
    send(node, DEPTH_TOPIC, msg)
    assert node._depth is None


# --- the aim relay at 40 Hz ----------------------------------------------

def test_nothing_is_aimed_at_before_a_lock(node):
    aim_tick(node)
    assert published(node, TARGET_TOPIC) == []


def test_the_aim_tick_publishes_a_target(node, clock):
    lock_on(node)
    aim_tick(node)
    target = published(node, TARGET_TOPIC)[-1]
    assert target.detected is True
    assert target.class_name == "bottle"
    assert target.target_x == pytest.approx(0.0)
    assert target.confidence == 1.0


def test_the_imu_moves_the_aim_between_deck_frames(node, clock):
    """This is the whole point of the relay: the body turns and the aim keeps up."""
    lock_on(node)
    send(node, IMU_TOPIC, imu(wz=math.radians(50.0)))
    clock.set(1000.2)
    send(node, IMU_TOPIC, imu(wz=math.radians(50.0)))
    aim_tick(node)
    target = published(node, TARGET_TOPIC)[-1]
    assert target.target_x > 0.1
    assert target.confidence == 1.0


def test_the_aim_coasts_at_zero_confidence_when_the_deck_goes_quiet(node, clock):
    lock_on(node)
    clock.set(1001.0)
    aim_tick(node)
    target = published(node, TARGET_TOPIC)[-1]
    assert target.confidence == 0.0
    assert target.detected is True


# --- the body loop at 10 Hz ----------------------------------------------

def test_an_idle_body_loop_publishes_no_velocity(node):
    body_tick(node)
    assert published(node, CMD_TOPIC) == []
    status = status_of(node)
    assert status["state"] == "IDLE"
    assert status["range_source"] == "none"
    assert status["azimuth_deg"] is None


def test_the_body_loop_walks_toward_a_target_it_can_measure(node):
    send(node, DEPTH_INFO_TOPIC, depth_info())
    send(node, DEPTH_TOPIC, depth_image(2000))
    lock_on(node)
    body_tick(node)
    command = published(node, CMD_TOPIC)[-1]
    assert command.linear.x == pytest.approx(0.4)
    assert command.angular.z == pytest.approx(0.0)
    status = status_of(node)
    assert status["state"] == "APPROACH"
    assert status["range_m"] == pytest.approx(2.0, abs=0.05)
    assert status["range_source"] == "depth"
    assert status["azimuth_deg"] == pytest.approx(0.0)


def test_without_depth_the_range_is_a_guess_and_the_body_does_not_walk(node):
    lock_on(node)
    body_tick(node)
    command = published(node, CMD_TOPIC)[-1]
    assert command.linear.x == 0.0
    status = status_of(node)
    assert status["range_source"] == "bbox"
    assert status["range_m"] == pytest.approx(500.0 * 0.07 / 60.0, abs=0.01)


def test_the_body_turns_toward_a_target_off_to_the_side(node):
    """A target near the right edge of the tower picture is 31 degrees away."""
    lock_on(node, cx=620)
    body_tick(node)
    command = published(node, CMD_TOPIC)[-1]
    assert command.angular.z < 0.0
    assert command.linear.x == 0.0
    status = status_of(node)
    assert status["state"] == "ALIGN"
    assert status["azimuth_deg"] == pytest.approx(-31.0, abs=0.5)


def test_the_status_carries_the_obstacle_bands(node):
    send(node, DEPTH_INFO_TOPIC, depth_info())
    send(node, DEPTH_TOPIC, depth_image(2000))
    lock_on(node)
    body_tick(node)
    bands = status_of(node)["nearest_obstacle_m"]
    assert set(bands) == {"left", "centre", "right"}
    assert bands["left"] == pytest.approx(2.0, abs=0.3)


def test_a_lock_that_goes_quiet_stops_the_body_and_then_drops(node, clock):
    lock_on(node)
    clock.set(1001.5)
    body_tick(node)
    assert status_of(node)["state"] == "LOST"
    assert published(node, CMD_TOPIC)[-1].linear.x == 0.0

    clock.set(1004.0)
    body_tick(node)
    assert enable_calls(node) == [True, False]
    assert not node._relay.has_track
    assert status_of(node)["state"] == "IDLE"

    # And then it is quiet.
    before = len(published(node, CMD_TOPIC))
    body_tick(node)
    assert len(published(node, CMD_TOPIC)) == before
    aim_tick(node)
    assert published(node, TARGET_TOPIC) == []


def test_the_measured_range_rides_along_to_the_gimbal(node):
    send(node, DEPTH_INFO_TOPIC, depth_info())
    send(node, DEPTH_TOPIC, depth_image(2000))
    lock_on(node)
    body_tick(node)
    aim_tick(node)
    assert published(node, TARGET_TOPIC)[-1].distance_m == pytest.approx(2.0, abs=0.05)


def test_a_guessed_range_does_not_ride_along(node):
    lock_on(node)
    body_tick(node)
    aim_tick(node)
    assert published(node, TARGET_TOPIC)[-1].distance_m == 0.0


def test_the_status_is_published_every_tick(node):
    body_tick(node)
    body_tick(node)
    assert len(published(node, FOLLOW_STATUS_TOPIC)) == 2


def test_the_default_signs_are_the_tower_controllers_negated_directions(node):
    assert node._aim_params.pan_sign == -1.0
    assert node._aim_params.tilt_sign == 1.0


# --- a depth camera that stops ------------------------------------------

def test_a_stale_depth_frame_does_not_drive_the_walk(node, clock):
    """The RealSense drops off the bus mid-approach.

    Its last frame still shows the wall behind the person at 2 m.  Walking
    on it would walk into the person, so past depth_max_age_s the frame is
    withheld: the range falls back to the box, the bands empty, the body
    stands.
    """
    send(node, DEPTH_INFO_TOPIC, depth_info())
    send(node, DEPTH_TOPIC, depth_image(2000))
    lock_on(node)
    body_tick(node)
    assert published(node, CMD_TOPIC)[-1].linear.x == pytest.approx(0.4)

    clock.set(1000.8)
    body_tick(node)
    command = published(node, CMD_TOPIC)[-1]
    assert command.linear.x == 0.0
    status = status_of(node)
    assert status["range_source"] == "bbox"
    assert status["nearest_obstacle_m"] == {"left": None, "centre": None, "right": None}
    assert status["depth_age_s"] == pytest.approx(0.8)
    assert any("depth" in text for text in warnings(node))

    # A fresh frame and the walk resumes.
    send(node, DEPTH_TOPIC, depth_image(2000))
    body_tick(node)
    assert published(node, CMD_TOPIC)[-1].linear.x == pytest.approx(0.4)
    assert status_of(node)["range_source"] == "depth"


def test_a_stamped_depth_frame_is_aged_by_its_stamp(node, clock):
    send(node, DEPTH_INFO_TOPIC, depth_info())
    msg = depth_image(2000)
    msg.header.stamp.sec = 999
    send(node, DEPTH_TOPIC, msg)
    lock_on(node)
    body_tick(node)
    assert status_of(node)["range_source"] == "bbox"
    assert status_of(node)["depth_age_s"] == pytest.approx(1.0)


# --- a lock with nothing to aim by ---------------------------------------

def test_a_lock_without_tower_intrinsics_waits_and_still_drops(node, clock):
    """No camera_info ever arrives: the status says so, and a track the Deck
    stops feeding is dropped exactly as any other."""
    send(node, TRACK_TOPIC, track_json())
    body_tick(node)
    status = status_of(node)
    assert status["state"] == "WAITING"
    assert TOWER_INFO_TOPIC in status["waiting_for"]
    assert published(node, CMD_TOPIC) == []
    assert any("camera_info" in text for text in warnings(node))
    assert enable_calls(node) == [True]

    clock.set(1004.0)
    body_tick(node)
    assert enable_calls(node) == [True, False]
    assert not node._relay.has_track
    assert status_of(node)["state"] == "IDLE"
    # The status goes out before the drop, so the next tick is the quiet one.
    body_tick(node)
    assert status_of(node)["waiting_for"] is None
    assert published(node, CMD_TOPIC) == []


# --- a gimbal that faults or restarts mid-lock ---------------------------

def gimbal_mode(mode):
    msg = TargetingStatus()
    msg.mode = mode
    return msg


def test_a_restarted_gimbal_is_asked_to_track_again(node, clock):
    lock_on(node)
    assert enable_calls(node) == [True]
    # Right after the request its status may still say DISABLED: not yet.
    clock.set(1000.5)
    send(node, STATUS_TOPIC, gimbal_mode(TargetingStatus.MODE_DISABLED))
    send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True]
    # Past the grace it is the controller's word over the request.
    clock.set(1001.5)
    send(node, STATUS_TOPIC, gimbal_mode(TargetingStatus.MODE_DISABLED))
    assert any("DISABLED" in text for text in warnings(node))
    send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True, True]


def test_a_faulted_gimbal_is_asked_again_too(node, clock):
    lock_on(node)
    clock.set(1001.5)
    send(node, STATUS_TOPIC, gimbal_mode(TargetingStatus.MODE_FAULT))
    send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True, True]


def test_a_tracking_gimbal_is_left_alone(node, clock):
    lock_on(node)
    clock.set(1001.5)
    for mode in (TargetingStatus.MODE_SEARCHING, TargetingStatus.MODE_TRACKING,
                 TargetingStatus.MODE_LOCKED):
        send(node, STATUS_TOPIC, gimbal_mode(mode))
        send(node, TRACK_TOPIC, track_json())
    assert enable_calls(node) == [True]


def test_a_disabled_gimbal_without_a_lock_is_nothing_to_reconcile(node, clock):
    clock.set(1001.5)
    send(node, STATUS_TOPIC, gimbal_mode(TargetingStatus.MODE_DISABLED))
    assert enable_calls(node) == []
    assert warnings(node) == []


def test_a_quiet_gimbal_state_is_a_warning_and_the_bearing_holds(node, clock):
    lock_on(node)
    send(node, GIMBAL_TOPIC, gimbal_state(pan_deg=10.0, stamp=1000.0))
    body_tick(node)
    azimuth = status_of(node)["azimuth_deg"]
    assert azimuth == pytest.approx(-10.0)     # pan_sign -1: pan +10 is right
    clock.set(1000.9)
    body_tick(node)
    assert any("gimbal_state" in text for text in warnings(node))
    assert status_of(node)["azimuth_deg"] == pytest.approx(azimuth)


def test_the_twist_only_ever_carries_the_three_axes_the_policy_takes(node):
    send(node, DEPTH_INFO_TOPIC, depth_info())
    send(node, DEPTH_TOPIC, depth_image(2000))
    lock_on(node, cx=400)
    body_tick(node)
    command = published(node, CMD_TOPIC)[-1]
    assert isinstance(command, Twist)
    assert command.linear.z == 0.0
    assert command.angular.x == 0.0
    assert command.angular.y == 0.0
