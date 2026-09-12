"""The aim relay, and every sign in it.

The three cases that matter are at the top of this file.  Turning the body
moves a fixed target across the picture one way.  Turning the tower moves it
the same way, because both of them turn the camera.  Doing one of each in
opposite directions is what holding a lock looks like, and it must leave the
predicted pixel exactly where it was.
"""

import math

import pytest

from wojtek_follow.core.aim import AimParams, AimRelay, GimbalHistory, GyroIntegral
from wojtek_follow.core.geometry import Intrinsics

TOWER = Intrinsics(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
T0 = 1000.0
STEP = 0.2


def make_relay(pan_sign=1.0, tilt_sign=1.0, **kw):
    """A relay with the signs spelled out and no round trip.

    The sign tests below are about the arithmetic, not the tower's mounting,
    so they fix `pan_sign` at +1 rather than taking the package default.  The
    round trip is zero here so a track's stamp is its capture time; the
    latency tests at the bottom set it on purpose.
    """
    params = dict(pan_sign=pan_sign, tilt_sign=tilt_sign, track_latency_s=0.0)
    params.update(kw)
    relay = AimRelay(AimParams(**params))
    relay.set_intrinsics(TOWER)
    return relay


def test_the_default_signs_follow_the_tower_controllers_driver():
    """pan_sign = -pan_direction and tilt_sign = -tilt_direction.

    The tower controller's driver applies `direction` twice, once in the
    command and once on the way to the encoder, so a target to the right
    always raises the raw count and `direction` only signs the reported
    angle.  With its defaults of pan_direction +1 and tilt_direction -1, a
    positive reported pan is camera right and a positive reported tilt is
    camera up.  See the README.
    """
    params = AimParams()
    assert params.pan_sign == -1.0
    assert params.tilt_sign == 1.0


def test_the_default_signs_do_not_run_away():
    """The scenario the old default produced: a target 20 deg right, the
    gimbal centres it and reports pan +20 deg.  The bearing must say right."""
    relay = AimRelay(AimParams(track_latency_s=0.0))
    relay.set_intrinsics(TOWER)
    relay.on_gimbal(T0, math.radians(20.0), 0.0)
    relay.on_track(centred_track())
    assert math.degrees(relay.bearing(T0).azimuth) == pytest.approx(-20.0)


def centred_track(stamp=T0, **kw):
    track = {
        "cx": 320.0, "cy": 240.0, "w": 60.0, "h": 60.0,
        "fw": 640, "fh": 480, "label": "bottle", "age": 0.0, "stamp": stamp,
    }
    track.update(kw)
    return track


def spin(relay, wy=0.0, wz=0.0, t0=T0, duration=STEP):
    """Hold a constant body rate over one window."""
    relay.on_gyro(t0, wy, wz)
    relay.on_gyro(t0 + duration, wy, wz)


def gimbal(relay, pan_deg=0.0, tilt_deg=0.0, t0=T0, duration=STEP):
    relay.on_gimbal(t0, 0.0, 0.0)
    relay.on_gimbal(t0 + duration, math.radians(pan_deg), math.radians(tilt_deg))


def target_x_of(relay, now=T0 + STEP):
    return relay.target(now)["target_x"]


def x_of_yaw(yaw_deg):
    """What target_x a given optical yaw comes out as."""
    px = 320.0 + 500.0 * math.tan(math.radians(yaw_deg))
    return 2.0 * px / 640.0 - 1.0


def y_of_pitch(pitch_deg):
    """What target_y a given optical pitch comes out as.

    Not the same arithmetic as x_of_yaw, because the picture is wider than it
    is tall and the normalization is per axis.
    """
    py = 240.0 + 500.0 * math.tan(math.radians(pitch_deg))
    return 2.0 * py / 480.0 - 1.0


# --- the three sign cases ------------------------------------------------

def test_body_yawing_left_pushes_the_target_right():
    """Turn the body left and a fixed target ends up on the right of the picture."""
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay)
    spin(relay, wz=math.radians(10.0) / STEP)
    assert target_x_of(relay) == pytest.approx(x_of_yaw(10.0), abs=1e-6)
    assert target_x_of(relay) > 0.0


def test_panning_the_tower_left_pushes_the_target_right_too():
    """The tower turning left is the camera turning left, exactly as the body is.

    The plan document writes this term with a minus and its own body-bearing
    formula contradicts it.  The identity yaw = pan_sign * pan - azimuth says
    plus, and the counter-rotation case below is what settles it.
    """
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay, pan_deg=10.0)
    spin(relay)
    assert target_x_of(relay) == pytest.approx(x_of_yaw(10.0), abs=1e-6)


def test_a_counter_rotating_tower_holds_the_target_still():
    """Body left ten degrees, tower right ten degrees, nothing moves.

    This is what the gimbal actually does while the robot turns, and it is the
    bench check in one line: turn the body by hand and the tower counter-
    rotates.
    """
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay, pan_deg=-10.0)
    spin(relay, wz=math.radians(10.0) / STEP)
    assert target_x_of(relay) == pytest.approx(0.0, abs=1e-9)


def test_body_and_tower_turning_the_same_way_add_up():
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay, pan_deg=10.0)
    spin(relay, wz=math.radians(10.0) / STEP)
    assert target_x_of(relay) == pytest.approx(x_of_yaw(20.0), abs=1e-6)


def test_a_negative_pan_sign_flips_the_tower_term():
    relay = make_relay(pan_sign=-1.0)
    relay.on_track(centred_track())
    gimbal(relay, pan_deg=10.0)
    spin(relay)
    assert target_x_of(relay) == pytest.approx(x_of_yaw(-10.0), abs=1e-6)


# --- the same three, one axis over ---------------------------------------

def test_a_nose_down_body_lifts_the_target_in_the_picture():
    """Positive wy is nose down in ROS, and a nose-down body raises the target."""
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay)
    spin(relay, wy=math.radians(10.0) / STEP)
    assert relay.target(T0 + STEP)["target_y"] == pytest.approx(y_of_pitch(-10.0), abs=1e-6)


def test_tilting_the_tower_up_pushes_the_target_down():
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay, tilt_deg=10.0)
    spin(relay)
    assert relay.target(T0 + STEP)["target_y"] == pytest.approx(y_of_pitch(10.0), abs=1e-6)


def test_a_counter_tilting_tower_holds_the_target_still():
    """Nose down ten degrees, tilt up ten degrees, nothing moves."""
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay, tilt_deg=10.0)
    spin(relay, wy=math.radians(10.0) / STEP)
    assert relay.target(T0 + STEP)["target_y"] == pytest.approx(0.0, abs=1e-9)


def test_a_negative_tilt_sign_flips_the_tilt_term():
    relay = make_relay(tilt_sign=-1.0)
    relay.on_track(centred_track())
    gimbal(relay, tilt_deg=10.0)
    spin(relay)
    assert relay.target(T0 + STEP)["target_y"] == pytest.approx(y_of_pitch(-10.0), abs=1e-6)


# --- the message the gimbal node reads -----------------------------------

def test_no_track_means_no_target():
    relay = make_relay()
    assert relay.target(T0) is None
    assert relay.bearing(T0) is None
    assert relay.track_age(T0) == float("inf")


def test_no_intrinsics_means_no_target():
    """Without a focal length a pixel carries no angle, so nothing goes out."""
    relay = AimRelay()
    relay.on_track(centred_track())
    assert relay.target(T0) is None
    relay.set_intrinsics(Intrinsics(640, 480, 0.0, 0.0, 320.0, 240.0))
    assert relay.target(T0) is None


def test_intrinsics_arriving_late_still_produce_a_target():
    relay = AimRelay()
    relay.on_track(centred_track())
    relay.set_intrinsics(TOWER)
    assert relay.target(T0)["target_x"] == pytest.approx(0.0)


def test_confidence_falls_to_zero_past_the_coast_window():
    """Zero confidence is how the gimbal node is told to hold its pose."""
    relay = make_relay()
    relay.on_track(centred_track(age=0.0))
    assert relay.target(T0 + 0.5)["confidence"] == 1.0
    assert relay.target(T0 + 0.69)["confidence"] == 1.0
    late = relay.target(T0 + 0.9)
    assert late["confidence"] == 0.0
    # Still published, still detected: the gimbal holds rather than searches.
    assert late["detected"] is True


def test_the_decks_own_age_counts_toward_the_coast():
    relay = make_relay()
    relay.on_track(centred_track(age=0.6))
    assert relay.track_age(T0) == pytest.approx(0.6)
    assert relay.target(T0)["confidence"] == 1.0
    assert relay.target(T0 + 0.2)["confidence"] == 0.0


def test_a_target_predicted_off_the_sensor_is_not_detected():
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay)
    spin(relay, wz=math.radians(60.0) / STEP)
    predicted = relay.target(T0 + STEP)
    assert predicted["detected"] is False
    assert predicted["target_x"] > 1.0


def test_the_range_rides_along_as_distance():
    relay = make_relay()
    relay.on_track(centred_track())
    assert relay.target(T0)["distance_m"] == 0.0
    relay.set_range(1.234)
    assert relay.target(T0)["distance_m"] == pytest.approx(1.234)
    relay.set_range(None)
    assert relay.target(T0)["distance_m"] == 0.0


def test_the_label_and_the_stamp_come_straight_through():
    relay = make_relay()
    relay.on_track(centred_track(label="person"))
    predicted = relay.target(T0 + 0.1)
    assert predicted["class_name"] == "person"
    assert predicted["stamp"] == pytest.approx(T0 + 0.1)


def test_a_new_track_restarts_the_integrals():
    """Nothing the body did before a track may be counted against it."""
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay)
    spin(relay, wz=math.radians(10.0) / STEP)
    assert target_x_of(relay) > 0.1
    relay.on_track(centred_track(stamp=T0 + STEP))
    assert relay.target(T0 + STEP)["target_x"] == pytest.approx(0.0, abs=1e-9)


def test_clearing_the_lock_stops_the_stream():
    relay = make_relay()
    relay.on_track(centred_track())
    assert relay.has_track
    relay.clear()
    assert not relay.has_track
    assert relay.target(T0) is None


def test_a_deck_frame_of_another_size_is_scaled_onto_the_camera():
    """The Deck sees a 320x240 stream of a 640x480 camera."""
    relay = make_relay()
    relay.on_track(centred_track(cx=160.0, cy=120.0, fw=320, fh=240))
    assert relay.target(T0)["target_x"] == pytest.approx(0.0)
    relay.on_track(centred_track(cx=210.0, cy=120.0, fw=320, fh=240, stamp=T0))
    expected = math.degrees(math.atan2(420.0 - 320.0, 500.0))
    assert relay.target(T0)["target_x"] == pytest.approx(x_of_yaw(expected))


def test_the_box_width_is_scaled_onto_the_camera_too():
    relay = make_relay()
    relay.on_track(centred_track(w=30.0, fw=320, fh=240))
    assert relay.box_width_px() == pytest.approx(60.0)


# --- bearing --------------------------------------------------------------

def test_bearing_of_a_centred_target_is_where_the_tower_points():
    relay = make_relay()
    relay.on_gimbal(T0, math.radians(25.0), 0.0)
    relay.on_track(centred_track())
    bearing = relay.bearing(T0)
    assert math.degrees(bearing.azimuth) == pytest.approx(25.0)


def test_a_pan_after_the_track_leaves_the_bearing_alone():
    """The target has not moved, so neither has its bearing.

    The tower panning left puts the target further right in the predicted
    picture, and the two cancel exactly in the bearing.  That cancellation is
    what lets the body loop read a bearing in the middle of a pan.
    """
    relay = make_relay()
    relay.on_track(centred_track())
    gimbal(relay, pan_deg=25.0)
    spin(relay)
    assert math.degrees(relay.bearing(T0 + STEP).azimuth) == pytest.approx(0.0)
    assert relay.target(T0 + STEP)["target_x"] == pytest.approx(
        x_of_yaw(25.0), abs=1e-6)


def test_bearing_ignores_a_pan_that_is_chasing_the_target():
    """Mid-pan the bearing must not move, because the target has not."""
    relay = make_relay()
    relay.on_track(centred_track(cx=420.0))
    relay.on_gimbal(T0, 0.0, 0.0)
    before = relay.bearing(T0)
    relay.on_gimbal(T0 + 0.1, math.radians(-5.0), 0.0)
    after = relay.bearing(T0 + 0.1)
    assert math.degrees(after.azimuth) == pytest.approx(
        math.degrees(before.azimuth), abs=1e-9)


def test_bearing_follows_the_body_turning():
    """Turn the body toward the target and its azimuth runs down to zero.

    The target sits right of the optical axis, so its azimuth is negative and
    the body has to turn right to face it.  That is a negative wz, and the
    bearing is what the controller drives to zero.
    """
    relay = make_relay()
    relay.on_track(centred_track(cx=420.0))
    gimbal(relay)
    before = relay.bearing(T0)
    assert math.degrees(before.azimuth) < 0.0
    spin(relay, wz=before.azimuth / STEP)
    after = relay.bearing(T0 + STEP)
    assert math.degrees(after.azimuth) == pytest.approx(0.0, abs=1e-9)


# --- the two little histories --------------------------------------------

def test_gimbal_history_interpolates():
    history = GimbalHistory()
    history.add(T0, 0.0, 0.0)
    history.add(T0 + 1.0, 1.0, -2.0)
    assert history.at(T0 + 0.25) == pytest.approx((0.25, -0.5))
    # Outside the samples the nearest one is held.
    assert history.at(T0 - 5.0) == pytest.approx((0.0, 0.0))
    assert history.at(T0 + 5.0) == pytest.approx((1.0, -2.0))
    assert history.at(T0) == pytest.approx((0.0, 0.0))


def test_gimbal_history_is_empty_until_it_is_not():
    history = GimbalHistory()
    assert history.at(T0) is None
    assert history.latest() is None


def test_gimbal_history_forgets_and_ignores_the_past():
    history = GimbalHistory(history_s=1.0)
    history.add(T0, 0.0, 0.0)
    history.add(T0 + 2.0, 1.0, 1.0)
    assert history.at(T0) == pytest.approx((1.0, 1.0))
    # A sample from before the last one would break the interpolation.
    history.add(T0 + 1.0, 9.0, 9.0)
    assert history.latest()[0] == T0 + 2.0


def test_the_gimbal_is_sampled_at_the_tracks_own_time():
    """A pan that happened after the track must not be charged to it."""
    relay = make_relay()
    relay.on_gimbal(T0 - 0.1, 0.0, 0.0)
    relay.on_gimbal(T0 + 0.1, math.radians(20.0), 0.0)
    relay.on_track(centred_track())
    # The track sits halfway between the samples, so the anchor pan is 10 deg
    # and only the remaining 10 deg counts.
    assert relay.target(T0 + 0.1)["target_x"] == pytest.approx(
        x_of_yaw(10.0), abs=1e-6)


def test_gyro_integral_over_a_window():
    gyro = GyroIntegral()
    gyro.add(T0, 1.0, 2.0)
    gyro.add(T0 + 1.0, 0.0, 0.0)
    dtheta, dpsi = gyro.integrate(T0, T0 + 1.0)
    assert (dtheta, dpsi) == pytest.approx((1.0, 2.0))
    # Half the window is half the angle.
    assert gyro.integrate(T0 + 0.5, T0 + 1.0) == pytest.approx((0.5, 1.0))
    # A window that has not started yet is nothing at all.
    assert gyro.integrate(T0 + 1.0, T0 + 1.0) == pytest.approx((0.0, 0.0))
    assert gyro.integrate(T0 + 1.0, T0) == pytest.approx((0.0, 0.0))


def test_gyro_integral_with_no_samples():
    assert GyroIntegral().integrate(T0, T0 + 1.0) == (0.0, 0.0)


def test_gyro_holds_the_last_sample_to_the_end_of_the_window():
    gyro = GyroIntegral()
    gyro.add(T0, 0.0, 1.0)
    assert gyro.integrate(T0, T0 + 0.5) == pytest.approx((0.0, 0.5))


# --- when the picture was taken ------------------------------------------

def test_a_coasting_track_keeps_the_anchor():
    """The page resends the same box with a growing age while it coasts.

    That message carries no new picture.  The body turned 12 degrees since
    the real match, and the prediction has to say so, instead of restarting
    from a time the box does not belong to.
    """
    relay = make_relay()
    relay.on_track(centred_track(age=0.0))
    gimbal(relay, duration=0.3)
    spin(relay, wz=math.radians(12.0) / 0.3, duration=0.3)
    relay.on_track(centred_track(stamp=T0 + 0.3, age=0.3))
    assert relay.target(T0 + 0.3)["target_x"] == pytest.approx(x_of_yaw(12.0), abs=1e-6)
    # The Deck's age is still the age of that one detection.
    assert relay.track_age(T0 + 0.3) == pytest.approx(0.3)


def test_a_fresh_match_on_the_same_pixels_is_a_new_anchor():
    """The same box with a smaller age is a new detection, not a coast."""
    relay = make_relay()
    relay.on_track(centred_track(age=0.3))
    gimbal(relay)
    spin(relay, wz=math.radians(10.0) / STEP)
    assert target_x_of(relay) > 0.1
    relay.on_track(centred_track(stamp=T0 + STEP, age=0.0))
    assert relay.target(T0 + STEP)["target_x"] == pytest.approx(0.0, abs=1e-9)


def test_a_moved_box_is_a_new_anchor_even_with_a_larger_age():
    relay = make_relay()
    relay.on_track(centred_track(age=0.0))
    gimbal(relay)
    spin(relay, wz=math.radians(10.0) / STEP)
    relay.on_track(centred_track(cx=420.0, stamp=T0 + STEP, age=0.1))
    # A new anchor at the new pixel, matched 0.1 s before its stamp, so the
    # last 0.1 s of the 50 deg/s spin (5 deg) is charged to it.
    expected = math.degrees(math.atan2(100.0, 500.0)) + 5.0
    assert relay.target(T0 + STEP)["target_x"] == pytest.approx(x_of_yaw(expected), abs=1e-6)


def test_the_decks_age_moves_the_anchor_back_in_time():
    """A track that arrives with age 0.2 describes a picture 0.2 s old."""
    relay = make_relay()
    gimbal(relay, duration=0.2)
    spin(relay, wz=math.radians(10.0) / 0.2, duration=0.2)
    # Stamped at T0 + 0.2, matched 0.2 s before: the turn is charged to it.
    relay.on_track(centred_track(stamp=T0 + 0.2, age=0.2))
    assert relay.target(T0 + 0.2)["target_x"] == pytest.approx(x_of_yaw(10.0), abs=1e-6)


def test_a_body_turn_inside_the_round_trip_is_charged_to_the_track():
    """The picture is older than its stamp by the Deck round trip.

    With the body turning left through the whole round trip, the target is
    already to the right of where the stamp-time picture says.
    """
    relay = make_relay(track_latency_s=0.2)
    relay.on_gimbal(T0 - 0.2, 0.0, 0.0)
    relay.on_gimbal(T0, 0.0, 0.0)
    relay.on_gyro(T0 - 0.2, 0.0, math.radians(10.0) / 0.2)
    relay.on_gyro(T0, 0.0, 0.0)
    relay.on_track(centred_track(stamp=T0))
    assert relay.target(T0 + 0.05)["target_x"] == pytest.approx(x_of_yaw(10.0), abs=1e-6)
    # The coast window is the page's, not the round trip's.
    assert relay.track_age(T0) == pytest.approx(0.0)


def test_a_counter_rotation_inside_the_round_trip_cancels():
    """Body left, tower right, both inside the round trip: the pixel holds."""
    relay = make_relay(track_latency_s=0.2)
    relay.on_gimbal(T0 - 0.2, 0.0, 0.0)
    relay.on_gimbal(T0, math.radians(-10.0), 0.0)
    relay.on_gyro(T0 - 0.2, 0.0, math.radians(10.0) / 0.2)
    relay.on_gyro(T0, 0.0, 0.0)
    relay.on_track(centred_track(stamp=T0))
    assert relay.target(T0 + 0.05)["target_x"] == pytest.approx(0.0, abs=1e-9)


def test_a_pan_inside_the_round_trip_is_charged_to_the_track():
    relay = make_relay(track_latency_s=0.2)
    relay.on_gimbal(T0 - 0.2, 0.0, 0.0)
    relay.on_gimbal(T0, math.radians(10.0), 0.0)
    relay.on_track(centred_track(stamp=T0))
    assert relay.target(T0 + 0.05)["target_x"] == pytest.approx(x_of_yaw(10.0), abs=1e-6)


# --- a gimbal that has gone quiet ----------------------------------------

def test_a_stale_gimbal_sample_is_not_the_current_angle():
    """Past gimbal_max_age_s the last sample is a guess, and the bearing
    falls back to the pan the track was anchored on."""
    relay = make_relay(gimbal_max_age_s=0.5)
    relay.on_gimbal(T0, math.radians(25.0), 0.0)
    relay.on_track(centred_track())
    assert relay.gimbal_stale(T0 + 0.4) is False
    assert relay.gimbal_stale(T0 + 0.6) is True
    assert relay.gimbal_age(T0 + 0.6) == pytest.approx(0.6)
    assert math.degrees(relay.bearing(T0 + 0.6).azimuth) == pytest.approx(25.0)
    assert relay.target(T0 + 0.6)["target_x"] == pytest.approx(0.0, abs=1e-9)
    # A fresh sample brings the gimbal's own motion back into the prediction.
    relay.on_gimbal(T0 + 0.6, math.radians(35.0), 0.0)
    assert relay.gimbal_stale(T0 + 0.6) is False
    assert relay.target(T0 + 0.6)["target_x"] == pytest.approx(x_of_yaw(10.0), abs=1e-6)
    assert math.degrees(relay.bearing(T0 + 0.6).azimuth) == pytest.approx(25.0)


def test_gimbal_age_before_any_sample():
    relay = make_relay()
    assert relay.gimbal_age(T0) is None
    assert relay.gimbal_stale(T0) is False
