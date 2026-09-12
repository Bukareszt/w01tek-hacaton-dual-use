"""Pinhole geometry, both directions, and the body bearing it implies."""

import math

import numpy as np
import pytest

from wojtek_follow.core.geometry import (
    Bearing,
    Intrinsics,
    body_bearing,
    body_direction_to_optical,
    inside_image,
    normalized_to_pixel,
    optical_from_bearing,
    optical_points_to_body,
    optical_to_pixel,
    pixel_to_normalized,
    pixel_to_optical,
    scale_intrinsics,
)

TOWER = Intrinsics(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0)


def test_uncalibrated_intrinsics_are_not_usable():
    assert not Intrinsics(640, 480, 0.0, 0.0, 320.0, 240.0).valid
    assert TOWER.valid


def test_intrinsics_from_camera_info():
    class Info:
        width = 640
        height = 480
        k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]

    assert Intrinsics.from_camera_info(Info()) == TOWER


def test_pixel_to_optical_signs():
    """Yaw is positive to the right, pitch is positive downward."""
    right, _ = pixel_to_optical(420.0, 240.0, TOWER)
    left, _ = pixel_to_optical(220.0, 240.0, TOWER)
    _, below = pixel_to_optical(320.0, 340.0, TOWER)
    _, above = pixel_to_optical(320.0, 140.0, TOWER)
    assert right > 0.0
    assert left < 0.0
    assert below > 0.0
    assert above < 0.0


def test_pixel_optical_round_trip():
    for px, py in ((320.0, 240.0), (10.0, 470.0), (630.0, 5.0)):
        yaw, pitch = pixel_to_optical(px, py, TOWER)
        back = optical_to_pixel(yaw, pitch, TOWER)
        assert back == pytest.approx((px, py), abs=1e-9)


def test_optical_angle_matches_the_arctangent():
    yaw, _ = pixel_to_optical(320.0 + 500.0 * math.tan(math.radians(10.0)), 240.0, TOWER)
    assert math.degrees(yaw) == pytest.approx(10.0)


def test_normalized_round_trip_matches_the_message_definition():
    """-1 is the left or top edge, +1 the right or bottom, 0 the centre."""
    assert pixel_to_normalized(0.0, 0.0, 640, 480) == pytest.approx((-1.0, -1.0))
    assert pixel_to_normalized(320.0, 240.0, 640, 480) == pytest.approx((0.0, 0.0))
    assert pixel_to_normalized(640.0, 480.0, 640, 480) == pytest.approx((1.0, 1.0))
    # The gimbal node's own conversion, from kinematics.py.
    assert normalized_to_pixel(0.5, -0.5, 640, 480) == pytest.approx((480.0, 120.0))
    for px, py in ((0.0, 0.0), (123.0, 45.0), (640.0, 480.0)):
        tx, ty = pixel_to_normalized(px, py, 640, 480)
        assert normalized_to_pixel(tx, ty, 640, 480) == pytest.approx((px, py))


def test_scaled_intrinsics_give_the_same_angle():
    """A Deck frame half the size must not halve the bearing."""
    deck = scale_intrinsics(TOWER, 320, 240)
    assert deck.fx == pytest.approx(250.0)
    assert deck.cx == pytest.approx(160.0)
    full = pixel_to_optical(420.0, 300.0, TOWER)
    half = pixel_to_optical(210.0, 150.0, deck)
    assert half == pytest.approx(full)


def test_scaling_to_the_same_size_is_the_same_object():
    assert scale_intrinsics(TOWER, 640, 480) is TOWER


def test_scaling_rejects_nonsense():
    with pytest.raises(ValueError):
        scale_intrinsics(TOWER, 0, 480)
    with pytest.raises(ValueError):
        scale_intrinsics(Intrinsics(640, 480, 0.0, 0.0, 320.0, 240.0), 320, 240)


def test_inside_image():
    assert inside_image(0.0, 0.0, TOWER)
    assert inside_image(639.0, 479.0, TOWER)
    assert not inside_image(-1.0, 100.0, TOWER)
    assert not inside_image(100.0, 480.0, TOWER)


def test_body_bearing_from_a_centred_target():
    """A centred target sits exactly where the tower points."""
    bearing = body_bearing(pan=math.radians(20.0), tilt=math.radians(5.0),
                           yaw=0.0, pitch=0.0)
    assert math.degrees(bearing.azimuth) == pytest.approx(20.0)
    assert math.degrees(bearing.elevation) == pytest.approx(5.0)


def test_body_bearing_puts_a_right_of_axis_target_clockwise():
    """Right of the optical axis is a negative azimuth, because left is positive."""
    bearing = body_bearing(pan=0.0, tilt=0.0, yaw=math.radians(10.0), pitch=0.0)
    assert math.degrees(bearing.azimuth) == pytest.approx(-10.0)
    # Below the axis is a negative elevation, because up is positive.
    bearing = body_bearing(pan=0.0, tilt=0.0, yaw=0.0, pitch=math.radians(10.0))
    assert math.degrees(bearing.elevation) == pytest.approx(-10.0)


def test_body_bearing_signs_flip_with_the_mounting():
    bearing = body_bearing(pan=math.radians(20.0), tilt=math.radians(5.0),
                           yaw=0.0, pitch=0.0, pan_sign=-1.0, tilt_sign=-1.0)
    assert math.degrees(bearing.azimuth) == pytest.approx(-20.0)
    assert math.degrees(bearing.elevation) == pytest.approx(-5.0)


def test_body_bearing_round_trip():
    pan, tilt = math.radians(15.0), math.radians(-7.0)
    yaw, pitch = math.radians(-3.0), math.radians(4.0)
    bearing = body_bearing(pan, tilt, yaw, pitch, pan_sign=-1.0, tilt_sign=1.0)
    back = optical_from_bearing(bearing, pan, tilt, pan_sign=-1.0, tilt_sign=1.0)
    assert back == pytest.approx((yaw, pitch))


def test_body_direction_to_optical_axes():
    """Straight ahead is straight down the optical axis."""
    assert body_direction_to_optical(0.0, 0.0) == pytest.approx((0.0, 0.0, 1.0))
    # To the left of the body is to the left of the picture, which is -x.
    x, y, z = body_direction_to_optical(math.radians(20.0), 0.0)
    assert x < 0.0 and z > 0.0
    # Above the body is above the picture, which is -y.
    x, y, z = body_direction_to_optical(0.0, math.radians(20.0))
    assert y < 0.0 and z > 0.0
    # Behind the robot is behind the camera.
    assert body_direction_to_optical(math.radians(170.0), 0.0)[2] < 0.0


def test_a_camera_pitched_down_sees_a_level_target_high():
    x, y, z = body_direction_to_optical(0.0, 0.0, camera_pitch=math.radians(30.0))
    assert y == pytest.approx(-math.sin(math.radians(30.0)))
    assert z == pytest.approx(math.cos(math.radians(30.0)))
    assert x == pytest.approx(0.0)


def test_optical_points_to_body_round_trip():
    """Back-projection undoes the projection, with the camera's height added."""
    bearing = Bearing(azimuth=math.radians(12.0), elevation=math.radians(-5.0))
    pitch = math.radians(10.0)
    x, y, z = body_direction_to_optical(bearing.azimuth, bearing.elevation, pitch)
    # The direction is a unit vector, so scaling it by two puts the point two
    # metres along the ray.
    scale = 2.0
    bx, by, bz = optical_points_to_body(
        np.array([x * scale]), np.array([y * scale]), np.array([z * scale]),
        camera_pitch=pitch, camera_height=0.2)
    assert float(np.arctan2(by[0], bx[0])) == pytest.approx(bearing.azimuth)
    assert float(bz[0] - 0.2) == pytest.approx(2.0 * math.sin(bearing.elevation))


def test_optical_points_to_body_with_no_pitch():
    bx, by, bz = optical_points_to_body(
        np.array([1.0]), np.array([0.5]), np.array([3.0]), camera_height=0.2)
    assert bx[0] == pytest.approx(3.0)
    assert by[0] == pytest.approx(-1.0)
    assert bz[0] == pytest.approx(-0.3)
