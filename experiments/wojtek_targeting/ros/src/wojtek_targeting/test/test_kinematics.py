"""Kinematics tests: pure maths, no stubs needed."""

import math

import pytest
from wojtek_targeting.kinematics import (
    Intrinsics,
    clamp,
    rate_limit,
    target_to_optical_angles,
)

CALIBRATED = Intrinsics(width=640, height=480, fx=600.0, fy=600.0, cx=320.0, cy=240.0)


def test_uncalibrated_intrinsics_are_not_valid():
    """fx=0 means no angle can be derived from a pixel offset at all."""
    assert not Intrinsics(width=640, height=480, fx=0.0, fy=0.0, cx=320.0, cy=240.0).valid
    assert not Intrinsics(width=0, height=0, fx=600.0, fy=600.0, cx=0.0, cy=0.0).valid
    assert CALIBRATED.valid


def test_centred_target_needs_no_correction():
    yaw, pitch = target_to_optical_angles(0.0, 0.0, CALIBRATED)
    assert yaw == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_target_right_of_centre_gives_positive_yaw():
    yaw, _ = target_to_optical_angles(0.5, 0.0, CALIBRATED)
    # Half way to the right edge is 160 px off axis at fx=600.
    assert yaw == pytest.approx(math.degrees(math.atan2(160.0, 600.0)), abs=1e-6)
    assert yaw > 0.0


def test_target_below_centre_gives_positive_pitch():
    """Optical convention: image y grows downward, so down is positive."""
    _, pitch = target_to_optical_angles(0.0, 0.5, CALIBRATED)
    assert pitch == pytest.approx(math.degrees(math.atan2(120.0, 600.0)), abs=1e-6)


def test_offset_principal_point_shifts_the_zero():
    shifted = Intrinsics(width=640, height=480, fx=600.0, fy=600.0, cx=300.0, cy=240.0)
    yaw, _ = target_to_optical_angles(0.0, 0.0, shifted)
    # Image centre is 20 px right of the principal point, so it is off axis.
    assert yaw == pytest.approx(math.degrees(math.atan2(20.0, 600.0)), abs=1e-6)


def test_uncalibrated_camera_raises_rather_than_guessing():
    blind = Intrinsics(width=640, height=480, fx=0.0, fy=0.0, cx=320.0, cy=240.0)
    with pytest.raises(ValueError):
        target_to_optical_angles(0.5, 0.0, blind)


def test_rate_limit_caps_a_large_step():
    assert rate_limit(100.0, 0.0, 45.0, 0.025) == pytest.approx(45.0 * 0.025)
    assert rate_limit(-100.0, 0.0, 45.0, 0.025) == pytest.approx(-45.0 * 0.025)


def test_rate_limit_passes_a_small_step_through():
    assert rate_limit(0.5, 0.0, 45.0, 0.025) == pytest.approx(0.5)


def test_rate_limit_holds_when_dt_is_zero():
    assert rate_limit(100.0, 7.0, 45.0, 0.0) == 7.0


def test_clamp():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5
