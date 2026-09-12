"""The three-band obstacle gate, on synthetic depth images."""

import numpy as np
import pytest

from wojtek_follow.core.gate import GateParams, body_points, obstacle_bands
from wojtek_follow.core.geometry import Intrinsics

DEPTH = Intrinsics(width=424, height=240, fx=210.0, fy=210.0, cx=212.0, cy=120.0)
PARAMS = GateParams()

# Rows 100 to 140 at a metre are between 0.10 m and 0.30 m above the floor with
# the camera at 0.20 m, which is the band the gate keeps.
ROWS = slice(100, 141)
# Columns are azimuth.  Left of the principal point is the robot's left.
LEFT = slice(100, 131)
CENTRE = slice(200, 225)
RIGHT = slice(300, 331)


def empty():
    return np.zeros((240, 424), dtype=np.uint16)


def blob(rows, cols, distance_mm=1000):
    image = empty()
    image[rows, cols] = distance_mm
    return image


def test_nothing_in_an_empty_image():
    assert obstacle_bands(empty(), DEPTH, PARAMS) == {
        "left": None, "centre": None, "right": None}


def test_no_image_and_no_intrinsics():
    assert obstacle_bands(None, DEPTH, PARAMS)["centre"] is None
    assert obstacle_bands(empty(), None, PARAMS)["centre"] is None
    assert body_points(None, DEPTH, PARAMS)[0].size == 0


def test_a_box_on_the_left_lands_in_the_left_band():
    bands = obstacle_bands(blob(ROWS, LEFT), DEPTH, PARAMS)
    assert bands["left"] == pytest.approx(1.1, abs=0.1)
    assert bands["centre"] is None
    assert bands["right"] is None


def test_a_box_in_front_lands_in_the_centre_band():
    bands = obstacle_bands(blob(ROWS, CENTRE), DEPTH, PARAMS)
    assert bands["centre"] == pytest.approx(1.0, abs=0.1)
    assert bands["left"] is None
    assert bands["right"] is None


def test_a_box_on_the_right_lands_in_the_right_band():
    bands = obstacle_bands(blob(ROWS, RIGHT), DEPTH, PARAMS)
    assert bands["right"] == pytest.approx(1.1, abs=0.1)
    assert bands["left"] is None
    assert bands["centre"] is None


def test_a_band_reports_its_nearest_point():
    image = blob(ROWS, CENTRE, 1500)
    image[ROWS, 210:214] = 700
    bands = obstacle_bands(image, DEPTH, PARAMS)
    assert bands["centre"] == pytest.approx(0.7, abs=0.05)


def test_the_floor_is_not_an_obstacle():
    """Rows below the horizon at a metre are floor, and the robot walks on it."""
    bands = obstacle_bands(blob(slice(170, 185), CENTRE), DEPTH, PARAMS)
    assert bands == {"left": None, "centre": None, "right": None}


def test_things_overhead_are_not_obstacles():
    bands = obstacle_bands(blob(slice(40, 60), CENTRE), DEPTH, PARAMS)
    assert bands == {"left": None, "centre": None, "right": None}


def test_the_blind_zone_is_not_an_obstacle():
    bands = obstacle_bands(blob(slice(116, 125), CENTRE, 200), DEPTH, PARAMS)
    assert bands == {"left": None, "centre": None, "right": None}


def test_the_blind_zone_is_measured_from_the_sensor():
    """A level return at 0.25 m is noise, whatever the camera's height.

    Measured from a point on the floor below the camera the same return would
    read 0.32 m and slip past the blind-zone cut, and the robot would go
    BLOCKED in front of nothing.
    """
    image = blob(slice(118, 123), CENTRE, 250)
    assert obstacle_bands(image, DEPTH, PARAMS) == {
        "left": None, "centre": None, "right": None}
    high = GateParams(depth_camera_height_m=0.30)
    assert obstacle_bands(image, DEPTH, high)["centre"] is None


def test_the_range_is_the_ray_from_the_sensor():
    """The same quantity the rangefinder reports for the target."""
    _, ranges = body_points(blob(slice(118, 123), slice(210, 215), 1000), DEPTH, PARAMS)
    assert ranges.size > 0
    assert ranges == pytest.approx(1.0, abs=0.001)


def test_far_things_are_not_this_ticks_problem():
    bands = obstacle_bands(blob(slice(116, 125), CENTRE, 4000), DEPTH, PARAMS)
    assert bands == {"left": None, "centre": None, "right": None}


def test_the_camera_pitch_moves_the_floor():
    """A pitch that is wrong by twenty degrees makes the gate see nothing at all."""
    image = blob(ROWS, CENTRE)
    assert obstacle_bands(image, DEPTH, PARAMS)["centre"] is not None
    pitched = GateParams(depth_camera_pitch_deg=20.0)
    assert obstacle_bands(image, DEPTH, pitched)["centre"] is None


def test_the_camera_height_moves_the_floor_too():
    """The same points are an obstacle or the floor depending on this number."""
    image = blob(slice(116, 125), CENTRE)
    assert obstacle_bands(image, DEPTH, PARAMS)["centre"] is not None
    low = GateParams(depth_camera_height_m=0.0)
    assert obstacle_bands(image, DEPTH, low)["centre"] is None


def test_the_target_is_not_an_obstacle():
    """Without the cone the robot refuses to approach the thing it follows."""
    image = blob(ROWS, CENTRE)
    bands = obstacle_bands(image, DEPTH, PARAMS, target_azimuth=0.0, target_range=1.0)
    assert bands["centre"] is None


def test_the_cone_does_not_reach_past_the_target():
    """Something standing behind the target is still an obstacle."""
    image = blob(ROWS, CENTRE, 1000)
    bands = obstacle_bands(image, DEPTH, PARAMS, target_azimuth=0.0, target_range=0.5)
    assert bands["centre"] == pytest.approx(1.0, abs=0.1)


def test_the_cone_is_only_around_the_target():
    image = blob(ROWS, CENTRE)
    bands = obstacle_bands(
        image, DEPTH, PARAMS, target_azimuth=np.radians(30.0), target_range=1.0)
    assert bands["centre"] is not None


def test_the_cone_can_empty_the_image():
    image = blob(ROWS, CENTRE)
    bands = obstacle_bands(image, DEPTH, PARAMS, target_azimuth=0.0, target_range=2.0)
    assert bands == {"left": None, "centre": None, "right": None}


def test_a_box_on_the_left_and_one_in_front():
    image = blob(ROWS, LEFT, 900)
    image[ROWS, CENTRE] = 1400
    bands = obstacle_bands(image, DEPTH, PARAMS)
    assert bands["left"] == pytest.approx(1.0, abs=0.1)
    assert bands["centre"] == pytest.approx(1.4, abs=0.1)


def test_a_person_at_the_hold_distance_is_cut_out_whole():
    """A person's shoulders at one metre span about 16 degrees, wider than the
    10 degree cone floor.  The cone grows with cone_half_width_m over the
    range, so the target does not spill into the centre band and flip HOLD
    into BLOCKED."""
    image = blob(ROWS, slice(149, 276))       # 0.30 m each side at 1 m
    bands = obstacle_bands(image, DEPTH, PARAMS, target_azimuth=0.0, target_range=1.0)
    assert bands == {"left": None, "centre": None, "right": None}


def test_the_cone_narrows_with_range():
    """At two and a half metres the same width is 8 degrees, so the floor of
    10 holds and the blob's outer columns, at 10 to 17 degrees, stay in the
    centre band.  Three metres would empty the image by the range ceiling
    instead, which is not the claim here."""
    image = blob(ROWS, slice(149, 276), 2500)
    bands = obstacle_bands(image, DEPTH, PARAMS, target_azimuth=0.0, target_range=2.5)
    assert bands["centre"] is not None


def test_decimation_keeps_the_answer():
    """Every fourth pixel is enough; the nearest point survives it."""
    image = blob(ROWS, CENTRE)
    fine = GateParams(decimate=1)
    coarse = obstacle_bands(image, DEPTH, PARAMS)["centre"]
    assert coarse == pytest.approx(obstacle_bands(image, DEPTH, fine)["centre"], abs=0.05)
