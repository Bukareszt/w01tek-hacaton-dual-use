"""Range to the target, from the depth image and from the box width."""

import math

import numpy as np
import pytest

from wojtek_follow.core.geometry import Bearing, Intrinsics
from wojtek_follow.core.rangefinder import (
    DEFAULT_WIDTH_M,
    RangeParams,
    estimate_range,
    project_bearing,
    range_from_box,
    range_from_depth,
)

# The D435's depth stream at 424x240, near enough for a test.
DEPTH = Intrinsics(width=424, height=240, fx=210.0, fy=210.0, cx=212.0, cy=120.0)
TOWER_FX = 500.0
AHEAD = Bearing(azimuth=0.0, elevation=0.0)
PARAMS = RangeParams()


def wall(distance_mm=1200):
    """A flat wall square in front of the camera, at a constant depth."""
    return np.full((240, 424), distance_mm, dtype=np.uint16)


def with_blob(image, distance_mm, row=120, col=212, half=20):
    image = image.copy()
    image[row - half:row + half, col - half:col + half] = distance_mm
    return image


def test_a_target_in_front_of_a_wall():
    """The near thing in the window is the target, which is why the quantile is low."""
    depth = with_blob(wall(1200), 800)
    assert range_from_depth(depth, DEPTH, AHEAD, PARAMS) == pytest.approx(0.8, abs=0.01)


def test_the_wall_itself_when_there_is_no_target_blob():
    assert range_from_depth(wall(1200), DEPTH, AHEAD, PARAMS) == pytest.approx(1.2, abs=0.01)


def test_an_off_axis_bearing_is_a_longer_ray_than_the_depth():
    """Depth is the z coordinate, and the controller wants the distance."""
    bearing = Bearing(azimuth=math.radians(30.0), elevation=0.0)
    measured = range_from_depth(wall(1200), DEPTH, bearing, PARAMS)
    assert measured == pytest.approx(1.2 / math.cos(math.radians(30.0)), abs=0.02)


def test_a_bearing_off_to_the_left_projects_left_of_centre():
    bearing = Bearing(azimuth=math.radians(20.0), elevation=0.0)
    px, py, forward = project_bearing(bearing, DEPTH, 0.0)
    assert px < DEPTH.cx
    assert py == pytest.approx(DEPTH.cy)
    assert forward == pytest.approx(math.cos(math.radians(20.0)))


def test_a_bearing_behind_the_camera_has_no_pixel():
    assert project_bearing(Bearing(math.radians(170.0), 0.0), DEPTH, 0.0) is None


def test_nothing_beyond_the_depth_cameras_view():
    """Past the limit the D435 is not looking, so there is no depth to be had."""
    assert range_from_depth(wall(1200), DEPTH, Bearing(math.radians(40.0), 0.0), PARAMS) is None
    assert range_from_depth(wall(1200), DEPTH, Bearing(math.radians(-40.0), 0.0), PARAMS) is None


def test_a_bearing_off_the_top_of_the_image_has_no_depth():
    assert range_from_depth(wall(1200), DEPTH, Bearing(0.0, math.radians(45.0)), PARAMS) is None


def test_zeros_are_not_readings():
    """A zero is the absence of a measurement, not a surface at the lens."""
    assert range_from_depth(np.zeros((240, 424), dtype=np.uint16), DEPTH, AHEAD, PARAMS) is None


def test_readings_outside_the_usable_band_are_dropped():
    assert range_from_depth(wall(5000), DEPTH, AHEAD, PARAMS) is None
    assert range_from_depth(wall(150), DEPTH, AHEAD, PARAMS) is None


def test_no_depth_image_and_no_intrinsics():
    assert range_from_depth(None, DEPTH, AHEAD, PARAMS) is None
    assert range_from_depth(wall(), None, AHEAD, PARAMS) is None
    assert range_from_depth(
        wall(), Intrinsics(424, 240, 0.0, 0.0, 212.0, 120.0), AHEAD, PARAMS) is None


def test_the_window_follows_the_bearing():
    """A blob off to one side is found by aiming at it and not otherwise."""
    depth = with_blob(wall(2000), 900, col=100)
    off_axis = Bearing(azimuth=math.atan2(212.0 - 100.0, 210.0), elevation=0.0)
    near = range_from_depth(depth, DEPTH, off_axis, PARAMS)
    assert near is not None and near < 1.1
    assert range_from_depth(depth, DEPTH, AHEAD, PARAMS) == pytest.approx(2.0, abs=0.02)


def test_the_window_is_clipped_at_the_edge_of_the_image():
    edge = Bearing(azimuth=math.atan2(212.0, 210.0), elevation=0.0)
    assert abs(math.degrees(edge.azimuth)) < PARAMS.max_azimuth_deg + 20.0
    wide = RangeParams(max_azimuth_deg=60.0)
    assert range_from_depth(wall(1000), DEPTH, edge, wide) is not None


def test_range_from_a_box_width():
    """A person 150 px wide in a 500 px focal length is a metre and a half off."""
    assert range_from_box(150.0, "person", TOWER_FX, PARAMS) == pytest.approx(1.5)
    assert range_from_box(150.0, "bottle", TOWER_FX, PARAMS) == pytest.approx(
        500.0 * 0.07 / 150.0)


def test_an_unknown_label_gets_the_default_width():
    assert range_from_box(150.0, "toaster", TOWER_FX, PARAMS) == pytest.approx(
        TOWER_FX * DEFAULT_WIDTH_M / 150.0)
    assert range_from_box(150.0, None, TOWER_FX, PARAMS) == pytest.approx(
        TOWER_FX * DEFAULT_WIDTH_M / 150.0)


def test_a_box_with_no_width_and_a_camera_with_no_focal_length():
    assert range_from_box(0.0, "person", TOWER_FX, PARAMS) is None
    assert range_from_box(None, "person", TOWER_FX, PARAMS) is None
    assert range_from_box(150.0, "person", 0.0, PARAMS) is None


def test_depth_wins_when_there_is_depth():
    depth = with_blob(wall(1200), 800)
    assert estimate_range(AHEAD, depth, DEPTH, 150.0, "person", TOWER_FX, PARAMS) == (
        pytest.approx(0.8, abs=0.01), "depth")


def test_the_box_is_the_fallback_and_says_so():
    """Behind the robot there is no depth, and the source is what protects the walk."""
    behind = Bearing(azimuth=math.radians(120.0), elevation=0.0)
    range_m, source = estimate_range(
        behind, wall(1200), DEPTH, 150.0, "person", TOWER_FX, PARAMS)
    assert source == "bbox"
    assert range_m == pytest.approx(1.5)


def test_no_range_at_all():
    assert estimate_range(None, None, None, None, "person", TOWER_FX, PARAMS) == (None, "none")
    assert estimate_range(AHEAD, None, None, 0.0, "person", 0.0, PARAMS) == (None, "none")
