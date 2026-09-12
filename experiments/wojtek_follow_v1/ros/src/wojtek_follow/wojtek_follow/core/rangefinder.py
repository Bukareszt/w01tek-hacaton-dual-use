"""How far away the target is.

The tower camera gives a bearing and no range.  The D435 on the front of the
body gives a range wherever it is looking, which is forward and nowhere else.
The two are joined here: the bearing becomes a ray, the ray becomes a pixel in
the depth image, and a window around that pixel becomes a distance.

When the bearing is outside the depth camera's view there is still a number to
be had from the size of the box, given a guess at how wide the thing really is.
That number is good to tens of percent.  It is enough to decide which way to
turn and it is never allowed to drive the robot forward, which is why the
source comes back with the range.

The depth image is `16UC1` millimetres with zero for no reading.  Zeros are
dropped rather than averaged in, because a zero is the absence of a
measurement and not a surface at the camera.  The window's 20th percentile is
taken instead of its median: the target is the near thing in the window and the
rest of the window is whatever is behind it.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import body_direction_to_optical

# How wide these things are, in metres, across the picture.  A COCO label is
# all the Deck knows about its target, so this table is the only thing standing
# between a box width and a range.  Every number is a guess at a typical object
# and the error goes straight into the range.
LABEL_WIDTHS_M = {
    "person": 0.45,
    "bottle": 0.07,
    "cup": 0.08,
    "sports ball": 0.22,
    "chair": 0.45,
    "backpack": 0.30,
}
DEFAULT_WIDTH_M = 0.3


@dataclass(frozen=True)
class RangeParams:
    """Where the depth camera is, and what counts as a usable reading."""

    # The D435's optical axis relative to the body x axis, positive downward.
    depth_camera_pitch_deg: float = 0.0
    # The side of the square window sampled around the projected pixel.
    window_px: int = 24
    # The quantile taken inside the window, in percent.
    percentile: float = 20.0
    # Readings outside this band are not the target.  The D435 sees nothing
    # closer than about 0.3 m at all.
    min_range_m: float = 0.3
    max_range_m: float = 4.0
    # Beyond this bearing the target is outside the depth camera's view and a
    # projected pixel would be a lie.
    max_azimuth_deg: float = 35.0
    widths_m: dict = field(default_factory=lambda: dict(LABEL_WIDTHS_M))


def project_bearing(bearing, intrinsics, camera_pitch):
    """A body-frame bearing as a pixel in a camera, or None behind the camera."""
    x, y, z = body_direction_to_optical(
        bearing.azimuth, bearing.elevation, camera_pitch)
    if z <= 1e-6:
        return None
    px = intrinsics.cx + intrinsics.fx * x / z
    py = intrinsics.cy + intrinsics.fy * y / z
    return px, py, z


def range_from_depth(depth_mm, intrinsics, bearing, params):
    """The range along the bearing, in metres, or None.

    The depth image stores the z coordinate in the optical frame, not the
    distance along the ray.  Dividing by the ray's own z component turns one
    into the other.  At the 35 degrees this function still accepts that is a
    22 percent correction, which is far too much to leave out at a hold
    distance of a metre.
    """
    if depth_mm is None or intrinsics is None or not intrinsics.valid:
        return None
    if abs(bearing.azimuth) >= math.radians(params.max_azimuth_deg):
        return None
    projected = project_bearing(
        bearing, intrinsics, math.radians(params.depth_camera_pitch_deg))
    if projected is None:
        return None
    px, py, forward = projected
    height, width = depth_mm.shape[:2]
    if not (0 <= px <= width - 1 and 0 <= py <= height - 1):
        return None
    half = max(1, int(params.window_px) // 2)
    col = int(round(px))
    row = int(round(py))
    window = depth_mm[
        max(0, row - half):min(height, row + half + 1),
        max(0, col - half):min(width, col + half + 1),
    ]
    values = window[window > 0]
    if values.size == 0:
        return None
    depth_m = float(np.percentile(values.astype(np.float64), params.percentile)) / 1000.0
    range_m = depth_m / forward
    if params.min_range_m <= range_m <= params.max_range_m:
        return range_m
    return None


def range_from_box(box_width_px, label, fx_tower, params):
    """The range a box of that width implies, given a guess at the real width."""
    if not box_width_px or box_width_px <= 0.0:
        return None
    if not fx_tower or fx_tower <= 0.0:
        return None
    real_width = params.widths_m.get(label or "", DEFAULT_WIDTH_M)
    return float(fx_tower) * float(real_width) / float(box_width_px)


def estimate_range(bearing, depth_mm, depth_intrinsics, box_width_px, label,
                   fx_tower, params):
    """The range and where it came from.

    Returns (range_m, source) with source one of "depth", "bbox" or "none".
    The caller is expected to act on the source: only a depth range is allowed
    to drive the body forward.
    """
    if bearing is not None:
        range_m = range_from_depth(depth_mm, depth_intrinsics, bearing, params)
        if range_m is not None:
            return range_m, "depth"
    range_m = range_from_box(box_width_px, label, fx_tower, params)
    if range_m is not None:
        return range_m, "bbox"
    return None, "none"
