"""The obstacle gate: three numbers that say where the robot may walk.

The D435 depth image is back-projected into the body frame, cut down to the
things that are neither floor nor ceiling, and split into a left band, a centre
band and a right band.  Each band reports its nearest point.  The controller
uses the centre band to slow down and stop, and the two side bands to pick
which way to step around.

This is deliberately not a map.  There is no pose, no accumulation and no
memory between frames, so a thing that leaves the field of view stops existing.
That is the follow v1 bargain: a reactive gate is worth having and a map is not
affordable on a Raspberry Pi 3 that is also walking.

Three things are thrown away on purpose.  Points below 0.06 m are the floor, or
close enough that calling them obstacles would stop the robot on its own
shadow.  Points above 0.35 m are above the body and the robot walks under them.
Points nearer than 0.3 m are inside the D435's blind zone and are noise.

Heights are measured from the floor and ranges from the sensor.  The range is
the length of the ray from the depth camera to the point, which is the same
quantity the rangefinder reports for the target, so the blind-zone cut and the
target cone compare like with like.

The target itself is an obstacle by every measure here, so its cone is cut out
before the bands are taken.  Without that the robot would refuse to approach
the thing it is following.
"""

import math
from dataclasses import dataclass

import numpy as np

from .geometry import optical_points_to_body


@dataclass(frozen=True)
class GateParams:
    """Where the depth camera is, and what counts as an obstacle."""

    # Every fourth pixel in both axes.  424x240 becomes 106x60, which is 6360
    # points per frame instead of 101760.
    decimate: int = 4
    # The D435's optical axis relative to the body x axis, positive downward.
    depth_camera_pitch_deg: float = 0.0
    # How far the D435 sits above the floor.
    depth_camera_height_m: float = 0.20
    # The band of heights that is neither floor nor overhead.
    min_height_m: float = 0.06
    max_height_m: float = 0.35
    # The band of ranges that is neither blind zone nor irrelevant.
    min_range_m: float = 0.3
    max_range_m: float = 3.0
    # The half-width of the centre band.
    side_azimuth_deg: float = 15.0
    # The cone around the target that is not treated as an obstacle.  Its
    # half-angle is the larger of this floor and the angle that
    # cone_half_width_m subtends at the target's range, so a person at the
    # hold distance is cut out whole.  At one metre 0.35 m is 19 degrees,
    # wider than the centre band; at three metres it is 7 degrees.
    cone_azimuth_deg: float = 10.0
    cone_half_width_m: float = 0.35
    # How far past the target's own range that cone reaches.
    cone_margin_m: float = 0.3


def body_points(depth_mm, intrinsics, params):
    """Every kept depth pixel as a body-frame point.

    Returns (azimuth, range_m) arrays, already filtered by height and range.
    The azimuth is in the body frame and the range is from the sensor.
    """
    if depth_mm is None or intrinsics is None or not intrinsics.valid:
        return np.empty(0), np.empty(0)
    step = max(1, int(params.decimate))
    depth = np.asarray(depth_mm)[::step, ::step]
    height, width = depth.shape[:2]
    if height == 0 or width == 0:
        return np.empty(0), np.empty(0)
    rows = np.arange(0, depth_mm.shape[0], step)[:height]
    cols = np.arange(0, depth_mm.shape[1], step)[:width]
    u, v = np.meshgrid(cols.astype(np.float64), rows.astype(np.float64))
    z = depth.astype(np.float64) / 1000.0
    valid = z > 0.0
    if not np.any(valid):
        return np.empty(0), np.empty(0)
    z = z[valid]
    u = u[valid]
    v = v[valid]
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    # The range from the sensor, taken before the point is moved into the
    # body frame.  The body transform adds the camera's height, and a range
    # taken after it would be measured from a point on the floor below the
    # camera: a level return at 0.25 m would then read 0.32 m and slip past
    # the blind-zone cut.
    ranges = np.sqrt(x * x + y * y + z * z)
    bx, by, bz = optical_points_to_body(
        x, y, z,
        camera_pitch=math.radians(params.depth_camera_pitch_deg),
        camera_height=params.depth_camera_height_m,
    )
    keep = (
        (bz >= params.min_height_m)
        & (bz <= params.max_height_m)
        & (ranges >= params.min_range_m)
        & (ranges <= params.max_range_m)
    )
    if not np.any(keep):
        return np.empty(0), np.empty(0)
    azimuth = np.arctan2(by[keep], bx[keep])
    return azimuth, ranges[keep]


def obstacle_bands(depth_mm, intrinsics, params, target_azimuth=None,
                   target_range=None):
    """The nearest obstacle in each band, in metres.

    Returns {"left": value, "centre": value, "right": value} where a value is
    None when that band holds nothing.  None means empty, not near, and the
    controller has to read it that way.
    """
    azimuth, ranges = body_points(depth_mm, intrinsics, params)
    bands = {"left": None, "centre": None, "right": None}
    if azimuth.size == 0:
        return bands
    if target_azimuth is not None and target_range is not None:
        half_angle = max(
            math.radians(params.cone_azimuth_deg),
            math.atan2(params.cone_half_width_m, max(float(target_range), 0.1)),
        )
        cone = (
            (np.abs(azimuth - float(target_azimuth)) < half_angle)
            & (ranges < float(target_range) + params.cone_margin_m)
        )
        azimuth = azimuth[~cone]
        ranges = ranges[~cone]
        if azimuth.size == 0:
            return bands
    side = math.radians(params.side_azimuth_deg)
    masks = {
        "left": azimuth > side,
        "centre": np.abs(azimuth) <= side,
        "right": azimuth < -side,
    }
    for name, mask in masks.items():
        if np.any(mask):
            bands[name] = float(np.min(ranges[mask]))
    return bands
