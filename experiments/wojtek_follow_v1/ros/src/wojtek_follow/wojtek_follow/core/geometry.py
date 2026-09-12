"""Pinhole geometry for the tower camera, and the body bearing it implies.

Angles are radians everywhere inside this package.  Degrees appear only at the
ROS boundary, where the message field names say deg.

Two conventions meet in this module and they are not the same one.

The optical convention is the tower team's, taken from
`wojtek_targeting/kinematics.py`.  Yaw is positive when the target is to the
right of the optical axis.  Pitch is positive when the target is below the
axis, because image rows run downward.

The body convention is ROS on a z-up body.  x is forward, y is left, z is up.
Azimuth is positive counterclockwise seen from above, so a positive azimuth is
to the robot's left.  Elevation is positive upward.

The two are tied together by the gimbal angles.  The camera's own azimuth in
the body frame is `pan_sign * pan`, and its elevation is `tilt_sign * tilt`.
The optical yaw is the camera's azimuth minus the target's azimuth, and the
optical pitch is the camera's elevation minus the target's elevation.  Every
sign in `aim.py` follows from those two identities.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

# tan() runs away at a quarter turn, so a predicted angle is clamped just short
# of it before it becomes a pixel.  A target that far off the axis is outside
# the picture either way.
_MAX_ANGLE = math.pi / 2.0 - 1e-6


class Bearing(NamedTuple):
    """Where the target is in the body frame, in radians."""

    azimuth: float
    elevation: float


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole parameters as they arrive on `camera_info`.

    An `fx` or `fy` of zero means the camera has never been calibrated.  A
    pixel offset carries no angle without a focal length, so `valid` is false
    and the follow node refuses to aim.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def valid(self) -> bool:
        return self.width > 0 and self.height > 0 and self.fx > 0.0 and self.fy > 0.0

    @classmethod
    def from_camera_info(cls, msg):
        """Read a `sensor_msgs/CameraInfo` without importing one."""
        k = list(msg.k)
        return cls(
            width=int(msg.width),
            height=int(msg.height),
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
        )


def scale_intrinsics(intrinsics, width, height):
    """The same camera described in a picture of a different size.

    The Deck page reports its box in the pixels of whatever frame the MJPEG
    stream handed it.  That frame can be a resized copy of the one
    `camera_info` describes.  Scaling the intrinsics is exact for a resize and
    wrong for a crop, and the gateway resizes.
    """
    if not intrinsics.valid:
        raise ValueError("camera intrinsics are not usable")
    if width <= 0 or height <= 0:
        raise ValueError("frame size must be positive")
    if width == intrinsics.width and height == intrinsics.height:
        return intrinsics
    sx = float(width) / float(intrinsics.width)
    sy = float(height) / float(intrinsics.height)
    return Intrinsics(
        width=int(width),
        height=int(height),
        fx=intrinsics.fx * sx,
        fy=intrinsics.fy * sy,
        cx=intrinsics.cx * sx,
        cy=intrinsics.cy * sy,
    )


def pixel_to_optical(px, py, intrinsics):
    """Pixel to the angle off the optical axis, in radians.

    Yaw is positive to the right and pitch is positive downward.
    """
    if not intrinsics.valid:
        raise ValueError("camera intrinsics are not usable")
    yaw = math.atan2(px - intrinsics.cx, intrinsics.fx)
    pitch = math.atan2(py - intrinsics.cy, intrinsics.fy)
    return yaw, pitch


def optical_to_pixel(yaw, pitch, intrinsics):
    """The inverse of `pixel_to_optical`."""
    if not intrinsics.valid:
        raise ValueError("camera intrinsics are not usable")
    yaw = max(-_MAX_ANGLE, min(_MAX_ANGLE, yaw))
    pitch = max(-_MAX_ANGLE, min(_MAX_ANGLE, pitch))
    px = intrinsics.cx + intrinsics.fx * math.tan(yaw)
    py = intrinsics.cy + intrinsics.fy * math.tan(pitch)
    return px, py


def pixel_to_normalized(px, py, width, height):
    """Pixel to the LaserTarget message's -1..1 across the picture."""
    return 2.0 * px / float(width) - 1.0, 2.0 * py / float(height) - 1.0


def normalized_to_pixel(target_x, target_y, width, height):
    """The message's -1..1 back to a pixel, the way the gimbal node reads it."""
    return (target_x + 1.0) * float(width) / 2.0, (target_y + 1.0) * float(height) / 2.0


def inside_image(px, py, intrinsics):
    """Whether a pixel is on the sensor."""
    return 0.0 <= px <= intrinsics.width - 1.0 and 0.0 <= py <= intrinsics.height - 1.0


def body_bearing(pan, tilt, yaw, pitch, pan_sign=1.0, tilt_sign=1.0):
    """Where the target is in the body frame, from the gimbal and the pixel.

    The camera points at azimuth `pan_sign * pan` and elevation
    `tilt_sign * tilt`.  The target sits `yaw` to the right of that axis and
    `pitch` below it, so its azimuth is that much further clockwise and its
    elevation that much further down.
    """
    return Bearing(
        azimuth=pan_sign * pan - yaw,
        elevation=tilt_sign * tilt - pitch,
    )


def optical_from_bearing(bearing, pan, tilt, pan_sign=1.0, tilt_sign=1.0):
    """The inverse of `body_bearing`, which is what makes the round trip a test."""
    return (
        pan_sign * pan - bearing.azimuth,
        tilt_sign * tilt - bearing.elevation,
    )


def body_direction_to_optical(azimuth, elevation, camera_pitch=0.0):
    """A body-frame bearing as a unit vector in a camera's optical frame.

    The camera looks along the body x axis rotated about the body y axis by
    `camera_pitch`.  Positive `camera_pitch` points the camera downward, which
    is the ROS sense of a pitch about the +y (left) axis.  The optical frame is
    the ROS camera one: x to the right, y downward, z forward.
    """
    cp = math.cos(camera_pitch)
    sp = math.sin(camera_pitch)
    ce = math.cos(elevation)
    dx = ce * math.cos(azimuth)
    dy = ce * math.sin(azimuth)
    dz = math.sin(elevation)
    forward = dx * cp - dz * sp
    left = dy
    up = dx * sp + dz * cp
    return -left, -up, forward


def optical_points_to_body(x, y, z, camera_pitch=0.0, camera_height=0.0):
    """Optical-frame points to body-frame points, as arrays.

    `camera_height` is how far the camera sits above the floor, so the body z
    that comes back is a height above the floor and not a height above the
    camera.
    """
    cp = math.cos(camera_pitch)
    sp = math.sin(camera_pitch)
    forward = np.asarray(z, dtype=np.float64)
    left = -np.asarray(x, dtype=np.float64)
    up = -np.asarray(y, dtype=np.float64)
    bx = forward * cp + up * sp
    by = left
    bz = -forward * sp + up * cp + camera_height
    return bx, by, bz
