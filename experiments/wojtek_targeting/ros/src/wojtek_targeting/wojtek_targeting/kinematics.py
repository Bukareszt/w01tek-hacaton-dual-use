"""Turning a detection into a pan/tilt correction.

The camera is mounted *on* the gimbal (`targeting_tilt_link ->
targeting_camera_link`), which makes aiming a servoing problem rather than a
geometry problem: the target's angle away from the optical axis is exactly the
angle the gimbal has to turn to centre it, and driving that to zero centres the
target whatever the range. That is why nothing here needs TF, depth, or a 3D
target point. A body-mounted camera with a separate gimbal would need all
three.

What that leaves out, deliberately: the laser sits a few centimetres off the
optical axis, so putting the target in the centre of the image is not quite the
same as putting the laser on it. The error is the parallax angle, offset over
range -- about a degree for a 5 cm offset at 3 m. That is absorbed by the
calibrated `pan_offset_deg`/`tilt_offset_deg` at the distance the demo is
actually shot at. Modelling it properly needs a depth the detector does not
reliably have, and would buy nothing at a fixed demo distance.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole parameters as they arrive on `camera_info`.

    `fx`/`fy` at zero means the camera has never been calibrated. The centre of
    the image is a reasonable stand-in for an unknown principal point, but
    there is no stand-in for an unknown focal length: without it a pixel offset
    carries no angle at all, so `valid` is false and the controller refuses to
    track.
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


def target_to_optical_angles(target_x, target_y, intrinsics):
    """Angle from the optical axis to a normalized detection, in degrees.

    `target_x`/`target_y` are the message's -1..1 across the image. The result
    follows the optical frame: x positive to the right, y positive *down*,
    because that is the direction image rows run. Which way a servo has to turn
    for that is a mounting question, so it is the caller's to apply.
    """
    if not intrinsics.valid:
        raise ValueError("camera intrinsics are not usable")

    pixel_x = (target_x + 1.0) * intrinsics.width / 2.0
    pixel_y = (target_y + 1.0) * intrinsics.height / 2.0

    yaw_deg = math.degrees(math.atan2(pixel_x - intrinsics.cx, intrinsics.fx))
    pitch_deg = math.degrees(math.atan2(pixel_y - intrinsics.cy, intrinsics.fy))
    return yaw_deg, pitch_deg


def clamp(value, low, high):
    return max(low, min(high, value))


def rate_limit(desired, current, max_rate_deg_s, dt):
    """Hold a commanded angle to `max_rate_deg_s` away from where it is now.

    The servo has its own motion profile, so this is not what keeps the gimbal
    from snapping -- it is what keeps a single bad detection from asking it to.
    A spurious box on the far side of the frame becomes one tick of slew
    instead of a full-speed traverse, and the target timeout cancels it before
    it gets far.
    """
    if dt <= 0.0 or max_rate_deg_s <= 0.0:
        return current
    step = max_rate_deg_s * dt
    return clamp(desired, current - step, current + step)
