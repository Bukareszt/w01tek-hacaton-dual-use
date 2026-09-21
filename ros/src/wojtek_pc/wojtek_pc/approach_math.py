"""The scripted approach's geometry, with no ROS in it.

`sim_approach` imports these; they live apart so they can be unit tested
on a machine with no rclpy (see `test/test_sim_approach.py`). Pure math,
no state, no messages.
"""

import math


def yaw_of(qw, qx, qy, qz):
    """Heading about +z of a w-first quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    """`a` folded into (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def nearest(targets, x, y):
    """The target closest to (x, y), or None when there is none."""
    best = None
    for t in targets:
        d = math.hypot(t["x"] - x, t["y"] - y)
        if best is None or d < best[0]:
            best = (d, t)
    return None if best is None else best[1]


def settle_command(err, yaw_min, yaw_max):
    """The yaw rate that turns in place toward a heading error of `err`.

    The gait barely turns below ~0.2 rad/s, so even a small error asks for
    at least `yaw_min`; a large one is capped at `yaw_max`. The sign is the
    sign of the error.
    """
    return math.copysign(max(yaw_min, min(yaw_max, abs(err))), err)
