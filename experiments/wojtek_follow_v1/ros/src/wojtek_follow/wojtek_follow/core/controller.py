"""The body loop: a bearing, a range and three bands become a velocity.

One pure function, `step`, called ten times a second.  It holds no clock and no
history beyond the state it is handed and the state it returns, which is what
makes the whole controller testable without a robot.

The states and what each one means:

    IDLE      No lock.  Nothing is published at all, and the gateway's
              dead-man is what keeps the robot still.
    WAITING   A lock is held but there is no bearing to act on, because the
              tower camera has published no usable intrinsics.  Nothing is
              published, and the loss timers still run: a lock the Deck
              stops feeding is dropped from here exactly as from any other
              state.
    ALIGN     The target is more than `align_threshold_deg` off the nose.
              The robot turns in place and does not walk.  A quadruped that
              walks and turns at once toward a target 90 degrees away takes a
              long curved path, and the tower is holding the target anyway.
    APPROACH  Driving the range error toward the hold distance.  Forward
              motion needs a depth range; a range guessed from the box width
              steers the turn and never the walk.
    HOLD      Inside the hysteresis band around the hold distance.  Standing
              still while the tower keeps looking.
    BLOCKED   Something that is not the target is inside the centre band.
              Forward speed is scaled down or killed, and the robot steps
              toward whichever side is emptier.
    LOST      The track has gone stale.  Zeros for a second, then the lock is
              dropped and the state returns to IDLE.

Speeds below `min_speed` are sent as zero.  The walking policy's gait clock
freezes below about 0.05 m/s, so a command under it is not a slow walk, it is a
robot standing still with its legs loaded.  Better to ask for nothing.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple

IDLE = "IDLE"
WAITING = "WAITING"
ALIGN = "ALIGN"
APPROACH = "APPROACH"
HOLD = "HOLD"
BLOCKED = "BLOCKED"
LOST = "LOST"


@dataclass(frozen=True)
class FollowParams:
    """Every number the body loop turns on.  All of them are node parameters."""

    # Turning.
    yaw_gain: float = 1.5
    yaw_deadband_deg: float = 2.0
    yaw_max: float = 0.7
    align_threshold_deg: float = 30.0
    # Distance keeping.
    hold_distance_m: float = 1.0
    hold_hysteresis_m: float = 0.15
    forward_gain: float = 0.8
    vx_max: float = 0.4
    vx_min_reverse: float = -0.2
    # Below this the gait clock freezes, so anything smaller is sent as zero.
    min_speed: float = 0.12
    # Obstacles.
    stop_range_m: float = 0.5
    slow_range_m: float = 1.0
    sidestep: float = 0.25
    # Losing the target.
    hold_after_loss_s: float = 1.0
    drop_after_loss_s: float = 3.0


@dataclass(frozen=True)
class FollowState:
    """What the loop remembers between ticks.

    `unlock` is set for the one tick on which the lock is dropped.  The node
    reads it, tells the gimbal to stop tracking and clears the relay.
    """

    mode: str = IDLE
    unlock: bool = False


class Step(NamedTuple):
    vx: float
    vy: float
    wz: float
    state: FollowState


def _clip(value, low, high):
    return max(low, min(high, value))


def _deadband(value, threshold):
    """Anything the legs cannot walk at becomes a clean zero."""
    return 0.0 if abs(value) < threshold else value


def _radians(degrees):
    return degrees * 0.017453292519943295


def _emptier_side(bands):
    """Which way to step: +1 is left, -1 is right.

    An empty band is the emptiest thing there is, so None beats any number.
    A tie goes left, arbitrarily and consistently.
    """
    left = bands.get("left")
    right = bands.get("right")
    if left is None and right is None:
        return 1.0
    if left is None:
        return 1.0
    if right is None:
        return -1.0
    return 1.0 if left >= right else -1.0


def step(state, bearing, range_m, range_source, bands, track_age, dt, params):
    """One tick of the body loop.

    `bearing` is a `geometry.Bearing` in radians, or None when there is
    nothing to aim by.  `range_m` is metres or None, and `range_source` is
    "depth", "bbox" or "none".  `bands` is the obstacle gate's dict.
    `track_age` is how stale the Deck's detection is, in seconds, and
    infinity when there is no lock at all.  `dt` is the tick period and is
    carried for the derivative terms this controller does not yet have.

    The loss timers come before the bearing check on purpose.  A lock with no
    bearing is a lock all the same, and when the Deck stops feeding it the
    gimbal must be released and the relay cleared, bearing or no bearing.

    Returns (vx, vy, wz, new_state).
    """
    del dt  # No rate limiting yet; the policy and the 10 Hz tick do that.
    bands = bands or {}

    if bearing is None and math.isinf(track_age):
        return Step(0.0, 0.0, 0.0, FollowState(IDLE, False))

    if track_age > params.drop_after_loss_s:
        # The lock is over.  The one tick that says so carries `unlock`.
        return Step(0.0, 0.0, 0.0, FollowState(IDLE, True))

    if track_age > params.hold_after_loss_s:
        return Step(0.0, 0.0, 0.0, FollowState(LOST, False))

    if bearing is None:
        return Step(0.0, 0.0, 0.0, FollowState(WAITING, False))

    azimuth = bearing.azimuth
    if abs(azimuth) <= _radians(params.yaw_deadband_deg):
        wz = 0.0
    else:
        wz = _clip(params.yaw_gain * azimuth, -params.yaw_max, params.yaw_max)

    if abs(azimuth) > _radians(params.align_threshold_deg):
        return Step(0.0, 0.0, wz, FollowState(ALIGN, False))

    mode = APPROACH
    vx = 0.0
    vy = 0.0
    if range_source == "depth" and range_m is not None:
        error = range_m - params.hold_distance_m
        band = params.hold_hysteresis_m
        # Widening the band once inside it is what stops the robot shuffling
        # back and forth across the hold distance.
        if state.mode == HOLD:
            band = 2.0 * params.hold_hysteresis_m
        if abs(error) <= band:
            mode = HOLD
            vx = 0.0
        else:
            mode = APPROACH
            vx = _clip(params.forward_gain * error,
                       params.vx_min_reverse, params.vx_max)

    centre = bands.get("centre")
    if centre is not None and centre < params.slow_range_m:
        mode = BLOCKED
        if centre < params.stop_range_m:
            # Never forward into it.  Backing away from the target is still
            # allowed, because that moves away from the obstacle too.
            vx = min(vx, 0.0)
        elif vx > 0.0:
            span = params.slow_range_m - params.stop_range_m
            scale = (centre - params.stop_range_m) / span if span > 0.0 else 0.0
            vx = vx * _clip(scale, 0.0, 1.0)
        vy = params.sidestep * _emptier_side(bands)

    vx = _deadband(vx, params.min_speed)
    vy = _deadband(vy, params.min_speed)
    vx = _clip(vx, params.vx_min_reverse, params.vx_max)
    vy = _clip(vy, -params.sidestep, params.sidestep)
    wz = _clip(wz, -params.yaw_max, params.yaw_max)
    return Step(vx, vy, wz, FollowState(mode, False))
