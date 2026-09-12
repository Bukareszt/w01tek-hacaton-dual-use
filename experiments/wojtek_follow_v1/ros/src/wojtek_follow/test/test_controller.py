"""The body loop: every state, every transition and every limit."""

import dataclasses
import math

import pytest

from wojtek_follow.core.controller import (
    ALIGN,
    APPROACH,
    BLOCKED,
    FollowParams,
    FollowState,
    HOLD,
    IDLE,
    LOST,
    WAITING,
    step,
)
from wojtek_follow.core.geometry import Bearing

P = FollowParams()
DT = 0.1
CLEAR = {"left": None, "centre": None, "right": None}
NO_TRACK = float("inf")


def ahead(azimuth_deg=0.0):
    return Bearing(azimuth=math.radians(azimuth_deg), elevation=0.0)


def run(state=None, bearing=None, range_m=None, source="none", bands=None,
        track_age=0.0, params=P):
    return step(state or FollowState(), bearing if bearing is not None else ahead(),
                range_m, source, bands if bands is not None else dict(CLEAR),
                track_age, DT, params)


# --- no lock and losing one ----------------------------------------------

def test_no_lock_is_idle_and_silent():
    """No track at all: the relay reports an infinite age and no bearing."""
    vx, vy, wz, state = step(FollowState(), None, None, "none", CLEAR, NO_TRACK, DT, P)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert state.mode == IDLE
    assert state.unlock is False


def test_a_lock_without_a_bearing_waits():
    """A track is held but the tower camera has given no intrinsics yet."""
    vx, vy, wz, state = step(FollowState(), None, None, "none", CLEAR, 0.2, DT, P)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert state.mode == WAITING
    assert state.unlock is False


def test_a_lock_without_a_bearing_still_goes_lost_and_drops():
    """The loss timers do not need a bearing: a dead track is dropped anyway."""
    _, _, _, state = step(FollowState(WAITING), None, None, "none", CLEAR, 1.5, DT, P)
    assert state.mode == LOST
    assert state.unlock is False
    vx, vy, wz, state = step(state, None, None, "none", CLEAR, 3.5, DT, P)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert state.mode == IDLE
    assert state.unlock is True


def test_a_stale_track_stops_the_body():
    vx, vy, wz, state = run(track_age=1.5, range_m=2.0, source="depth")
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert state.mode == LOST
    assert state.unlock is False


def test_a_track_at_the_hold_timer_is_still_good():
    _, _, _, state = run(track_age=P.hold_after_loss_s, range_m=1.0, source="depth")
    assert state.mode == HOLD


def test_a_very_stale_track_drops_the_lock():
    vx, vy, wz, state = run(state=FollowState(LOST), track_age=3.5)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert state.mode == IDLE
    assert state.unlock is True


def test_the_drop_only_fires_once():
    """The second tick has no bearing, because the node cleared the relay."""
    _, _, _, state = run(track_age=3.5)
    assert state.unlock is True
    _, _, _, state = step(state, None, None, "none", CLEAR, NO_TRACK, DT, P)
    assert state.unlock is False
    assert state.mode == IDLE


# --- turning --------------------------------------------------------------

def test_a_target_to_the_left_turns_the_robot_left():
    _, _, wz, state = run(bearing=ahead(20.0), range_m=1.0, source="depth")
    assert wz > 0.0
    assert state.mode in (APPROACH, HOLD)


def test_a_target_to_the_right_turns_the_robot_right():
    _, _, wz, _ = run(bearing=ahead(-20.0), range_m=1.0, source="depth")
    assert wz < 0.0


def test_the_yaw_deadband():
    """Inside a couple of degrees the robot does not chase gimbal backlash."""
    assert run(bearing=ahead(1.5))[2] == 0.0
    assert run(bearing=ahead(-1.5))[2] == 0.0
    assert run(bearing=ahead(5.0))[2] > 0.0


def test_the_yaw_gain_and_its_limit():
    _, _, wz, _ = run(bearing=ahead(10.0), range_m=1.0, source="depth")
    assert wz == pytest.approx(P.yaw_gain * math.radians(10.0))
    assert run(bearing=ahead(120.0))[2] == pytest.approx(P.yaw_max)
    assert run(bearing=ahead(-120.0))[2] == pytest.approx(-P.yaw_max)


def test_a_target_far_off_the_nose_is_turned_to_first():
    vx, vy, wz, state = run(bearing=ahead(60.0), range_m=2.0, source="depth")
    assert state.mode == ALIGN
    assert (vx, vy) == (0.0, 0.0)
    assert wz == pytest.approx(P.yaw_max)


def test_aligning_beats_the_obstacle_gate():
    """Turning in place is safe with something in front, and walking is not."""
    vx, vy, _, state = run(bearing=ahead(60.0), range_m=2.0, source="depth",
                           bands={"left": None, "centre": 0.3, "right": None})
    assert state.mode == ALIGN
    assert (vx, vy) == (0.0, 0.0)


def test_the_align_threshold_is_where_walking_starts():
    assert run(bearing=ahead(31.0), range_m=2.0, source="depth")[3].mode == ALIGN
    assert run(bearing=ahead(29.0), range_m=2.0, source="depth")[3].mode == APPROACH


# --- approach and hold ----------------------------------------------------

def test_approach_drives_the_range_error():
    vx, _, _, state = run(range_m=1.5, source="depth")
    assert state.mode == APPROACH
    assert vx == pytest.approx(P.forward_gain * 0.5)


def test_the_forward_limit():
    assert run(range_m=3.5, source="depth")[0] == pytest.approx(P.vx_max)


def test_backing_off_is_slower_than_walking_forward():
    """The robot cannot see behind itself, so reverse is small."""
    vx, _, _, state = run(range_m=0.4, source="depth")
    assert state.mode == APPROACH
    assert vx == pytest.approx(P.vx_min_reverse)


def test_holding_the_distance():
    vx, vy, wz, state = run(range_m=1.0, source="depth")
    assert state.mode == HOLD
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)


def test_the_hold_band_widens_once_inside_it():
    """Hysteresis, so the robot does not shuffle across the hold distance."""
    _, _, _, state = run(range_m=1.14, source="depth")
    assert state.mode == HOLD
    # Still holding at an error that would have started an approach.
    _, _, _, state = run(state=FollowState(HOLD), range_m=1.25, source="depth")
    assert state.mode == HOLD
    # Past twice the band it walks again.
    vx, _, _, state = run(state=FollowState(HOLD), range_m=1.4, source="depth")
    assert state.mode == APPROACH
    assert vx == pytest.approx(P.forward_gain * 0.4)


def test_a_guessed_range_never_drives_the_walk():
    """A box-width range steers the turn and nothing else."""
    vx, vy, wz, state = run(bearing=ahead(10.0), range_m=3.0, source="bbox")
    assert state.mode == APPROACH
    assert vx == 0.0
    assert vy == 0.0
    assert wz > 0.0


def test_no_range_at_all_means_no_walking():
    vx, _, _, state = run(range_m=None, source="none")
    assert state.mode == APPROACH
    assert vx == 0.0


def test_speeds_the_gait_cannot_walk_are_sent_as_zero():
    """The gait clock freezes below about 0.05 m/s, so a crawl is a stand."""
    gentle = dataclasses.replace(P, forward_gain=0.2)
    vx, _, _, state = run(range_m=1.3, source="depth", params=gentle)
    assert state.mode == APPROACH
    assert vx == 0.0


# --- obstacles ------------------------------------------------------------

def test_something_close_in_front_stops_the_walk():
    vx, vy, _, state = run(range_m=2.0, source="depth",
                           bands={"left": None, "centre": 0.4, "right": 2.0})
    assert state.mode == BLOCKED
    assert vx == 0.0
    # The left band is empty, so that is the way around.
    assert vy == pytest.approx(P.sidestep)


def test_something_at_middle_distance_slows_the_walk():
    vx, _, _, state = run(range_m=3.0, source="depth",
                          bands={"left": 2.0, "centre": 0.75, "right": 2.0})
    assert state.mode == BLOCKED
    assert vx == pytest.approx(P.vx_max * 0.5)


def test_a_slowed_walk_below_the_gait_speed_becomes_a_stand():
    vx, _, _, state = run(range_m=3.0, source="depth",
                          bands={"left": 2.0, "centre": 0.6, "right": 2.0})
    assert state.mode == BLOCKED
    assert vx == 0.0


def test_backing_away_is_still_allowed_when_blocked():
    """Reverse moves away from the obstacle in front as well as from the target."""
    vx, _, _, state = run(range_m=0.4, source="depth",
                          bands={"left": None, "centre": 0.4, "right": None})
    assert state.mode == BLOCKED
    assert vx == pytest.approx(P.vx_min_reverse)


def test_the_sidestep_goes_toward_the_emptier_side():
    blocked = {"centre": 0.4}
    assert run(bands=dict(blocked, left=2.0, right=1.0))[1] == pytest.approx(P.sidestep)
    assert run(bands=dict(blocked, left=1.0, right=2.0))[1] == pytest.approx(-P.sidestep)
    assert run(bands=dict(blocked, left=None, right=1.0))[1] == pytest.approx(P.sidestep)
    assert run(bands=dict(blocked, left=1.0, right=None))[1] == pytest.approx(-P.sidestep)
    # Nothing on either side, and a tie, both go left.
    assert run(bands=dict(blocked, left=None, right=None))[1] == pytest.approx(P.sidestep)
    assert run(bands=dict(blocked, left=1.0, right=1.0))[1] == pytest.approx(P.sidestep)


def test_an_obstacle_beyond_the_slow_range_is_not_a_blockage():
    vx, vy, _, state = run(range_m=2.0, source="depth",
                           bands={"left": None, "centre": 1.2, "right": None})
    assert state.mode == APPROACH
    assert vy == 0.0
    assert vx == pytest.approx(P.vx_max)


def test_holding_next_to_an_obstacle_still_sidesteps():
    vx, vy, _, state = run(state=FollowState(HOLD), range_m=1.0, source="depth",
                           bands={"left": 2.0, "centre": 0.4, "right": None})
    assert state.mode == BLOCKED
    assert vx == 0.0
    assert vy == pytest.approx(-P.sidestep)


def test_missing_bands_are_treated_as_empty():
    vx, vy, _, state = step(FollowState(), ahead(), 2.0, "depth", None, 0.0, DT, P)
    assert state.mode == APPROACH
    assert (vx, vy) == (pytest.approx(P.vx_max), 0.0)


def test_every_output_stays_inside_the_limits():
    wild = dataclasses.replace(P, forward_gain=50.0, yaw_gain=50.0, sidestep=5.0)
    vx, vy, wz, _ = step(FollowState(), ahead(20.0), 3.5, "depth",
                         {"left": None, "centre": 0.4, "right": None}, 0.0, DT, wild)
    assert P.vx_min_reverse <= vx <= P.vx_max
    assert abs(wz) <= wild.yaw_max
    assert abs(vy) <= wild.sidestep
