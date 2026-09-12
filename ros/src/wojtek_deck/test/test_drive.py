"""Model-free tests of the drive gate (the dead-man behind /cmd_vel)."""
import pytest

from wojtek_deck.drive import DEADMAN, FOLLOW, IDLE, LIVE, DriveGate


def make(**kw):
    return DriveGate(cmd_low=(-0.6, -0.4, -0.7), cmd_high=(1.2, 0.4, 0.7),
                     height_range=(0.09, 0.17), height_default=0.125, **kw)


def test_silent_until_the_pad_speaks():
    g = make()
    assert g.tick(0.0) is None
    assert g.tick(100.0) is None
    assert g.state == IDLE


def test_sticks_scale_into_the_asymmetric_box():
    g = make()
    g.command(1.0, vx=1.0, vy=-0.5, yaw=0.5)
    vx, vy, yaw, h = g.tick(1.1)
    assert g.state == LIVE
    assert vx == pytest.approx(1.2)      # +1 -> box high
    assert vy == pytest.approx(-0.2)     # -0.5 -> half of box low
    assert yaw == pytest.approx(0.35)
    assert h == pytest.approx(0.125)     # default stance until set


def test_stick_values_are_clipped_to_unit():
    g = make()
    g.command(0.0, vx=7.0, vy=0.0, yaw=-9.0)
    vx, _, yaw, _ = g.tick(0.0)
    assert vx == pytest.approx(1.2)
    assert yaw == pytest.approx(-0.7)


def test_deadman_zeros_then_goes_silent():
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.command(0.0, vx=1.0, vy=0.0, yaw=0.0, height=0.15)
    assert g.tick(0.4)[0] == pytest.approx(1.2)
    # link gone: zeros, height held
    out = g.tick(0.6)
    assert g.state == DEADMAN
    assert out == pytest.approx((0.0, 0.0, 0.0, 0.15))
    assert g.tick(2.4) == pytest.approx((0.0, 0.0, 0.0, 0.15))
    # burst over: silence, so another source can take /cmd_vel
    assert g.tick(2.6) is None
    assert g.state == IDLE


def test_fresh_frame_recovers_from_deadman():
    g = make()
    g.command(0.0, vx=1.0, vy=0.0, yaw=0.0)
    g.tick(1.0)
    assert g.state == DEADMAN
    g.command(1.1, vx=0.5, vy=0.0, yaw=0.0)
    assert g.tick(1.2)[0] == pytest.approx(0.6)
    assert g.state == LIVE


def test_explicit_stop_starts_the_burst_immediately():
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.command(10.0, vx=1.0, vy=0.0, yaw=0.0, height=0.11)
    g.stop(10.1)
    out = g.tick(10.1)
    assert g.state == DEADMAN
    assert out == pytest.approx((0.0, 0.0, 0.0, 0.11))
    assert g.tick(12.05) is not None     # still bursting
    assert g.tick(12.2) is None          # 2 s after the (backdated) stamp


def test_stop_before_any_frame_stays_silent():
    g = make()
    g.stop(5.0)
    assert g.tick(5.0) is None
    assert g.state == IDLE


def test_height_is_clamped_and_held():
    g = make()
    g.command(0.0, 0.0, 0.0, 0.0, height=0.5)
    assert g.tick(0.0)[3] == pytest.approx(0.17)
    g.command(0.1, 0.0, 0.0, 0.0)        # no height in the frame: held
    assert g.tick(0.1)[3] == pytest.approx(0.17)
    assert g.step_height(-0.005) == pytest.approx(0.165)
    assert g.step_height(-1.0) == pytest.approx(0.09)


# ---- the follow source ------------------------------------------------------
# The follow node publishes /wojtek/follow/cmd_vel in m/s and rad/s while the
# page holds a lock. A moved stick always wins; a resting pad only keeps the
# link alive; the dead-man treats both sources the same.

def test_follow_frames_are_dropped_until_a_lock_starts():
    g = make()
    g.follow(0.0, vx=0.3, vy=0.0, yaw=0.2)
    assert g.tick(0.0) is None
    assert g.state == IDLE
    g.start_follow()
    g.follow(0.1, vx=0.3, vy=0.0, yaw=0.2)
    assert g.tick(0.1) == pytest.approx((0.3, 0.0, 0.2, 0.125))
    assert g.state == FOLLOW


def test_follow_values_are_clipped_to_the_command_box():
    g = make()
    g.start_follow()
    g.follow(0.0, vx=5.0, vy=-3.0, yaw=-9.0)
    vx, vy, yaw, _ = g.tick(0.0)
    assert vx == pytest.approx(1.2)      # box high
    assert vy == pytest.approx(-0.4)     # box low
    assert yaw == pytest.approx(-0.7)


def test_a_resting_pad_lets_the_follow_drive():
    # The Steam Deck's own controller is always connected, so the page
    # streams all-zero pad frames every 50 ms, lock or no lock. Those frames
    # keep the link alive but do not claim the gate: with a lock on and the
    # follow node fresh, every tick is FOLLOW.
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    for i in range(0, 100):                   # 5 s at 20 Hz
        t = i * 0.05
        g.command(t, vx=0.0, vy=0.0, yaw=0.0)
        if i % 2 == 0:                        # the follow node at 10 Hz
            g.follow(t, vx=0.3, vy=0.0, yaw=0.1)
        assert g.tick(t + 0.01) == pytest.approx((0.3, 0.0, 0.1, 0.125))
        assert g.state == FOLLOW
    assert g.follow_active is True


def test_a_resting_pad_keeps_the_gate_while_the_follow_node_is_quiet():
    # Lock on, pad at rest, and no follow frame yet (or none for a while):
    # the pad's zeros drive, so nothing the follow node last asked for stays
    # latched, and the pad's own dead-man covers it when the page goes away.
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    g.command(0.0, vx=0.0, vy=0.0, yaw=0.0)
    assert g.tick(0.1) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == LIVE
    g.follow(0.2, vx=0.3, vy=0.0, yaw=0.1)
    g.command(0.2, vx=0.0, vy=0.0, yaw=0.0)
    assert g.tick(0.3)[0] == pytest.approx(0.3)
    assert g.state == FOLLOW
    # The follow node stops; the pad is still resting and fresh: LIVE zeros.
    g.command(0.8, vx=0.0, vy=0.0, yaw=0.0)
    assert g.tick(0.9) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == LIVE
    # The page goes away too: the dead-man burst, then silence.
    assert g.tick(1.5) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == DEADMAN
    assert g.tick(3.5) is None
    assert g.state == IDLE


def test_follow_takes_over_when_the_pad_goes_quiet():
    # A keyboard desk session: no pad frames at rest at all.
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    g.command(0.0, vx=0.0, vy=0.0, yaw=0.0)   # one resting frame, then silence
    g.follow(0.6, vx=0.3, vy=0.0, yaw=0.1)
    assert g.tick(0.6) == pytest.approx((0.3, 0.0, 0.1, 0.125))
    assert g.state == FOLLOW


def test_a_held_stick_keeps_the_pad_in_charge_of_a_later_lock():
    # A lock that starts while a stick is still held: the pad frame is not
    # at rest, so it owns the gate until the stick returns to zero.
    g = make()
    g.command(0.0, vx=0.5, vy=0.0, yaw=0.0)
    g.start_follow()
    g.follow(0.1, vx=0.3, vy=0.0, yaw=0.1)
    assert g.tick(0.1)[0] == pytest.approx(0.6)
    assert g.state == LIVE


def test_a_non_finite_follow_frame_is_dropped_whole():
    # Python's max/min let NaN through as the box's upper limit, which would
    # send the robot off at the trained maximum. Such a frame is rejected and
    # the follow node's silence is what the dead-man then sees.
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    g.follow(0.0, vx=0.3, vy=0.0, yaw=0.1)
    assert g.tick(0.0)[0] == pytest.approx(0.3)
    for bad in (float("nan"), float("inf"), float("-inf")):
        g.follow(0.1, vx=bad, vy=0.0, yaw=bad)
        out = g.tick(0.1)
        assert out == pytest.approx((0.3, 0.0, 0.1, 0.125))   # the last good one
        assert all(v == v for v in out)
    # No good frame followed, so the last good one ages out: zeros.
    assert g.tick(0.6) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == DEADMAN


def test_a_moved_stick_ends_the_follow_and_the_pad_wins():
    g = make()
    g.start_follow()
    g.follow(0.0, vx=0.3, vy=0.0, yaw=0.0)
    g.command(0.1, vx=-1.0, vy=0.0, yaw=0.0)
    assert g.follow_active is False
    assert g.tick(0.1) == pytest.approx((-0.6, 0.0, 0.0, 0.125))
    assert g.state == LIVE
    # A follow frame arriving after that is dropped at the door, so what
    # follows is the pad's own dead-man and nothing else.
    g.follow(0.2, vx=0.3, vy=0.0, yaw=0.0)
    assert g.tick(0.7) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == DEADMAN


def test_stop_ends_the_follow_and_zeroes_what_it_left():
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    g.follow(10.0, vx=0.4, vy=0.0, yaw=0.0)
    g.stop(10.1)
    assert g.follow_active is False
    assert g.tick(10.1) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == DEADMAN
    assert g.tick(12.05) is not None     # still bursting
    assert g.tick(12.2) is None          # 2 s after the backdated stamp
    assert g.state == IDLE


def test_the_unlock_zeroes_the_follow_at_once():
    # The page dropped the lock. Whatever the follow node last asked for is
    # latched in policy_node, so the burst starts now, not in half a second.
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    g.follow(1.0, vx=0.4, vy=0.0, yaw=0.2)
    assert g.tick(1.0)[0] == pytest.approx(0.4)
    g.end_follow(1.05)
    assert g.tick(1.05) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == DEADMAN
    assert g.tick(3.6) is None
    assert g.state == IDLE


def test_the_deadman_covers_follow_frames_too():
    g = make(timeout_s=0.5, silence_after_s=2.0)
    g.start_follow()
    g.follow(0.0, vx=0.4, vy=0.0, yaw=0.0)
    assert g.tick(0.4)[0] == pytest.approx(0.4)
    # The follow node went quiet with the lock still on: zeros, then silence.
    assert g.tick(0.6) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.state == DEADMAN
    assert g.tick(2.4) == pytest.approx((0.0, 0.0, 0.0, 0.125))
    assert g.tick(2.6) is None
    assert g.state == IDLE


def test_a_fresh_follow_frame_recovers_from_its_own_deadman():
    g = make()
    g.start_follow()
    g.follow(0.0, vx=0.4, vy=0.0, yaw=0.0)
    g.tick(1.0)
    assert g.state == DEADMAN
    g.follow(1.1, vx=0.2, vy=0.0, yaw=0.0)
    assert g.tick(1.2)[0] == pytest.approx(0.2)
    assert g.state == FOLLOW


def test_follow_leaves_the_height_alone():
    g = make()
    g.command(0.0, 0.0, 0.0, 0.0, height=0.15)
    g.start_follow()
    g.follow(1.0, vx=0.3, vy=0.0, yaw=0.0)
    assert g.tick(1.0)[3] == pytest.approx(0.15)
    assert g.step_height(0.005) == pytest.approx(0.155)
    assert g.tick(1.0)[3] == pytest.approx(0.155)
