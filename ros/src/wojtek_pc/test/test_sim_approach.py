"""Unit tests for the scripted approach's geometry (wojtek_pc.approach_math).

Pure math -- no ROS, no rclpy, no mujoco. `sim_approach` itself imports
rclpy at module top and cannot be imported on a desk machine, which is why
its maths lives in `approach_math`; these tests import that and read
sim_approach as text only. Run anywhere:

    pytest ros/src/wojtek_pc/test/test_sim_approach.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wojtek_pc.approach_math import (  # noqa: E402
    nearest,
    settle_command,
    wrap,
    yaw_of,
)

SIM_APPROACH = (
    Path(__file__).resolve().parents[1] / "wojtek_pc" / "sim_approach.py"
)


class TestWrap:
    def test_small_angles_pass_through(self):
        for a in (0.0, 0.3, -0.3, 1.5, -1.5):
            assert abs(wrap(a) - a) < 1e-12

    def test_just_past_pi_comes_back_negative(self):
        assert abs(wrap(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-12

    def test_just_below_minus_pi_comes_back_positive(self):
        assert abs(wrap(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-12

    def test_the_boundaries_land_on_the_seam(self):
        # +-pi and their multiples are the same heading; wrap returns the
        # half-open end (-pi), which is what matters: the magnitude is pi
        # and nothing escapes the interval.
        for a in (math.pi, -math.pi, 3.0 * math.pi, -3.0 * math.pi):
            assert abs(abs(wrap(a)) - math.pi) < 1e-9

    def test_nothing_escapes_the_interval(self):
        for i in range(-100, 101):
            a = i * 0.37
            assert -math.pi - 1e-12 <= wrap(a) <= math.pi + 1e-12

    def test_a_full_turn_changes_nothing(self):
        for a in (0.4, -1.2, 2.9):
            assert abs(wrap(a + 2.0 * math.pi) - a) < 1e-12


class TestYawOf:
    def test_identity_is_zero(self):
        assert abs(yaw_of(1.0, 0.0, 0.0, 0.0)) < 1e-12

    def test_ninety_degrees_about_z(self):
        h = math.radians(45.0)
        yaw = yaw_of(math.cos(h), 0.0, 0.0, math.sin(h))
        assert abs(yaw - math.pi / 2.0) < 1e-9

    def test_minus_ninety_degrees_about_z(self):
        h = math.radians(45.0)
        yaw = yaw_of(math.cos(h), 0.0, 0.0, -math.sin(h))
        assert abs(yaw + math.pi / 2.0) < 1e-9

    def test_a_roll_does_not_become_a_heading(self):
        # 90 deg about +x: the body is on its side, still pointing at +x.
        h = math.radians(45.0)
        assert abs(yaw_of(math.cos(h), math.sin(h), 0.0, 0.0)) < 1e-9


class TestNearest:
    def test_picks_the_closest(self):
        targets = [
            {"name": "far", "x": 10.0, "y": 0.0},
            {"name": "near", "x": 1.0, "y": 1.0},
            {"name": "middling", "x": -4.0, "y": 2.0},
        ]
        assert nearest(targets, 0.0, 0.0)["name"] == "near"

    def test_closest_is_measured_from_the_robot_not_the_origin(self):
        targets = [
            {"name": "a", "x": 0.0, "y": 0.0},
            {"name": "b", "x": 9.0, "y": 0.0},
        ]
        assert nearest(targets, 8.0, 0.0)["name"] == "b"

    def test_empty_is_none(self):
        assert nearest([], 0.0, 0.0) is None
        assert nearest((), 3.0, -2.0) is None

    def test_one_target_is_that_target(self):
        only = {"name": "only", "x": -3.0, "y": 4.0}
        assert nearest([only], 0.0, 0.0) is only


class TestSettleCommand:
    def test_a_small_error_still_asks_for_yaw_min(self):
        # The gait ignores anything slower, so a near-aligned body would
        # otherwise never finish turning.
        assert settle_command(0.05, 0.35, 0.6) == 0.35
        assert settle_command(-0.05, 0.35, 0.6) == -0.35

    def test_a_large_error_is_capped_at_yaw_max(self):
        assert settle_command(2.0, 0.35, 0.6) == 0.6
        assert settle_command(-2.0, 0.35, 0.6) == -0.6

    def test_in_between_the_error_is_the_rate(self):
        assert abs(settle_command(0.5, 0.35, 0.6) - 0.5) < 1e-12
        assert abs(settle_command(-0.5, 0.35, 0.6) + 0.5) < 1e-12

    def test_the_sign_follows_the_error(self):
        for err in (0.05, 0.5, 2.0):
            assert settle_command(err, 0.35, 0.6) > 0.0
            assert settle_command(-err, 0.35, 0.6) < 0.0

    def test_zero_error_turns_at_yaw_min(self):
        assert abs(settle_command(0.0, 0.35, 0.6)) == 0.35


class TestSimApproachUsesTheseHelpers:
    """Read sim_approach as text: importing it needs rclpy, which a desk
    machine does not have, but the wiring is still worth pinning."""

    def test_it_imports_the_pure_module(self):
        src = SIM_APPROACH.read_text()
        assert "from wojtek_pc.approach_math import" in src
        # The copies that used to live there are gone, not shadowed.
        for name in ("def wrap(", "def yaw_of(", "def nearest(",
                     "def settle_command("):
            assert name not in src

    def test_align_face_and_settle_all_go_through_settle_command(self):
        src = SIM_APPROACH.read_text()
        assert src.count("settle_command(") == 3  # ALIGN, FACE, SETTLE
        assert "math.copysign" not in src

    def test_home_returns_to_the_spawn_with_a_short_standoff(self):
        src = SIM_APPROACH.read_text()
        assert "HOME_X, HOME_Y = 0.0, 0.0" in src
        assert "HOME_STANDOFF = 0.4" in src
        assert "HOME_YAW = 0.0" in src
        assert 'create_service(Trigger, "wojtek/intercept_home"' in src
