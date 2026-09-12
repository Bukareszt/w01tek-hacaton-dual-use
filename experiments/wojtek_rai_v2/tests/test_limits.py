"""The safety envelope is exactly what the README promises. Model-free."""

import pytest

from wojtek_rai import limits


def test_validate_walk_accepts_every_direction_in_range():
    for d in limits.WALK_DIRECTIONS:
        assert limits.validate_walk(d, 1.0) == (d, 1.0)


def test_validate_walk_normalises_case_and_whitespace():
    assert limits.validate_walk("  Forward ", "2") == ("forward", 2.0)


@pytest.mark.parametrize("direction", ["backward", "stop", "", "up", None])
def test_validate_walk_rejects_unknown_direction(direction):
    with pytest.raises(ValueError, match="direction must be one of"):
        limits.validate_walk(direction, 1.0)


@pytest.mark.parametrize("seconds", [0.0, -1.0, limits.MOVE_MAX_SECONDS + 0.01, 60])
def test_validate_walk_rejects_out_of_range_seconds(seconds):
    with pytest.raises(ValueError, match="seconds must be between"):
        limits.validate_walk("forward", seconds)


def test_validate_walk_rejects_non_numeric_seconds():
    with pytest.raises(ValueError, match="must be a number"):
        limits.validate_walk("forward", "two")


def test_republish_is_well_inside_text_commander_deadman():
    assert limits.REPUBLISH_PERIOD_S * 2 < limits.TEXT_COMMANDER_DEADMAN_S


def test_only_nav_command_is_writable_and_cmd_vel_is_forbidden():
    assert limits.WRITABLE_TOPICS == (limits.NAV_COMMAND_TOPIC,)
    assert limits.CMD_VEL_TOPIC in limits.FORBIDDEN
    assert not set(limits.WRITABLE_TOPICS) & set(limits.FORBIDDEN)
    assert not set(limits.WRITABLE_SERVICES) & set(limits.FORBIDDEN)


def test_every_actuator_path_is_forbidden():
    for name in ("/wojtek/arm", "/wojtek/enable", "/wojtek/zero", "/wojtek/reset",
                 "/wojtek/joint_targets", "/sim/reset"):
        assert name in limits.FORBIDDEN
