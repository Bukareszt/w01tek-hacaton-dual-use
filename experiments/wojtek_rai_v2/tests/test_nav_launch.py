"""Exactly one cmd_vel watchdog runs per target, with the stop burst wired the
way cmd_vel_watchdog.py documents it: off in the sim, on for the real robot.
Model-free: the launch description is built and inspected, never executed."""

import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
pytest.importorskip("launch_ros")

from launch import LaunchContext  # noqa: E402
from launch.actions import ExecuteProcess  # noqa: E402
from launch.substitutions import TextSubstitution  # noqa: E402

LAUNCH_FILE = Path(__file__).resolve().parent.parent / "wojtek_rai/nav/launch/nav.launch.py"
WATCHDOG_MODULE = "wojtek_rai.nav.cmd_vel_watchdog"


def _launch_description():
    spec = importlib.util.spec_from_file_location("nav_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _plain_cmd(action):
    """The action's command as plain words, or None where any of it is a
    substitution (launch_ros Nodes always carry one; they are not watchdogs)."""
    words = []
    for part in action.cmd:
        if len(part) != 1 or not isinstance(part[0], TextSubstitution):
            return None
        words.append(part[0].text)
    return words


def _watchdogs_for(target):
    context = LaunchContext()
    context.launch_configurations["target"] = target
    out = []
    for entity in _launch_description().entities:
        if not isinstance(entity, ExecuteProcess):
            continue
        cmd = _plain_cmd(entity)
        if not cmd or WATCHDOG_MODULE not in cmd:
            continue
        if entity.condition is None or entity.condition.evaluate(context):
            out.append(cmd)
    return out


@pytest.mark.parametrize("target,burst", [("sim", "false"), ("real", "true")])
def test_one_watchdog_per_target_with_the_documented_stop_burst(target, burst):
    cmds = _watchdogs_for(target)
    assert len(cmds) == 1, cmds
    assert cmds[0][-3:] == ["--ros-args", "-p", f"stop_burst_on_reconnect:={burst}"]


def test_every_declared_target_gets_a_watchdog():
    """The watchdog is the only writer of /cmd_vel; a target with none (a new
    value, or a condition that matches neither) would let Nav2 drive with no
    dead-man at all."""
    from launch.actions import DeclareLaunchArgument

    targets = [
        a.choices
        for a in _launch_description().entities
        if isinstance(a, DeclareLaunchArgument) and a.name == "target"
    ]
    assert targets == [["sim", "real"]]
    for target in targets[0]:
        assert len(_watchdogs_for(target)) == 1
