"""The walk/stop/posture tools against a fake connector. No ROS runtime, no LLM.

Needs the rai package (runs inside the wojtek_rai container, or any interpreter
with rai-core); skipped elsewhere.
"""

import pytest

rai = pytest.importorskip("rai")

from wojtek_rai import limits  # noqa: E402
from wojtek_rai.tools import (  # noqa: E402
    STRING_MSG,
    TRIGGER_SRV,
    LieDownTool,
    StandUpTool,
    StopTool,
    WalkTool,
    _permissions,
)


def make_fake_connector():
    """A Mock that passes pydantic's isinstance check and records what tools send."""
    from unittest.mock import MagicMock

    from rai.communication.ros2 import ROS2Connector
    from rai.communication.ros2.messages import ROS2Message

    fake = MagicMock(spec=ROS2Connector)
    fake.published = []
    fake.services = []

    def send_message(message, target, *, msg_type, **_):
        fake.published.append((target, msg_type, dict(message.payload)))

    class Response:
        success = True
        message = "ok"

    def service_call(message, target, timeout_sec=5.0, *, msg_type, **_):
        fake.services.append((target, msg_type, dict(message.payload), timeout_sec))
        return ROS2Message(payload=Response())

    fake.send_message.side_effect = send_message
    fake.service_call.side_effect = service_call
    # The text-command tools publish through one rclpy publisher on the node.
    fake.node.create_publisher.return_value.publish.side_effect = lambda m: fake.published.append(
        (limits.NAV_COMMAND_TOPIC, STRING_MSG, {"data": m.data})
    )
    return fake


@pytest.fixture
def connector():
    return make_fake_connector()


@pytest.fixture(autouse=True)
def no_sleep(monkeypatch):
    slept = []
    monkeypatch.setattr("wojtek_rai.tools.time.sleep", lambda s: slept.append(s) if s != 0.3 else None)
    # monotonic advances by whatever was "slept", so the loop terminates.
    clock = {"t": 0.0}

    def monotonic():
        clock["t"] += sum(slept)
        slept.clear()
        return clock["t"]

    monkeypatch.setattr("wojtek_rai.tools.time.monotonic", monotonic)
    return slept


def _walk(connector, **kw):
    return WalkTool(connector=connector, **_permissions(), **kw)


def test_walk_pulses_the_command_then_stops(connector):
    out = _walk(connector)._run(direction="forward", seconds=1.0)

    targets = {t for t, _, _ in connector.published}
    assert targets == {limits.NAV_COMMAND_TOPIC}
    assert all(m == STRING_MSG for _, m, _ in connector.published)
    commands = [p["data"] for _, _, p in connector.published]
    assert commands[-1] == limits.STOP_COMMAND
    assert set(commands[:-1]) == {"forward"}
    # 1.0 s at a 0.5 s republish period: pulses at t=0, 0.5 and 1.0, then stop.
    assert len(commands) == 4
    assert "stopped" in out


def test_walk_rejects_bad_arguments_before_publishing(connector):
    with pytest.raises(ValueError):
        _walk(connector)._run(direction="backward", seconds=1.0)
    with pytest.raises(ValueError):
        _walk(connector)._run(direction="forward", seconds=limits.MOVE_MAX_SECONDS + 1)
    assert connector.published == []


def test_walk_refuses_when_nav_command_is_not_writable(connector):
    tool = WalkTool(connector=connector, writable=[], forbidden=list(limits.FORBIDDEN))
    with pytest.raises(ValueError, match="not writable"):
        tool._run(direction="forward", seconds=1.0)
    assert connector.published == []


def test_stop_sends_a_single_stop(connector):
    StopTool(connector=connector, **_permissions())._run()
    assert connector.published == [
        (limits.NAV_COMMAND_TOPIC, STRING_MSG, {"data": limits.STOP_COMMAND})
    ]


def test_posture_tools_call_only_their_trigger_service(connector):
    StandUpTool(connector=connector, **_permissions())._run()
    LieDownTool(connector=connector, **_permissions())._run()
    assert [(s, m, p) for s, m, p, _ in connector.services] == [
        (limits.STAND_UP_SERVICE, TRIGGER_SRV, {}),
        (limits.LIE_DOWN_SERVICE, TRIGGER_SRV, {}),
    ]
    assert connector.published == []


def test_forbidden_names_are_never_writable(connector):
    tool = _walk(connector)
    for name in limits.FORBIDDEN:
        assert not tool.is_writable(name)
        assert not tool.is_readable(name)


# --- the tool set per target (sim with Nav2 vs the physical robot) ------------


def _names(tools):
    return {t.name for t in tools}


def test_without_odometry_no_tool_can_reach_nav2(connector):
    """WOJTEK_RAI_ODOMETRY=0 (the physical robot): Nav2 is not running and has
    nothing to track the robot with, so no tool may send it a goal. Looking
    (find_objects, camera) and text-command walking stay."""
    from wojtek_rai.tools import build_tools

    names = _names(build_tools(connector, odometry=False))
    assert {"walk", "stop", "stand_up", "lie_down", "find_objects", "get_camera_image"} <= names
    nav2 = {"navigate_to_pose", "go_to_place", "go_to_object", "cancel_navigation", "turn", "get_map_pose", "get_map_image"}
    assert names.isdisjoint(nav2)


def test_with_odometry_the_nav2_tools_are_offered(connector):
    from wojtek_rai.tools import build_tools

    names = _names(build_tools(connector, odometry=True))
    assert {"turn", "navigate_to_pose", "go_to_place", "go_to_object", "find_objects"} <= names


def test_read_only_offers_nothing_that_moves(connector):
    from wojtek_rai.tools import build_tools

    names = _names(build_tools(connector, read_only=True))
    assert "get_camera_image" in names
    movers = {"walk", "stop", "stand_up", "lie_down", "turn", "navigate_to_pose", "go_to_object", "go_to_place"}
    assert names.isdisjoint(movers)


# --- subscriber discovery over WiFi ------------------------------------------


def test_walk_waits_for_text_commander_to_discover_the_publisher(connector):
    """On the physical robot text_commander's subscription becomes visible
    ~2 s after the publisher exists (measured over the WiFi AP); the tool must
    wait for it instead of refusing after a fixed 0.3 s."""
    pub = connector.node.create_publisher.return_value
    pub.get_subscription_count.side_effect = [0, 0, 0, 0, 0, 0, 0, 0, 1] + [1] * 100

    out = _walk(connector)._run(direction="forward", seconds=1.0)

    assert "walked forward" in out or "forward" in out
    assert connector.published[0][2] == {"data": "forward"}
    assert connector.published[-1][2] == {"data": "stop"}


def test_walk_refuses_when_nothing_subscribes_after_the_wait(connector):
    pub = connector.node.create_publisher.return_value
    pub.get_subscription_count.return_value = 0

    with pytest.raises(ValueError, match="nothing subscribes"):
        _walk(connector)._run(direction="forward", seconds=1.0)
    assert connector.published == []


# --- stop must beat Nav2, and a walk must always end in a stop ---------------


def test_stop_cancels_the_nav2_goal_before_publishing(connector, monkeypatch):
    """A text stop alone loses: Nav2 keeps streaming /cmd_vel at 20 Hz through
    the watchdog and overwrites text_commander's zero within 50 ms. The goal
    has to be cancelled, and cancelled first."""
    from wojtek_rai import nav_tools
    from wojtek_rai.tools import StopTool

    order = []
    monkeypatch.setattr(nav_tools, "cancel_active_nav_goal", lambda: order.append("cancel") or True)
    pub = connector.node.create_publisher.return_value
    pub.publish.side_effect = lambda m: order.append(f"publish:{m.data}")

    out = StopTool(connector=connector, **_permissions())._run()

    assert order == ["cancel", f"publish:{limits.STOP_COMMAND}"]
    assert "cancelled" in out


def test_stop_still_works_where_the_nav2_tools_are_not_built(connector, monkeypatch):
    """WOJTEK_RAI_ODOMETRY=0 (the physical robot): nav_tools is not part of the
    tool set, and `stop` must not disappear or raise with it."""
    import builtins

    from wojtek_rai.tools import StopTool

    real_import = builtins.__import__

    def no_nav_tools(name, *args, **kwargs):
        if name == "wojtek_rai.nav_tools":
            raise ImportError("no Nav2 tools on this target")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_nav_tools)

    out = StopTool(connector=connector, **_permissions())._run()

    assert out == "Stop command sent."
    assert connector.published == [
        (limits.NAV_COMMAND_TOPIC, STRING_MSG, {"data": limits.STOP_COMMAND})
    ]


def test_a_failing_send_mid_walk_still_stops_the_robot(connector):
    """The publish can fail mid-walk (link drop, subscriber gone). Without the
    closing stop the robot would walk on until text_commander's 2 s dead-man.
    TurnTool has had this finally; WalkTool now matches it."""
    calls = {"n": 0}

    def publish(msg):
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("publisher lost the link")
        connector.published.append((limits.NAV_COMMAND_TOPIC, STRING_MSG, {"data": msg.data}))

    connector.node.create_publisher.return_value.publish.side_effect = publish

    with pytest.raises(RuntimeError, match="lost the link"):
        _walk(connector)._run(direction="forward", seconds=5.0)

    commands = [p["data"] for _, _, p in connector.published]
    assert commands == ["forward", limits.STOP_COMMAND]
