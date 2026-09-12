"""The /cmd_vel dead-man in policy_node: with cmd_vel_timeout_s on, a dead
publisher must not keep Wojtek walking.

The robot-side drive sources (pad, deck gateway, consoles, text commander)
stream at 20 Hz and zero before they go quiet, so with the timeout on the
only way the node sees a gap is that the source died mid-drive -- a crashed
Nav2, a dropped link, a closed tab. Then the command has to decay instead
of latching. Off (the default, because teleop_twist_keyboard and the
Foxglove Teleop panel publish per keypress) the node latches as it always
did.

The real node on a fake clock: Twists go straight into the callback, time
is moved by hand, and the command the policy would be stepped with is read
back. The fake clock means the ROS-time (use_sim_time) path is not what is
exercised here; only that the node asks its clock, whichever it is. Needs
rclpy (skipped on a host without ROS); run in the dev container:

    pytest ros/src/wojtek_policy/test/test_policy_node_cmd_vel.py
"""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

# Never let a test node reach the robot's graph: own domain, this host only.
os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.time import Time  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_policy import LIVE_HEIGHT_META, make_policy  # noqa: E402

TEST_DOMAIN = 88
TIMEOUT = 0.5   # cmd_vel_timeout_s under test
# A live standing-height contract: its command_fill (the height a 3-D
# command falls back to) differs from the height driven below, which is how
# "the set-point survived the timeout" is told apart from "the command was
# rebuilt from the contract default".
HEIGHT_DEFAULT = LIVE_HEIGHT_META["command_fill"][0]
HEIGHT_DRIVEN = 0.15


class FakeClock:
    """The node clock, moved by hand. `now()` is all policy_node asks for."""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return Time(nanoseconds=int(self.t * 1e9))


@pytest.fixture(scope="module")
def policy_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("policy")
    make_policy(d, meta_updates=LIVE_HEIGHT_META)  # writes npz + meta json
    return d


@pytest.fixture(scope="module", autouse=True)
def _ros(policy_dir):
    # The node reads its policy from a parameter, so it has to be set before
    # the node exists; a global override is the only pre-node hook there is.
    rclpy.init(
        domain_id=TEST_DOMAIN,
        args=["--ros-args", "-p", f"policy:={policy_dir}"],
    )
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    from wojtek_policy.policy_node import PolicyNode

    n = PolicyNode()
    n.set_parameters(
        [Parameter("cmd_vel_timeout_s", Parameter.Type.DOUBLE, TIMEOUT)]
    )
    n.clock = FakeClock()
    n.get_clock = lambda: n.clock
    yield n
    n.destroy_node()


def drive(node, vx=0.0, vy=0.0, yaw=0.0, height=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, height
    msg.angular.z = yaw
    node._on_cmd(msg)


def test_fresh_command_passes_through(node):
    drive(node, vx=0.3, yaw=0.2, height=HEIGHT_DRIVEN)
    node.clock.t += TIMEOUT * 0.9
    assert np.allclose(node._command(), [0.3, 0.0, 0.2, HEIGHT_DRIVEN])


def test_stale_command_stands_in_place(node):
    drive(node, vx=0.3, vy=-0.2, yaw=0.2, height=HEIGHT_DRIVEN)
    node.clock.t += TIMEOUT * 1.1
    cmd = node._command()
    assert np.allclose(cmd[:3], 0.0)
    # The standing height is a held set-point, not a velocity: the stance
    # must not jump when the driver goes away.
    assert cmd[3] == pytest.approx(HEIGHT_DRIVEN)
    # The latched command itself is untouched -- a resumed stream drives
    # again without any recovery step.
    assert np.allclose(node._cmd, [0.3, -0.2, 0.2, HEIGHT_DRIVEN])


def test_a_resumed_stream_drives_again(node):
    drive(node, vx=0.3)
    node.clock.t += TIMEOUT * 2
    assert np.allclose(node._command()[:3], 0.0)
    drive(node, vx=0.25)
    assert np.allclose(node._command(), [0.25, 0.0, 0.0, HEIGHT_DEFAULT])


def test_zero_timeout_latches_forever(node):
    """0 = the behaviour before this dead-man existed, byte for byte."""
    node.set_parameters(
        [Parameter("cmd_vel_timeout_s", Parameter.Type.DOUBLE, 0.0)]
    )
    drive(node, vx=0.3, yaw=0.2)
    node.clock.t += 3600.0
    assert np.allclose(node._command(), [0.3, 0.0, 0.2, HEIGHT_DEFAULT])


def test_the_default_is_off():
    """A node nobody configured latches: teleop_twist_keyboard publishes
    one Twist per keypress and would otherwise turn into a pulse per key.
    The nav stack opts in with cmd_vel_timeout_s:=0.5."""
    from wojtek_policy.policy_node import PolicyNode

    n = PolicyNode()
    try:
        assert n.get_parameter("cmd_vel_timeout_s").value == 0.0
        n.clock = FakeClock()
        n.get_clock = lambda: n.clock
        drive(n, vx=0.3)
        n.clock.t += 3600.0
        assert np.allclose(n._command(), [0.3, 0.0, 0.0, HEIGHT_DEFAULT])
    finally:
        n.destroy_node()


def test_a_dead_driver_is_logged_once_per_episode(node):
    """One warning per stale episode, and only when the robot was moving:
    the gates zero before they go quiet, so a stick release must not fill
    the journal at the tick rate."""
    warnings = []

    class Spy:
        def warning(self, msg, *a, **k):
            warnings.append(msg)

    node.get_logger = lambda: Spy()

    drive(node, vx=0.3)
    node.clock.t += TIMEOUT * 1.1
    for _ in range(5):
        node._command()
    assert len(warnings) == 1

    # The stream comes back and dies again: a second episode, a second line.
    drive(node, vx=0.3)
    node._command()
    node.clock.t += TIMEOUT * 1.1
    for _ in range(5):
        node._command()
    assert len(warnings) == 2

    # A driver that zeroed before going quiet is the normal case: silent.
    drive(node, vx=0.0)
    node._command()
    node.clock.t += TIMEOUT * 1.1
    node._command()
    assert len(warnings) == 2


def test_no_command_at_all_is_no_motion(node):
    """A node nobody has ever driven stands there; it must not trip on the
    missing stamp either."""
    node.clock.t += 10.0
    assert np.allclose(node._command(), [0.0, 0.0, 0.0, HEIGHT_DEFAULT])


def test_tick_steps_the_policy_with_the_decayed_command(node):
    """The wiring, not just the accessor: what reaches policy.step is the
    zeroed command."""
    stepped = []
    real_step = node.policy.step

    def spy(gyro, gravity, q, dq, cmd):
        stepped.append(np.asarray(cmd, dtype=float).copy())
        return real_step(gyro, gravity, q, dq, cmd)

    node.policy.step = spy
    node._pub.publish = lambda msg: None
    node._q_urdf = np.array(node.policy.home_ctrl)
    node._dq_urdf = np.zeros(12)

    drive(node, vx=0.3, yaw=0.2, height=HEIGHT_DRIVEN)
    node._joints_stamp = node.clock.now()
    node._tick()
    assert np.allclose(stepped[-1], [0.3, 0.0, 0.2, HEIGHT_DRIVEN])

    # Sensors keep arriving, the driver dies: the policy keeps running (it
    # must, the robot is standing on it) on a zero command.
    node.clock.t += TIMEOUT * 1.1
    node._joints_stamp = node.clock.now()
    node._tick()
    assert np.allclose(stepped[-1], [0.0, 0.0, 0.0, HEIGHT_DRIVEN])
