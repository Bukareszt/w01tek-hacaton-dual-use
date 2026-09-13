"""Walk the simulated robot up to a point on the quay, scripted.

One-shot, from a shell:

    ros2 run wojtek_pc sim_approach --x 6.0 --y -4.2 --standoff 1.2

Resident, as the sim launch runs it, answering the deck panel's button:

    ros2 run wojtek_pc sim_approach --serve --ros-args \\
        -p targets:='[{"name": "intruder_1", "x": 9.5, "y": 2.6}]'

A demo helper and nothing more. It reads the plant's ground-truth pose from
/sim/qpos (the free joint: x, y, z, then the quaternion), turns the body
toward the target, walks at it, and stops one standoff short, facing it.
Everything goes out on /cmd_vel, the same surface the pad, the consoles and
text_commander use, and the last message of a run is a zero Twist because
the policy latches whatever it heard last. Between runs it publishes
nothing, so the pad keeps the wheel.

In `--serve` mode two Trigger services carry the panel's buttons:
/wojtek/intercept walks to the nearest of the configured targets, and
/wojtek/intercept_stop ends the run where the robot stands. The deck
gateway lists both among its trigger services; on the physical robot they
do not exist, so the panel's buttons stay disabled there.

It is open-loop about everything but the pose: no obstacle check, no depth,
no detector. The point is a robot that visibly turns, walks and stops at a
person on the deck panel's picture while the panel's own boxes name them.
Nothing here exists on the physical robot, which has no /sim/qpos.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


def yaw_of(qw, qx, qy, qz):
    """Heading about +z of a w-first quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def nearest(targets, x, y):
    """The target closest to (x, y), or None when there is none."""
    best = None
    for t in targets:
        d = math.hypot(t["x"] - x, t["y"] - y)
        if best is None or d < best[0]:
            best = (d, t)
    return None if best is None else best[1]


class Gains:
    """The walk's numbers, shared by both modes."""

    def __init__(self, standoff=1.2, speed=0.25, yaw_min=0.35, yaw_max=0.6,
                 align_tol=0.12, face_tol=0.2, timeout=120.0):
        self.standoff, self.speed = standoff, speed
        self.yaw_min, self.yaw_max = yaw_min, yaw_max
        self.align_tol, self.face_tol = align_tol, face_tol
        self.timeout = timeout


class Approach(Node):
    def __init__(self, gains, hz=20.0, targets=(), serve=False):
        super().__init__("sim_approach")
        self.g = gains
        self.pose = None
        self.goal = None          # (x, y, name) while a run is on
        self.state = "IDLE"
        self.t0 = 0.0
        self.serve = serve
        self.targets = list(targets)
        self.create_subscription(
            Float64MultiArray, "/sim/qpos", self._on_qpos, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / hz, self._tick)
        if serve:
            self.create_service(Trigger, "wojtek/intercept", self._srv_intercept)
            self.create_service(Trigger, "wojtek/intercept_stop", self._srv_stop)
            names = ", ".join(t["name"] for t in self.targets) or "none"
            self.get_logger().info(f"serving /wojtek/intercept; targets: {names}")

    # ---- goals ----------------------------------------------------------
    def start(self, x, y, name="target"):
        self.goal = (x, y, name)
        self.t0 = time.monotonic()
        self.state = "START"

    def stop(self, why):
        if self.goal is not None:
            self.get_logger().info(f"{why}")
        self.goal = None
        self.state = "IDLE"
        for _ in range(3):
            self.pub.publish(Twist())

    def _srv_intercept(self, _req, res):
        if self.pose is None:
            res.success, res.message = False, "no /sim/qpos yet"
            return res
        t = nearest(self.targets, self.pose[0], self.pose[1])
        if t is None:
            res.success, res.message = False, "no targets configured"
            return res
        self.start(t["x"], t["y"], t["name"])
        d = math.hypot(t["x"] - self.pose[0], t["y"] - self.pose[1])
        res.success, res.message = True, f"walking to {t['name']}, {d:.1f} m"
        return res

    def _srv_stop(self, _req, res):
        was = self.goal
        self.stop("STOPPED by the panel")
        res.success = True
        res.message = f"stopped short of {was[2]}" if was else "nothing to stop"
        return res

    # ---- the loop -------------------------------------------------------
    def _on_qpos(self, msg):
        d = msg.data
        if len(d) < 7:
            return
        self.pose = (d[0], d[1], yaw_of(d[3], d[4], d[5], d[6]))

    def _say(self, state, note=""):
        if state != self.state:
            self.state = state
            self.get_logger().info(f"{state} {note}".strip())

    def _tick(self):
        if self.goal is None:
            return
        if self.pose is None:
            self._say("WAIT", "for /sim/qpos")
            return
        g = self.g
        if time.monotonic() - self.t0 > g.timeout:
            self.stop("TIMEOUT, stopping")
            if not self.serve:
                raise SystemExit(1)
            return
        x, y, yaw = self.pose
        tx, ty, name = self.goal
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        err = wrap(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        if dist <= g.standoff and abs(err) < g.face_tol:
            self.stop(f"ARRIVED {dist:.2f} m from {name}")
            if not self.serve:
                raise SystemExit(0)
            return
        if abs(err) > g.align_tol and self.state != "APPROACH":
            self._say("ALIGN", f"bearing error {math.degrees(err):+.0f} deg")
            # The gait barely turns below ~0.2 rad/s, so ask for at least
            # yaw_min, in the right direction.
            cmd.angular.z = math.copysign(max(g.yaw_min, min(g.yaw_max, abs(err))), err)
        elif dist > g.standoff:
            self._say("APPROACH", f"{dist:.1f} m to {name}")
            cmd.linear.x = g.speed
            # Steer while walking, gently; a strong turn stalls the walk.
            cmd.angular.z = max(-g.yaw_max, min(g.yaw_max, 1.2 * err))
            if abs(err) > g.align_tol * 3:
                self.state = "REALIGN"  # go back through ALIGN next tick
        else:
            self._say("FACE", f"bearing error {math.degrees(err):+.0f} deg")
            cmd.angular.z = math.copysign(max(g.yaw_min, min(g.yaw_max, abs(err))), err)
        self.pub.publish(cmd)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--serve", action="store_true",
                   help="stay resident and answer /wojtek/intercept; targets come from the "
                        "`targets` parameter (a JSON list of {name, x, y})")
    p.add_argument("--x", type=float, help="one-shot target x in the plant's world frame")
    p.add_argument("--y", type=float, help="one-shot target y")
    p.add_argument("--standoff", type=float, default=None, help="stop this far short of the target, m")
    p.add_argument("--speed", type=float, default=None, help="walking speed, m/s")
    p.add_argument("--yaw-min", type=float, default=0.35, help="smallest yaw rate the gait answers, rad/s")
    p.add_argument("--yaw-max", type=float, default=0.6, help="largest yaw rate asked for, rad/s")
    p.add_argument("--align-tol", type=float, default=0.12, help="heading error that counts as aligned, rad")
    p.add_argument("--face-tol", type=float, default=0.2, help="heading error accepted on arrival, rad")
    p.add_argument("--hz", type=float, default=20.0, help="/cmd_vel rate")
    p.add_argument("--timeout", type=float, default=120.0, help="give up and stop after this many seconds")
    args = p.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])
    if not args.serve and (args.x is None or args.y is None):
        p.error("--x and --y are required unless --serve")

    rclpy.init(args=argv)
    gains = Gains(yaw_min=args.yaw_min, yaw_max=args.yaw_max, align_tol=args.align_tol,
                  face_tol=args.face_tol, timeout=args.timeout)
    targets = ()
    if args.serve:
        # Parameters only exist once a node does; a throwaway node reads them.
        probe = Node("sim_approach_params")
        probe.declare_parameter("targets", "[]")
        probe.declare_parameter("standoff", 3.5)
        probe.declare_parameter("speed", 0.4)
        targets = json.loads(probe.get_parameter("targets").value)
        gains.standoff = probe.get_parameter("standoff").value
        gains.speed = probe.get_parameter("speed").value
        probe.destroy_node()
    if args.standoff is not None:
        gains.standoff = args.standoff
    elif not args.serve:
        gains.standoff = 1.2
    if args.speed is not None:
        gains.speed = args.speed
    elif not args.serve:
        gains.speed = 0.25

    node = Approach(gains, hz=args.hz, targets=targets, serve=args.serve)
    if not args.serve:
        node.start(args.x, args.y)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = 130
    finally:
        # The policy latches the last command: the last word is always zero.
        for _ in range(5):
            node.pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
