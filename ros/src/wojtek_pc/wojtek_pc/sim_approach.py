"""Walk the simulated robot up to a point on the quay, scripted.

    ros2 run wojtek_pc sim_approach --x 6.0 --y -4.2 --standoff 1.2

A demo helper and nothing more. It reads the plant's ground-truth pose from
/sim/qpos (the free joint: x, y, z, then the quaternion), turns the body
toward the target, walks at it, and stops one standoff short, facing it.
Everything goes out on /cmd_vel, the same surface the pad, the consoles and
text_commander use, and the last message is a zero Twist because the policy
latches whatever it heard last.

It is open-loop about everything but the pose: no obstacle check, no depth,
no detector. The point is a robot that visibly turns, walks and stops at a
person on the deck panel's picture while the panel's own boxes name them.
Nothing here exists on the physical robot, which has no /sim/qpos.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


def yaw_of(qw, qx, qy, qz):
    """Heading about +z of a w-first quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Approach(Node):
    def __init__(self, args):
        super().__init__("sim_approach")
        self.a = args
        self.pose = None
        self.state = "WAIT"
        self.t0 = time.monotonic()
        self.create_subscription(
            Float64MultiArray, "/sim/qpos", self._on_qpos, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / args.hz, self._tick)

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
        cmd = Twist()
        if self.pose is None:
            self._say("WAIT", "for /sim/qpos")
            return
        if time.monotonic() - self.t0 > self.a.timeout:
            self._say("TIMEOUT", "stopping")
            self.pub.publish(cmd)
            raise SystemExit(1)
        x, y, yaw = self.pose
        dx, dy = self.a.x - x, self.a.y - y
        dist = math.hypot(dx, dy)
        err = wrap(math.atan2(dy, dx) - yaw)
        if dist <= self.a.standoff and abs(err) < self.a.face_tol:
            self._say("ARRIVED", f"{dist:.2f} m from the target")
            self.pub.publish(cmd)
            raise SystemExit(0)
        if abs(err) > self.a.align_tol and self.state != "APPROACH":
            self._say("ALIGN", f"bearing error {math.degrees(err):+.0f} deg")
            # The gait barely turns below ~0.2 rad/s, so ask for at least
            # yaw_min, in the right direction.
            cmd.angular.z = math.copysign(max(self.a.yaw_min, min(self.a.yaw_max, abs(err))), err)
        elif dist > self.a.standoff:
            self._say("APPROACH", f"{dist:.1f} m to go")
            cmd.linear.x = self.a.speed
            # Steer while walking, gently; a strong turn stalls the walk.
            cmd.angular.z = max(-self.a.yaw_max, min(self.a.yaw_max, 1.2 * err))
            if abs(err) > self.a.align_tol * 3:
                self.state = "REALIGN"  # go back through ALIGN next tick
        else:
            self._say("FACE", f"bearing error {math.degrees(err):+.0f} deg")
            cmd.angular.z = math.copysign(max(self.a.yaw_min, min(self.a.yaw_max, abs(err))), err)
        self.pub.publish(cmd)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--x", type=float, required=True, help="target x in the plant's world frame")
    p.add_argument("--y", type=float, required=True, help="target y")
    p.add_argument("--standoff", type=float, default=1.2, help="stop this far short of the target, m")
    p.add_argument("--speed", type=float, default=0.25, help="walking speed, m/s")
    p.add_argument("--yaw-min", type=float, default=0.35, help="smallest yaw rate the gait answers, rad/s")
    p.add_argument("--yaw-max", type=float, default=0.6, help="largest yaw rate asked for, rad/s")
    p.add_argument("--align-tol", type=float, default=0.12, help="heading error that counts as aligned, rad")
    p.add_argument("--face-tol", type=float, default=0.2, help="heading error accepted on arrival, rad")
    p.add_argument("--hz", type=float, default=20.0, help="/cmd_vel rate")
    p.add_argument("--timeout", type=float, default=120.0, help="give up and stop after this many seconds")
    args = p.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    rclpy.init(args=argv)
    node = Approach(args)
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
