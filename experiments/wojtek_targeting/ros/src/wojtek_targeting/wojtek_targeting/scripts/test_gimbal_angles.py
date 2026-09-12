#!/usr/bin/env python3
"""Interactive CLI for testing gimbal pan/tilt angles.

Usage:
    ros2 run wojtek_targeting test_gimbal_angles.py

Once running, type commands at the prompt:
    pan_deg tilt_deg   (e.g., "30 -10" sets pan=30, tilt=-10)
    help                (show usage)
    status              (print current gimbal state)
    quit                (exit)

The script calls /targeting/set_manual_angles service to move the gimbal
and reports the result. Use this to test motor rotations before full tracking.
"""

import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from wojtek_targeting_msgs.srv import SetGimbalAngles


class GimbalTestClient(Node):
    def __init__(self):
        super().__init__("gimbal_test_client")
        self.cli = self.create_client(SetGimbalAngles, "/targeting/set_manual_angles")
        self.gimbal_state_sub = self.create_subscription(
            JointState, "/targeting/gimbal_state", self._gimbal_state_callback, 10
        )
        self.last_state = None
        self.logger.info("Gimbal test client ready. Type 'help' for commands.")

    def _gimbal_state_callback(self, msg):
        self.last_state = msg

    def set_angles(self, pan_deg, tilt_deg):
        """Call the set_manual_angles service."""
        if not self.cli.wait_for_service(timeout_sec=1.0):
            print("ERROR: /targeting/set_manual_angles service not available")
            return False

        request = SetGimbalAngles.Request()
        request.pan_deg = float(pan_deg)
        request.tilt_deg = float(tilt_deg)

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            resp = future.result()
            status = "✓ SUCCESS" if resp.success else "✗ FAILED"
            print(f"\n{status}: pan={pan_deg:.1f}° tilt={tilt_deg:.1f}°")
            if resp.message:
                print(f"Message: {resp.message}")
            self._print_state()
            return resp.success
        else:
            print("ERROR: Service call failed")
            return False

    def _print_state(self):
        """Print the last gimbal state."""
        if self.last_state is not None:
            pan_rad = self.last_state.position[0]
            tilt_rad = self.last_state.position[1]
            pan_deg = math.degrees(pan_rad)
            tilt_deg = math.degrees(tilt_rad)
            print(f"Current gimbal state: pan={pan_deg:+.1f}° tilt={tilt_deg:+.1f}°")
        else:
            print("(no gimbal state received yet)")

    def run_interactive(self):
        """Run the interactive command loop."""
        print("\n" + "=" * 60)
        print("GIMBAL MANUAL TEST CLIENT")
        print("=" * 60)
        print("Commands:")
        print("  <pan_deg> <tilt_deg>  Set gimbal angles (e.g., '30 -10')")
        print("  status                Print current gimbal state")
        print("  help                  Show this message")
        print("  quit                  Exit")
        print("=" * 60 + "\n")

        while True:
            try:
                user_input = input("gimbal> ").strip()
            except EOFError:
                print("\nEOF received, exiting.")
                break
            except KeyboardInterrupt:
                print("\n\nKeyboard interrupt, exiting.")
                break

            if not user_input:
                continue

            if user_input.lower() == "quit" or user_input.lower() == "exit":
                print("Exiting gimbal test client.")
                break

            if user_input.lower() == "help":
                print(
                    "Commands: <pan_deg> <tilt_deg> | status | help | quit\n"
                )
                continue

            if user_input.lower() == "status":
                self._print_state()
                continue

            try:
                parts = user_input.split()
                if len(parts) != 2:
                    print("ERROR: Expected two angles (pan_deg tilt_deg)")
                    continue
                pan_deg = float(parts[0])
                tilt_deg = float(parts[1])
                self.set_angles(pan_deg, tilt_deg)
            except ValueError:
                print("ERROR: Could not parse angles. Expected two floats.")
                continue


def main(args=None):
    rclpy.init(args=args)
    client = GimbalTestClient()

    try:
        client.run_interactive()
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
