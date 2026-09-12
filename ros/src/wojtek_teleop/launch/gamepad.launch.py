"""Bluetooth Xbox pad teleop: the standard `joy` driver + gamepad_teleop.

    ros2 launch wojtek_teleop gamepad.launch.py [device_id:=0]

Pair the pad over bluetooth on the machine this runs on first (bluetoothctl:
scan on -> pair -> trust -> connect; the pad must show up under /dev/input).
On the RPi the bluez/ERTM groundwork comes from deploy/rpi/install.sh. Runs
unchanged against the sim or the real stack -- it publishes the same
/cmd_vel the consoles do. `sim.launch.py gamepad:=true` brings this up next to
the web console; see wojtek_teleop/gamepad_teleop.py for the stick mapping.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Cores for the two nodes, as a taskset list; empty = wherever the
    # parent runs. The robot service starts its tree on the isolated RT
    # cores, and with load balancing off there these two landed on the
    # same core as policy_node and real_io and took a fifth of it. They
    # are not on the control path, so the service sends them to 0,1.
    cpus = LaunchConfiguration("gamepad_cpus")
    prefix = PythonExpression(["'taskset -c ", cpus, "' if '", cpus, "' else ''"])
    return LaunchDescription(
        [
            # Which /dev/input device the joy driver opens (0 = first pad).
            DeclareLaunchArgument("device_id", default_value="0"),
            # Policy reference whose contract sets the stick command box
            # (same reference policy_node gets); empty = the teleop node's
            # conservative default limits.
            DeclareLaunchArgument("policy", default_value=""),
            DeclareLaunchArgument("gamepad_cpus", default_value=""),
            Node(
                package="joy",
                executable="joy_node",
                output="screen",
                prefix=prefix,
                parameters=[
                    {
                        "device_id": ParameterValue(
                            LaunchConfiguration("device_id"), value_type=int
                        ),
                        # Keep publishing while sticks are held steady --
                        # gamepad_teleop's dead-man counts on a steady stream
                        # and must only trip when the pad actually drops off.
                        "autorepeat_rate": 20.0,
                        "deadzone": 0.05,
                    }
                ],
            ),
            Node(
                package="wojtek_teleop",
                executable="gamepad_teleop",
                output="screen",
                prefix=prefix,
                parameters=[{"policy": LaunchConfiguration("policy")}],
            ),
        ]
    )
