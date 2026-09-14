"""Targeting gimbal controller.

    ros2 launch wojtek_targeting targeting.launch.py
    ros2 launch wojtek_targeting targeting.launch.py device:=/dev/ttyUSB0
    ros2 launch wojtek_targeting targeting.launch.py cpus:=2,3

Brings up the controller only. The camera is its own launch
(`wojtek_targeting_camera targeting_camera.launch.py`) and the detector is a
separate effort entirely -- this node just waits for LaserTarget messages on
/targeting/target and holds still until they arrive.

Nothing moves on startup beyond the servos holding the position they were
already in, and tracking starts DISABLED. Enabling it is an explicit call:

    ros2 service call /targeting/enable_tracking std_srvs/srv/SetBool "{data: true}"
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "wojtek_targeting"


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # Same affinity story as wojtek_perception_bringup: the robot's bringup
    # runs the walking stack under taskset on the isolcpus RT cores, and
    # nothing in this package may land there. The 400 Hz control loop owns
    # those exclusively and a serial driver spinning on them would be felt in
    # the gait.
    cpus = arg("cpus")
    prefix = [f"taskset -c {cpus}"] if cpus else None

    overrides = {}
    for name, cast in (("device", str), ("baud_rate", int), ("update_rate_hz", float)):
        if arg(name):
            overrides[name] = cast(arg(name))

    return [
        Node(
            package=PKG,
            executable="targeting_controller",
            name=arg("node_name"),
            namespace=arg("namespace"),
            parameters=[arg("params_file"), overrides],
            remappings=[("camera_info", arg("camera_info_topic"))],
            prefix=prefix,
            output="screen",
        )
    ]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{share}/config/targeting.yaml",
                description="Gimbal settings; see the file's own commentary.",
            ),
            DeclareLaunchArgument(
                "node_name", default_value="targeting_controller",
                description="Node name.",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="targeting",
                description="Namespace. Topics are namespace-relative, so this "
                            "is what puts them at /targeting/target, "
                            "/targeting/status and /targeting/gimbal_state.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/targeting_camera/targeting_camera/camera_info",
                description="Where the targeting camera publishes its "
                            "intrinsics. NOT /camera/camera/... -- that is the "
                            "walking planner's terrain D435.",
            ),
            DeclareLaunchArgument(
                "device", default_value="",
                description="U2D2 serial device. Empty = the config file's "
                            "/dev/wojtek_gimbal udev alias.",
            ),
            DeclareLaunchArgument(
                "baud_rate", default_value="",
                description="Dynamixel bus rate. Empty = the config file. Must "
                            "match how the servos are configured.",
            ),
            DeclareLaunchArgument(
                "update_rate_hz", default_value="",
                description="Control rate. Empty = the config file's 40.",
            ),
            DeclareLaunchArgument(
                "cpus", default_value="",
                description="CPU affinity (comma list, e.g. \"2,3\"); empty = "
                            "inherit. Set it to the non-isolated cores whenever "
                            "this runs on the robot rather than on a bench.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
