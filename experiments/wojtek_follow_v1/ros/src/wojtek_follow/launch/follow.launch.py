"""The tower-camera follow node.

    ros2 launch wojtek_follow follow.launch.py
    ros2 launch wojtek_follow follow.launch.py params_file:=/path/to/follow.yaml
    ros2 launch wojtek_follow follow.launch.py cpus:=0,1

Brings up the follow node only. The gimbal is its own launch
(`wojtek_targeting targeting.launch.py`), the cameras are theirs, and the Deck
gateway is the robot's own bringup. This node does nothing at all until a track
arrives on /wojtek/track/target, which means until someone taps a target on the
Deck page.

Every topic is a parameter with an absolute default, so the namespace argument
moves the node and its parameters without moving what it talks to.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "wojtek_follow"


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory(PKG)

    # The robot's bringup runs the walking stack under taskset on the isolcpus
    # real-time cores. Nothing in this package may land there: the 400 Hz
    # control loop owns those, and a numpy pass over a depth image on one of
    # them would be felt in the gait.
    cpus = arg("cpus")
    prefix = [f"taskset -c {cpus}"] if cpus else None

    overrides = {}
    for name, cast in (("pan_sign", float), ("tilt_sign", float),
                       ("tower_camera_info_topic", str)):
        if arg(name):
            overrides[name] = cast(arg(name))
    if arg("gimbal_fixed").lower() in ("true", "1", "yes"):
        overrides["gimbal_fixed"] = True

    return [
        Node(
            package=PKG,
            executable="follow_node",
            name=arg("node_name"),
            namespace=arg("namespace"),
            # follow.yaml is always the base; params_file lays its own
            # values over it, so a preset like follow_sim.yaml lists only
            # what differs.
            parameters=[f"{share}/config/follow.yaml", arg("params_file"), overrides],
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
                default_value=f"{share}/config/follow.yaml",
                description="Follow settings; see the file's own commentary.",
            ),
            DeclareLaunchArgument(
                "node_name", default_value="wojtek_follow",
                description="Node name.",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="",
                description="Namespace for the node itself. The topics are "
                            "absolute parameters, so this does not move them.",
            ),
            DeclareLaunchArgument(
                "pan_sign", default_value="",
                description="Override the config's pan sign. +1 means a "
                            "positive pan angle turns the tower left. The "
                            "config's -1 is the negated pan_direction of the "
                            "targeting controller. Verify on the bench "
                            "before the robot walks.",
            ),
            DeclareLaunchArgument(
                "tilt_sign", default_value="",
                description="Override the config's tilt sign. +1 means a "
                            "positive tilt angle points the tower up. The "
                            "config's +1 is the negated tilt_direction of "
                            "the targeting controller.",
            ),
            DeclareLaunchArgument(
                "gimbal_fixed", default_value="false",
                description="true: no gimbal at all, pan and tilt are zero, "
                            "nothing is sent to the targeting side. The "
                            "simulation's setting.",
            ),
            DeclareLaunchArgument(
                "tower_camera_info_topic", default_value="",
                description="Override the config's tower camera_info topic. "
                            "In simulation: /camera/camera/color/camera_info.",
            ),
            DeclareLaunchArgument(
                "cpus", default_value="",
                description="CPU affinity (comma list, e.g. \"0,1\"); empty = "
                            "inherit. Set it to the non-isolated cores whenever "
                            "this runs on the robot rather than on a bench.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
