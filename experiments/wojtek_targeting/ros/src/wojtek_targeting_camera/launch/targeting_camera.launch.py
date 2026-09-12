"""Targeting camera: a USB webcam on the RPi, published for the detector.

    ros2 launch wojtek_targeting_camera targeting_camera.launch.py
    ros2 launch wojtek_targeting_camera targeting_camera.launch.py device:=/dev/video2
    ros2 launch wojtek_targeting_camera targeting_camera.launch.py cpus:=2,3

Topics land under /<namespace>/<name>/, i.e. by default

    /targeting_camera/targeting_camera/image_raw
    /targeting_camera/targeting_camera/camera_info

The defaults are NOT camera/camera on purpose: that pair belongs to
wojtek_perception_bringup's terrain D435, which is already live on the robot,
and two drivers publishing into the same namespace is the kind of collision
that costs an afternoon to notice.

Plain node, no component container -- the same reasoning as the perception
launch. Nothing else in this graph is C++, and the detector that consumes the
image is a separate process (possibly a separate machine) by design.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "wojtek_targeting_camera"


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # Same affinity story as the perception launch: the robot bringup runs its
    # tree under taskset on the isolcpus RT cores, and nothing in this package
    # may land there -- the 400 Hz control loop owns those exclusively.
    cpus = arg("cpus")
    prefix = [f"taskset -c {cpus}"] if cpus else None

    # Single-value overrides layered on top of the parameter file. Each entry
    # of `parameters=` becomes its own --params-file in list order, so the
    # override that comes last wins. An empty argument means "whatever the file
    # says".
    overrides = {}
    for name, cast in (
        ("device", str),
        ("image_width", int),
        ("image_height", int),
        ("framerate", float),
    ):
        if arg(name):
            overrides[name] = cast(arg(name))

    return [
        Node(
            package=PKG,
            executable="usb_camera_node",
            name=arg("camera_name"),
            namespace=arg("camera_namespace"),
            parameters=[arg("camera_params_file"), overrides],
            prefix=prefix,
            output="screen",
        )
    ]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=f"{share}/config/targeting_camera.yaml",
                description="Camera settings; see the file's own commentary.",
            ),
            DeclareLaunchArgument(
                "camera_name", default_value="targeting_camera",
                description="Node name. Must not be `camera` -- that is the "
                            "terrain D435.",
            ),
            DeclareLaunchArgument(
                "camera_namespace", default_value="targeting_camera",
                description="Namespace; topics land under "
                            "/<namespace>/<name>/. Must not be `camera`.",
            ),
            DeclareLaunchArgument(
                "device", default_value="",
                description="V4L2 device. Empty = the config file's "
                            "/dev/video0. Use a /dev/v4l/by-id/... path if the "
                            "numbering is not stable.",
            ),
            DeclareLaunchArgument(
                "image_width", default_value="",
                description="Frame width. Empty = the config file.",
            ),
            DeclareLaunchArgument(
                "image_height", default_value="",
                description="Frame height. Empty = the config file.",
            ),
            DeclareLaunchArgument(
                "framerate", default_value="",
                description="Capture rate. Empty = the config file's 30.",
            ),
            DeclareLaunchArgument(
                "cpus", default_value="",
                description="CPU affinity (comma list, e.g. \"2,3\"); empty = "
                            "inherit. Set it to the non-isolated cores "
                            "whenever this runs on the robot rather than on a "
                            "bench.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
