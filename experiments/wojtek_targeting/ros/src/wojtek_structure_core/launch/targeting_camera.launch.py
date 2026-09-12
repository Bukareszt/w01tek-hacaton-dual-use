"""Targeting camera: Occipital Structure Core (ST02D-C) colour + depth.

    ros2 launch wojtek_structure_core targeting_camera.launch.py
    ros2 launch wojtek_structure_core targeting_camera.launch.py enable_depth:=false
    ros2 launch wojtek_structure_core targeting_camera.launch.py cpus:=2,3

Topics land under /<namespace>/<name>/, i.e. by default

    /targeting_camera/targeting_camera/color/image_raw
    /targeting_camera/targeting_camera/color/camera_info
    /targeting_camera/targeting_camera/depth/image_rect_raw
    /targeting_camera/targeting_camera/depth/camera_info

The defaults are NOT camera/camera on purpose: that pair belongs to
wojtek_perception_bringup's terrain D435, which is already live on the robot,
and two drivers publishing into the same namespace is the kind of collision
that costs an afternoon to notice.

Plain node, no component container -- the same reasoning as the perception
launch, and more so here: nothing else in this graph is C++, and the only
consumer of the colour image is the detector, which is a separate process (and
possibly a separate machine) by design.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "wojtek_structure_core"


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # Same affinity story as the perception launch: the robot bringup runs its
    # tree under taskset on the isolcpus RT cores, and neither this driver nor
    # anything downstream of it may land there -- the 400 Hz control loop owns
    # those exclusively, and this package is not allowed to cost it a cycle.
    cpus = arg("cpus")
    prefix = [f"taskset -c {cpus}"] if cpus else None

    # Single-value overrides layered on top of the parameter file. Each entry of
    # `parameters=` becomes its own --params-file in list order, so the override
    # that comes last wins. An empty argument means "whatever the file says".
    overrides = {}
    for name, cast in (
        ("sensor_serial", str),
        ("enable_color", lambda v: v.lower() in ("true", "1")),
        ("enable_depth", lambda v: v.lower() in ("true", "1")),
        ("depth_resolution", str),
    ):
        if arg(name):
            overrides[name] = cast(arg(name))

    return [
        Node(
            package=PKG,
            executable="structure_core_node",
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
                description="Driver node name. Must not be `camera` -- that is "
                            "the terrain D435.",
            ),
            DeclareLaunchArgument(
                "camera_namespace", default_value="targeting_camera",
                description="Driver namespace; topics land under "
                            "/<namespace>/<name>/. Must not be `camera`.",
            ),
            DeclareLaunchArgument(
                "sensor_serial", default_value="",
                description="Pin a specific sensor by serial. Empty = the first "
                            "Structure Core found, which is what you want "
                            "unless two are plugged in.",
            ),
            DeclareLaunchArgument(
                "enable_color", default_value="",
                description="Colour stream (the one YOLO reads). Empty = the "
                            "config file.",
            ),
            DeclareLaunchArgument(
                "enable_depth", default_value="",
                description="Depth stream (range at the bbox centre). "
                            "enable_depth:=false drops it when only the "
                            "detection path is being worked on, and is the "
                            "first thing to try if the USB link is saturated.",
            ),
            DeclareLaunchArgument(
                "depth_resolution", default_value="",
                description="QVGA | VGA | SXGA. Empty = the config file's QVGA.",
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
