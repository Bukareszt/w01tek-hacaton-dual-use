"""The guard that keeps `core/` testable.

Every module under `core/` has to run on a laptop with numpy and nothing else.
The moment one of them imports rclpy or a message type, the tests stop being
runnable anywhere the robot is not, and the arithmetic stops being separable
from the plumbing.  This test is what says so out loud.
"""

import pathlib
import sys

CORE = pathlib.Path(__file__).resolve().parent.parent / "wojtek_follow" / "core"
FORBIDDEN = ("rclpy", "sensor_msgs", "std_msgs", "geometry_msgs", "std_srvs",
             "wojtek_targeting_msgs", "cv_bridge", "cv2")


def test_no_module_in_core_imports_ros():
    modules = sorted(CORE.glob("*.py"))
    assert len(modules) >= 5
    for path in modules:
        for number, line in enumerate(path.read_text().splitlines(), start=1):
            stripped = line.strip()
            if not (stripped.startswith("import ") or stripped.startswith("from ")):
                continue
            for name in FORBIDDEN:
                assert name not in stripped, f"{path.name}:{number} imports {name}"


def test_importing_core_pulls_in_no_ros():
    """Not even indirectly, through something else in the package."""
    for name in list(sys.modules):
        if name.startswith("wojtek_follow"):
            del sys.modules[name]
    import wojtek_follow.core.aim  # noqa: F401
    import wojtek_follow.core.controller  # noqa: F401
    import wojtek_follow.core.gate  # noqa: F401
    import wojtek_follow.core.geometry  # noqa: F401
    import wojtek_follow.core.rangefinder  # noqa: F401

    assert "wojtek_follow.follow_node" not in sys.modules
