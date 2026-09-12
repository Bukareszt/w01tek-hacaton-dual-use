from glob import glob

from setuptools import find_packages, setup

package_name = "wojtek_targeting"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/udev", glob("udev/*.rules")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jakub Staudt",
    maintainer_email="kubastaudt@gmail.com",
    description="Dynamixel pan/tilt gimbal driver and controller for wojtek_targeting.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "targeting_controller = wojtek_targeting.targeting_controller:main",
            "test_gimbal_angles = wojtek_targeting.scripts.test_gimbal_angles:main",
        ],
    },
)
