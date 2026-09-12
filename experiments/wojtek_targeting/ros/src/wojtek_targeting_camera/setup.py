from glob import glob

from setuptools import setup

package_name = "wojtek_targeting_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jakub Staudt",
    maintainer_email="kubastaudt@gmail.com",
    description="USB webcam driver for the wojtek_targeting camera.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "usb_camera_node = wojtek_targeting_camera.usb_camera_node:main",
        ],
    },
)
