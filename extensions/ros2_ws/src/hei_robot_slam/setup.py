from glob import glob
import os

from setuptools import setup

package_name = "hei_robot_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HEI ReBot Lift",
    maintainer_email="dev@hei-rebot-lift.invalid",
    description="MID-360S 3D SLAM integration: FAST-LIO2 frontend + RTAB-Map 0.23.13 backend",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "slam_cloud_adapter = hei_robot_slam.cloud_adapter:main",
            "slam_cloud_probe = hei_robot_slam.cloud_probe:main",
            "slam_db_tools = hei_robot_slam.db_tools:main",
            "slam_run_bounded = hei_robot_slam.run_bounded:main",
        ],
    },
)
