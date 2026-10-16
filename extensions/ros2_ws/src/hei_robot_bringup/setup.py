from glob import glob
import os

from setuptools import setup

package_name = "hei_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HEI ReBot Lift",
    maintainer_email="dev@hei-rebot-lift.invalid",
    description="MID-360S 参数化 bringup、现场配置生成与原始 rosbag 离线验收工具",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mid360s_make_config = hei_robot_bringup.make_config:main",
            "mid360s_bag_qc = hei_robot_bringup.bag_qc:main",
        ],
    },
)
