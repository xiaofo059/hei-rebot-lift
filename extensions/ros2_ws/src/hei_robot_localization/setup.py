from glob import glob
import os

from setuptools import setup

package_name = "hei_robot_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HEI ReBot Lift",
    maintainer_email="dev@hei-rebot-lift.invalid",
    description="MID-360S -> FAST-LIO2 输入契约、只读输入审计与离线 uncalibrated 启动",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lio_input_audit = hei_robot_localization.lio_input_audit:main",
        ],
    },
)
