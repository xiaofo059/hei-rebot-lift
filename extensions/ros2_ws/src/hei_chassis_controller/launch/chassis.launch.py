"""Chassis-only ros2_control bringup; real hardware by default."""
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    share = Path(get_package_share_directory("hei_chassis_controller"))
    config = LaunchConfiguration("config_file").perform(context)
    hw_config = LaunchConfiguration("hardware_config").perform(context)
    mock = LaunchConfiguration("use_mock_hardware").perform(context).lower()
    if mock not in ("true", "false"):
        raise ValueError("use_mock_hardware must be true or false")
    with open(config, encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["chassis_controller"]["ros__parameters"]
    with open(hw_config, encoding="utf-8") as stream:
        hardware = yaml.safe_load(stream)["hardware"]
    port = LaunchConfiguration("port").perform(context) or hardware["port"]
    description = xacro.process_file(
        str(share / "urdf" / "chassis.urdf.xacro"),
        mappings={
            "use_mock_hardware": mock, "hardware_config": hw_config, "port": port,
            "wheel_radius": str(parameters["wheel_radius"]),
            "half_length": str(parameters["half_length"]),
            "half_width": str(parameters["half_width"]),
            "base_frame": parameters["base_frame_id"],
        },
    ).toxml()
    nodes = [
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[config, {"robot_description": description}], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "chassis_controller",
                        "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "30"], output="screen"),
    ]
    if LaunchConfiguration("publish_robot_state").perform(context).lower() == "true":
        nodes.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                          parameters=[{"robot_description": description}], output="screen"))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("hei_chassis_controller"))
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="false (default) connects and enables real wheel motors; true uses mock hardware"),
        DeclareLaunchArgument("config_file", default_value=str(share / "config" / "controllers.yaml")),
        DeclareLaunchArgument("hardware_config", default_value=str(share / "config" / "hardware.yaml")),
        DeclareLaunchArgument("port", default_value="", description="Optional serial port override"),
        DeclareLaunchArgument("publish_robot_state", default_value="false",
                              description="Enable only when no full robot_state_publisher owns base_link/wheel TFs"),
        OpaqueFunction(function=_launch),
    ])
