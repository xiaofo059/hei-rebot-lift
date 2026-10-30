"""FAST-LIO2 odometry for AGX MID-360S; start the Livox driver separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("fast_lio"), "config", "mid360s_agx.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=config_file,
            description="FAST-LIO2 parameter YAML; verify LiDAR-IMU extrinsics before use"),
        DeclareLaunchArgument(
            "lidar_topic", default_value="/livox/lidar",
            description="Livox CustomMsg input; driver requires xfer_format:=1"),
        DeclareLaunchArgument(
            "imu_topic", default_value="/livox/imu",
            description="sensor_msgs/Imu input"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use ROS simulation clock"),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "common.lid_topic": ParameterValue(
                        LaunchConfiguration("lidar_topic"), value_type=str),
                    "common.imu_topic": ParameterValue(
                        LaunchConfiguration("imu_topic"), value_type=str),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
