"""AGX MID-360S: Livox point cloud and IMU, without RViz or static TF."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Works both from the source tree and the installed package share directory.
    config_file = str(Path(__file__).resolve().parent.parent / "config" / "MID360s_agx.json")
    return LaunchDescription([
        DeclareLaunchArgument(
            "user_config_path", default_value=config_file,
            description="MID-360S JSON network configuration"),
        DeclareLaunchArgument(
            "xfer_format", default_value="1",
            description="1: Livox CustomMsg for FAST-LIO2; 0: sensor_msgs/PointCloud2"),
        DeclareLaunchArgument(
            "publish_freq", default_value="10.0",
            description="Point-cloud publication frequency in Hz"),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "user_config_path": ParameterValue(
                    LaunchConfiguration("user_config_path"), value_type=str),
                "xfer_format": ParameterValue(
                    LaunchConfiguration("xfer_format"), value_type=int),
                "publish_freq": ParameterValue(
                    LaunchConfiguration("publish_freq"), value_type=float),
                "multi_topic": 0,
                "data_src": 0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
            }],
        ),
    ])
