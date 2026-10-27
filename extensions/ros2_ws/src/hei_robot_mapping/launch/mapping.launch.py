"""FAST-LIO2 -> XYZI adapter -> RTAB-Map online mapping, without map persistence."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("hei_robot_mapping"),
                          "config", "mapping.yaml")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body",
                              description="FAST-LIO2 deskewed body-frame PointCloud2"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry",
                              description="FAST-LIO2 Odometry, camera_init -> body"),
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz2 on the AGX desktop"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("hei_robot_mapping"), "config", "mapping.rviz"),
                              description="RViz2 configuration file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="hei_robot_mapping", executable="fastlio_cloud_adapter",
            namespace="hei_robot_mapping", name="cloud_adapter", output="screen",
            parameters=[{"expected_frame": "body", "use_sim_time": use_sim_time}],
            remappings=[("input_cloud", LaunchConfiguration("cloud_topic"))],
        ),
        Node(
            package="rtabmap_slam", executable="rtabmap", namespace="rtabmap",
            name="rtabmap", output="screen",
            parameters=[LaunchConfiguration("config_file"), {"use_sim_time": use_sim_time}],
            remappings=[("scan_cloud", "/hei_robot_mapping/scan_cloud"),
                        ("odom", LaunchConfiguration("odom_topic"))],
        ),
        Node(
            package="rviz2", executable="rviz2", name="mapping_rviz",
            output="screen", condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
