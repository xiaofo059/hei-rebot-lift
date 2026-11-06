"""Export a saved RTAB-Map DB as full 3-D PCD and a height-projected Nav2 map."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_projector(context):
    config = LaunchConfiguration("config_file").perform(context)
    if not os.path.isfile(config):
        raise RuntimeError(f"Projection configuration not found: {config}")
    overrides = {}
    for key in ("db_path", "map_name", "output_directory"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            overrides[key] = ParameterValue(value, value_type=str)
    projector = Node(
        package="hei_robot_mapping", executable="map_projector", name="map_projector",
        output="screen", parameters=[config, overrides],
    )

    def check_exit(event, _context):
        if event.returncode != 0:
            raise RuntimeError(f"Map projection failed (exit code {event.returncode}); see log above")
        return []

    return [RegisterEventHandler(OnProcessExit(target_action=projector, on_exit=check_exit)), projector]


def generate_launch_description():
    share = get_package_share_directory("hei_robot_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("db_path", default_value="", description="Path to the saved RTAB-Map .db file"),
        DeclareLaunchArgument("map_name", default_value="", description="Legacy alternative to db_path, e.g. warehouse_01"),
        DeclareLaunchArgument("config_file", default_value=os.path.join(share, "config", "map_projection.yaml")),
        DeclareLaunchArgument("output_directory", default_value="", description="Output directory; empty uses the DB directory. All filenames use the DB stem"),
        OpaqueFunction(function=start_projector),
    ])
