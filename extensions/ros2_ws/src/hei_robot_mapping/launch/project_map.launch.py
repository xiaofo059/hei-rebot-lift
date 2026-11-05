"""Project a saved 3-D RTAB-Map database into a named Nav2 map, then exit."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_projector(context):
    config = LaunchConfiguration("config_file").perform(context)
    if not os.path.isfile(config):
        raise RuntimeError(f"Projection configuration not found: {config}")
    overrides = {"map_name": LaunchConfiguration("map_name").perform(context)}
    for key in ("output_name", "output_directory"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            overrides[key] = value
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
        DeclareLaunchArgument("map_name", description="Saved map filename, e.g. warehouse_01"),
        DeclareLaunchArgument("config_file", default_value=os.path.join(share, "config", "map_projection.yaml")),
        DeclareLaunchArgument("output_name", default_value="", description="Optional basename; existing files are protected"),
        DeclareLaunchArgument("output_directory", default_value="", description="Optional output directory override"),
        OpaqueFunction(function=start_projector),
    ])
