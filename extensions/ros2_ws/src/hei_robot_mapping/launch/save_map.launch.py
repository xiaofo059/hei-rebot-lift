"""Save a named RTAB-Map laser database snapshot, then exit."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    saver = Node(
        package="hei_robot_mapping", executable="map_saver", name="map_saver",
        output="screen", parameters=[{
            key: ParameterValue(LaunchConfiguration(key), value_type=str)
            for key in ("map_name", "output_dir", "rtabmap_node", "map_service")
        }, {
            key: ParameterValue(LaunchConfiguration(key), value_type=float)
            for key in ("wait_timeout_sec", "request_timeout_sec")
        }],
    )

    def check_exit(event, context):
        if event.returncode != 0:
            raise RuntimeError(f"Map save failed (map_saver exit code {event.returncode}); see log above")
        return []

    return LaunchDescription([
        DeclareLaunchArgument("map_name", description="Map filename, e.g. warehouse_01"),
        DeclareLaunchArgument("output_dir", default_value=
                              "/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map"),
        DeclareLaunchArgument("rtabmap_node", default_value="/rtabmap/rtabmap"),
        DeclareLaunchArgument("map_service", default_value=[
            LaunchConfiguration("rtabmap_node"), "/get_map_data"]),
        DeclareLaunchArgument("wait_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("request_timeout_sec", default_value="120.0"),
        RegisterEventHandler(OnProcessExit(target_action=saver, on_exit=check_exit)),
        saver,
    ])
