"""Load a saved laser map read-only and localize with FAST-LIO2 odometry."""
import math
from pathlib import Path
import sqlite3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_localization(context):
    name = LaunchConfiguration("map_name").perform(context)
    if (not name or name in (".", "..", ".db") or not name.strip()
            or "/" in name or "\\" in name
            or any(ord(c) < 32 or ord(c) == 127 for c in name)):
        raise RuntimeError("map_name must be one filename, e.g. warehouse_01 (optional .db suffix)")
    directory = Path(LaunchConfiguration("map_dir").perform(context)).expanduser()
    if not directory.is_absolute():
        raise RuntimeError("map_dir must be an absolute directory")
    database = directory / (name if Path(name).suffix == ".db" else name + ".db")
    if not database.is_file():
        raise RuntimeError(f"Saved map not found: {database}. Save the map before starting localization.")
    database = database.resolve()
    try:
        with sqlite3.connect(database.as_uri() + "?mode=ro", uri=True) as connection:
            tables = {row[0] for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'")}
            if not {"Node", "Link", "Data", "Admin"}.issubset(tables):
                raise RuntimeError(f"Not an RTAB-Map database: {database}")
            count = connection.execute("SELECT COUNT(*) FROM Node WHERE id>0").fetchone()[0]
            if not count:
                raise RuntimeError(f"Saved map has no nodes: {database}")
    except sqlite3.Error as error:
        raise RuntimeError(f"Cannot read saved map {database}: {error}") from error
    text = LaunchConfiguration("initial_pose").perform(context)
    try:
        pose = [float(value) for value in text.split()]
    except ValueError as error:
        raise RuntimeError("initial_pose must be: x y z roll pitch yaw (metres and radians)") from error
    if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
        raise RuntimeError("initial_pose needs six finite numbers: x y z roll pitch yaw (m, rad)")
    initial_pose = " ".join(str(value) for value in pose)
    share = Path(get_package_share_directory("hei_robot_mapping"))
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [
        LogInfo(msg=f"Localizing with {database} ({count} stored nodes), initial pose: {initial_pose}"),
        Node(
            package="hei_robot_mapping", executable="fastlio_cloud_adapter",
            namespace="hei_robot_mapping", name="cloud_adapter", output="screen",
            parameters=[{"expected_frame": "body", "use_sim_time": use_sim_time}],
            remappings=[("input_cloud", LaunchConfiguration("cloud_topic"))],
        ),
        Node(
            package="rtabmap_slam", executable="rtabmap", namespace="rtabmap",
            name="rtabmap", output="screen",
            parameters=[str(share / "config" / "mapping.yaml"),
                        str(share / "config" / "localization.yaml"), {
                            "database_path": str(database),
                            "initial_pose": initial_pose,
                            "use_sim_time": use_sim_time,
                            # Keep these guarantees even if a YAML file is edited.
                            "Mem/IncrementalMemory": "false",
                            "Mem/InitWMWithAllNodes": "true",
                            "Mem/LocalizationReadOnly": "true",
                        }],
            remappings=[("scan_cloud", "/hei_robot_mapping/scan_cloud"),
                        ("odom", LaunchConfiguration("odom_topic")),
                        ("initialpose", "/initialpose")],
        ),
        Node(
            package="rviz2", executable="rviz2", name="localization_rviz",
            output="screen", condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_name", description="Saved map name, e.g. warehouse_01"),
        DeclareLaunchArgument("map_dir", default_value=
                              "/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map",
                              description="Absolute directory containing the saved .db"),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0",
                              description="Approximate body pose in map: x y z roll pitch yaw (m, rad)"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body",
                              description="FAST-LIO2 deskewed body-frame PointCloud2"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz2 on the AGX desktop"),
        DeclareLaunchArgument("rviz_config", default_value=str(
            Path(get_package_share_directory("hei_robot_mapping")) / "config" / "localization.rviz"),
                              description="RViz2 configuration file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_start_localization),
    ])
