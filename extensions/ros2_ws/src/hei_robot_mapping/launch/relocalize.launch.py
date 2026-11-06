"""Load a read-only DB and expose a checked pre-navigation relocalize service."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start(context):
    config = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    with config.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = document["/**"]["ros__parameters"]
        database = Path(parameters["database_path"]).expanduser()
        pose = parameters["initial_pose"]
    except (KeyError, TypeError) as error:
        raise RuntimeError("config_file needs /**.ros__parameters.database_path and initial_pose") from error
    if not database.is_absolute() or database.suffix != ".db" or not database.is_file():
        raise RuntimeError(f"database_path must be an existing absolute .db file: {database}")
    if (not isinstance(pose, list) or len(pose) != 6
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in pose)):
        raise RuntimeError("initial_pose must contain six finite numbers: x y z roll pitch yaw (m, rad)")
    for name in ("service_timeout_sec", "localization_timeout_sec", "data_timeout_sec"):
        if name in parameters:
            value = parameters[name]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise RuntimeError(f"{name} must be finite and positive")
    if ("required_matches" in parameters and
            (type(parameters["required_matches"]) is not int or parameters["required_matches"] < 1)):
        raise RuntimeError("required_matches must be a positive integer")
    if "auto_relocalize" in parameters and type(parameters["auto_relocalize"]) is not bool:
        raise RuntimeError("auto_relocalize must be true or false")
    # The shared localization launch owns the RTAB-Map instance and fixed frames.
    for name, expected in {"rtabmap_node": "/rtabmap/rtabmap", "map_frame": "map",
                           "body_frame": "body", "initialpose_topic": "/initialpose",
                           "info_topic": "/rtabmap/info",
                           "localization_pose_topic": "/rtabmap/localization_pose"}.items():
        if parameters.get(name, expected) != expected:
            raise RuntimeError(f"{name} must be {expected} for this shared localization launch")
    parameters = dict(parameters)
    parameters["database_path"] = str(database.resolve())
    parameters["initial_pose"] = [float(value) for value in pose]
    for name in ("service_timeout_sec", "localization_timeout_sec", "data_timeout_sec"):
        if name in parameters:
            parameters[name] = float(parameters[name])
    parameters["use_sim_time"] = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    share = Path(get_package_share_directory("hei_robot_mapping"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "localization.launch.py")),
            launch_arguments={
                "map_name": database.name,
                "map_dir": str(database.parent),
                "initial_pose": " ".join(str(float(value)) for value in pose),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "rviz": LaunchConfiguration("rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items()),
        Node(package="hei_robot_mapping", executable="rtabmap_relocalizer.py",
             namespace="hei_robot_mapping", name="relocalizer", output="screen",
             parameters=[parameters]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("hei_robot_mapping"))
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config" / "relocalize.yaml")),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Optionally open RViz2 for localization inspection"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_start),
    ])
