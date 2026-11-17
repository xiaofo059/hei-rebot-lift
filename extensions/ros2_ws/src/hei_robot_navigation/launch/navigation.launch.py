#!/usr/bin/env python3
"""Start Nav2 and a 2D map server alongside existing RTAB-Map localization."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _launch_setup(context):
    share = Path(get_package_share_directory('hei_robot_navigation'))
    params_path = Path(LaunchConfiguration('params_file').perform(context)).expanduser().resolve()
    if not params_path.is_file():
        raise RuntimeError(f'Nav2 parameters file does not exist: {params_path}')
    with params_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    # Keep Nav2 poses/commands and wheel-odometry twist at the chassis center.
    # RTAB-Map and FAST-LIO2 continue to track body; do not rename their frames.
    try:
        base_transform = config['body_to_base_link']['ros__parameters']
        publish_base_tf = base_transform['publish_tf']
        if not isinstance(publish_base_tf, bool):
            raise ValueError('publish_tf must be a YAML boolean')
        transform_values = {key: float(base_transform[key])
                            for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}
        if not all(math.isfinite(value) for value in transform_values.values()):
            raise ValueError('transform values must be finite')
    except (TypeError, KeyError, ValueError) as exc:
        raise RuntimeError('Set body_to_base_link.ros__parameters with publish_tf '
                           'and numeric x/y/z/roll/pitch/yaw mounting values') from exc

    map_override = LaunchConfiguration('map').perform(context).strip()
    try:
        map_path = map_override or config['map_server']['ros__parameters']['yaml_filename']
    except (TypeError, KeyError) as exc:
        raise RuntimeError('Set map_server.ros__parameters.yaml_filename or the map launch argument') from exc
    if not isinstance(map_path, str) or not map_path.strip():
        raise RuntimeError('The 2D map YAML path must be a non-empty string')
    map_path = Path(map_path).expanduser()
    if not map_path.is_absolute():
        map_path = params_path.parent / map_path
    map_path = map_path.resolve()
    if not map_path.is_file():
        raise RuntimeError(f'2D map YAML does not exist: {map_path}')
    with map_path.open(encoding='utf-8') as stream:
        map_config = yaml.safe_load(stream)
    if not isinstance(map_config, dict) or not isinstance(map_config.get('image'), str):
        raise RuntimeError(f'Map YAML must contain an image path: {map_path}')
    image_path = Path(map_config['image']).expanduser()
    if not image_path.is_absolute():
        image_path = map_path.parent / image_path
    if not image_path.is_file():
        raise RuntimeError(f'2D map image does not exist: {image_path}')

    bt_value = config.get('bt_navigator', {}).get('ros__parameters', {}).get(
        'default_nav_to_pose_bt_xml', 'navigate_with_replanning.xml')
    if not isinstance(bt_value, str) or not bt_value.strip():
        raise RuntimeError('bt_navigator.default_nav_to_pose_bt_xml must name a BT XML file')
    bt_path = Path(bt_value).expanduser()
    if not bt_path.is_absolute():
        bt_path = params_path.parent / bt_path
    bt_path = bt_path.resolve()
    if not bt_path.is_file():
        raise RuntimeError(f'NavigateToPose behavior tree does not exist: {bt_path}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params = ParameterFile(RewrittenYaml(
        source_file=str(params_path),
        param_rewrites={'use_sim_time': use_sim_time, 'yaml_filename': str(map_path),
                        'default_nav_to_pose_bt_xml': str(bt_path)},
        convert_types=True), allow_substs=False)
    raw_command = '/hei_robot_navigation/cmd_vel_raw'
    output_command = LaunchConfiguration('cmd_vel_topic')
    # rmw_fastrtps 6.2.x waits for the service client's response reader using
    # the service writer's max_blocking_time (100 ms by default). Allow discovery
    # to finish during concurrent startup without changing sensor/topic QoS.
    service_profile = share / 'config' / 'fastdds_services.xml'
    navigation_env = {
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(service_profile),
        # Scope the AGX transport workaround to this launch's navigation nodes.
        'FASTDDS_BUILTIN_TRANSPORTS': LaunchConfiguration('nav_dds_transport'),
    }
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    navigation_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    nodes = [Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[params], remappings=tf_remaps,
        additional_env=navigation_env),
    ]
    if publish_base_tf:
        transform_arguments = []
        for key, value in transform_values.items():
            transform_arguments.extend([f'--{key}', str(value)])
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='navigation_body_to_base_link', output='screen',
            arguments=[*transform_arguments, '--frame-id', 'body',
                       '--child-frame-id', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}], remappings=tf_remaps,
            additional_env=navigation_env))
    for package, name in navigation_nodes:
        remaps = list(tf_remaps)
        if name in ('controller_server', 'behavior_server'):
            remaps.append(('cmd_vel', raw_command))
        elif name == 'velocity_smoother':
            remaps.extend([('cmd_vel', raw_command), ('cmd_vel_smoothed', output_command)])
        nodes.append(Node(
            package=package, executable=name, name=name, output='screen',
            parameters=[params], remappings=remaps, additional_env=navigation_env))
    rviz_delay = float(LaunchConfiguration('rviz_start_delay').perform(context))
    if not math.isfinite(rviz_delay) or rviz_delay < 0:
        raise RuntimeError('rviz_start_delay must be a finite non-negative number')
    nodes.extend([
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': LaunchConfiguration('autostart'),
                         # Configure/activate map_server first. A map startup
                         # failure must prevent a partially active Nav2 stack.
                         'node_names': ['map_server', *[name for _, name in navigation_nodes]]}],
            additional_env=navigation_env),
        # Separate graphics initialization from the concurrent Nav2 startup burst.
        TimerAction(period=rviz_delay, actions=[Node(
            package='rviz2', executable='rviz2', name='navigation_rviz',
            arguments=['-d', str(share / 'config' / 'navigation.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            # RViz retains a separate transport override for graphics diagnostics.
            additional_env={
                'FASTDDS_BUILTIN_TRANSPORTS': LaunchConfiguration('rviz_dds_transport')},
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen')]),
    ])
    return [LogInfo(msg=['Nav2 framework: map=', str(map_path),
                         ', autostart=', LaunchConfiguration('autostart'),
                         ', velocity output=', output_command]), *nodes]


def generate_launch_description():
    share = Path(get_package_share_directory('hei_robot_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'nav2_params.yaml'),
                              description='Nav2 parameters, including the 2D map YAML path'),
        DeclareLaunchArgument('map', default_value='',
                              description='Optional 2D map YAML override; never a DB or PCD file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Configure and activate map_server, then navigation servers'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Smoothed Twist output topic for chassis commands'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav_dds_transport', default_value='UDPv4',
                              choices=['UDPv4', 'DEFAULT'],
                              description='Nav2 Fast DDS transport; DEFAULT restores UDP + shared memory'),
        DeclareLaunchArgument('rviz_start_delay', default_value='5.0',
                              description='Seconds before starting RViz to reduce startup contention'),
        DeclareLaunchArgument('rviz_dds_transport', default_value='UDPv4',
                              choices=['UDPv4', 'DEFAULT'],
                              description='RViz-only Fast DDS transport; DEFAULT restores UDP + shared memory'),
        OpaqueFunction(function=_launch_setup),
    ])
