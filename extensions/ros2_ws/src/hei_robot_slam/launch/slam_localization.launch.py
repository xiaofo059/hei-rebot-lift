"""已有数据库定位模式（只读原地图，不新增/不清空建图数据库）。

契约：

* database_path 必须是**已存在**的建图数据库；Mem/LocalizationReadOnly=true，退出不回写；
* 运行前后数据库 SHA256 必须一致（由 harness/测试核对）；
* frame 分工不变：FAST-LIO camera_init->body，RTAB-Map map->camera_init；
* 手动初始位姿入口：topic /initialpose（geometry_msgs/PoseWithCovarianceStamped），
  pose 为 base(body) 在 map 帧中的位姿；header.frame_id 为空时按 map 处理；
* 离线隔离：ROS_DOMAIN_ID=89 且 ROS_LOCALHOST_ONLY=1。

典型用法：

  ros2 launch hei_robot_slam slam_localization.launch.py \
      bag_path:=/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456 \
      database_path:=/home/robo/hei-data/mid360s_slam_20261019/<mapping_run>/rtabmap_mapping.db \
      uncalibrated:=true probe:=true
"""

from __future__ import annotations

import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hei_robot_localization.mid360s_lio_params import load_and_validate as load_lio_params
from hei_robot_slam import launch_support as S
from hei_robot_slam import slam_contract as C

INITIAL_POSE_EXAMPLE = (
    'ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '
    '"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
    'orientation: {w: 1.0}}}}"'
)


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001
    cfg = context.launch_configurations
    S.require_uncalibrated_optin(cfg)
    S.require_offline_isolation()

    bag_dir, db_path, run_dir = S.resolve_localization_inputs(cfg)
    rate, start_offset = S.parse_replay_arguments(cfg)
    db_sha256_before = C.sha256_file(db_path)

    share = get_package_share_directory("hei_robot_slam")
    params_file = S.resolve_param_file(cfg, share, C.MODE_LOCALIZATION)

    lio_share = get_package_share_directory("hei_robot_localization")
    lio_override = str(cfg.get("lio_config_file", "")).strip()
    lio_config = (
        Path(lio_override).expanduser()
        if lio_override
        else Path(lio_share) / "config" / "mid360s_fastlio_uncalibrated.yaml"
    )
    try:
        load_lio_params(str(lio_config))
    except ValueError as exc:
        S.fail("FAST-LIO 参数不符合输入契约", f"{lio_config}: {exc}")

    probe_enabled = str(cfg.get("probe", "false")).strip().lower() in ("true", "1", "yes")
    rviz_enabled = str(cfg.get("rviz", "false")).strip().lower() in ("true", "1", "yes")
    play_bag = str(cfg.get("play_bag", "true")).strip().lower() in ("true", "1", "yes")
    rviz_config = Path(share) / "config" / "slam_localization.rviz"
    if rviz_enabled and not rviz_config.is_file():
        S.fail("RViz 配置缺失", str(rviz_config))

    probe_report = run_dir / "probe_report.json"
    info_path = S.write_launch_info(
        run_dir,
        {
            "kind": "hei_mid360s_slam_localization_readonly",
            "mode": C.MODE_LOCALIZATION,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "bag_path": str(bag_dir),
            "database_path": str(db_path),
            "database_sha256_before": db_sha256_before,
            "run_dir": str(run_dir),
            "replay": {
                "rate": rate,
                "start_offset_s": start_offset,
                "delay_s": C.BAG_PLAYER_DELAY_S,
                "use_sim_time": True,
                "bag_clock_hz": C.SIM_CLOCK_HZ,
            },
            "frames": {"map": C.MAP_FRAME, "odom": C.ODOM_FRAME, "base": C.BASE_FRAME},
            "initial_pose_topic": "/initialpose",
            "initial_pose_example": INITIAL_POSE_EXAMPLE,
            "versions": C.versions(),
            "rtabmap_params_file": {"path": str(params_file), "sha256": C.sha256_file(params_file)},
            "lio_config_file": {"path": str(lio_config), "sha256": C.sha256_file(lio_config)},
            "database_expected_unmodified": True,
        },
    )

    actions = [
        LogInfo(msg="[hei_robot_slam] 只读定位模式（原建图数据库，不新增/不清空）"),
        LogInfo(
            msg=(
                f"  bag        : {bag_dir}\n"
                f"  database   : {db_path}\n"
                f"  db_sha256  : {db_sha256_before}\n"
                f"  run_dir    : {run_dir}\n"
                f"  params     : {params_file}\n"
                f"  rate       : {rate}    start_offset: {start_offset}s\n"
                f"  time       : use_sim_time=true（bag --clock {C.SIM_CLOCK_HZ} Hz，与源包同时间域）\n"
                f"  手动初始位姿: {INITIAL_POSE_EXAMPLE}"
            )
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[str(lio_config), {"use_sim_time": True}],
        ),
        Node(
            package="hei_robot_slam",
            executable="slam_cloud_adapter",
            name="slam_cloud_adapter",
            output="screen",
            parameters=[{"input_qos": "reliable", "report_period_s": 5.0, "use_sim_time": True}],
            remappings=[("input_cloud", C.LIO_BODY_CLOUD_TOPIC), ("output_cloud", C.SLAM_CLOUD_TOPIC)],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[str(params_file), {"database_path": str(db_path), "use_sim_time": True}],
            remappings=[("scan_cloud", C.SLAM_CLOUD_TOPIC)],
        ),
    ]

    if probe_enabled:
        actions.append(
            Node(
                package="hei_robot_slam",
                executable="slam_cloud_probe",
                name="slam_cloud_probe",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "report_path": str(probe_report),
                        "report_period_s": 2.0,
                        "odom_topic": C.LIO_ODOM_TOPIC,
                        "body_cloud_topic": C.LIO_BODY_CLOUD_TOPIC,
                        "adapted_cloud_topic": C.SLAM_CLOUD_TOPIC,
                        "map_cloud_topic": "/cloud_map",
                        "map_path_topic": "/mapPath",
                    }
                ],
            )
        )

    if play_bag:
        actions.append(ExecuteProcess(cmd=S.build_bag_play_command(bag_dir, rate, start_offset), output="screen"))

    if rviz_enabled:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        )

    actions.append(LogInfo(msg=f"[hei_robot_slam] launch_info 已写入 {info_path}"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path", description="必填：要回放的原始 MID-360S rosbag 目录"),
            DeclareLaunchArgument("database_path", description="必填：已存在的建图数据库（只读）"),
            DeclareLaunchArgument(
                "uncalibrated",
                default_value="false",
                description="必填 opt-in：确认外参只是厂商名义值、现场未标定（传 true）",
            ),
            DeclareLaunchArgument("rate", default_value="1.0", description="回放倍速"),
            DeclareLaunchArgument("start_offset", default_value="0.0", description="回放起点（秒）"),
            DeclareLaunchArgument(
                "output_root",
                default_value=str(C.DATA_ROOT),
                description="输出根目录；每次运行在其下新建唯一 session 子目录（只放日志/取证）",
            ),
            DeclareLaunchArgument("session", default_value="", description="本次运行目录名（空=时间戳）"),
            DeclareLaunchArgument("play_bag", default_value="true", description="是否在 launch 内启动 ros2 bag play"),
            DeclareLaunchArgument("probe", default_value="false", description="是否启动只读取证探针"),
            DeclareLaunchArgument("rviz", default_value="false", description="是否启动 RViz2"),
            DeclareLaunchArgument("rtabmap_params_file", default_value="", description="覆盖 RTAB-Map 定位参数文件"),
            DeclareLaunchArgument("lio_config_file", default_value="", description="覆盖 FAST-LIO 参数文件"),
            OpaqueFunction(function=_launch_setup),
        ]
    )