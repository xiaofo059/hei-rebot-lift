"""使用实时 MID-360S 数据对已有 RTAB-Map 地图做只读定位。

只启动雷达驱动、FAST-LIO2、点云适配、RTAB-Map 定位和可选 RViz；
不启动底盘、机械臂或升降机构。真实雷达访问必须显式 opt-in。
"""

from __future__ import annotations

import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hei_robot_localization.mid360s_lio_params import load_and_validate as load_lio_params
from hei_robot_slam import launch_support as S
from hei_robot_slam import slam_contract as C


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001
    cfg = context.launch_configurations
    allow = str(cfg.get("allow_live_hardware", "false")).strip().lower() in ("true", "1", "yes")
    if not allow:
        S.fail(
            "live 硬件定位未获显式授权",
            "该 launch 会打开真实 MID-360S 传感器。确认现场条件后传 allow_live_hardware:=true。",
        )
    S.require_uncalibrated_optin(cfg)
    S.require_offline_isolation()

    config_path = str(cfg.get("config_path", "")).strip()
    if not config_path:
        S.fail("缺少必填参数 config_path", "请传现场 MID-360S 网络配置 JSON（IP 不进仓库）")
    config_path = str(Path(config_path).expanduser())
    if not Path(config_path).is_file():
        S.fail("现场配置不存在", config_path)

    db_path, run_dir = S.resolve_live_localization_inputs(cfg)
    db_sha256_before = C.sha256_file(db_path)

    share = get_package_share_directory("hei_robot_slam")
    params_file = S.resolve_param_file(cfg, share, C.MODE_LOCALIZATION)
    lio_share = get_package_share_directory("hei_robot_localization")
    lio_config = Path(lio_share) / "config" / "mid360s_fastlio_uncalibrated.yaml"
    try:
        load_lio_params(str(lio_config))
    except ValueError as exc:
        S.fail("FAST-LIO 参数不符合输入契约", f"{lio_config}: {exc}")

    bringup_launch = Path(get_package_share_directory("hei_robot_bringup")) / "launch" / "mid360s_bringup.launch.py"
    if not bringup_launch.is_file():
        S.fail("找不到 hei_robot_bringup 的 MID-360S 启动文件", str(bringup_launch))

    probe_enabled = str(cfg.get("probe", "false")).strip().lower() in ("true", "1", "yes")
    rviz_enabled = str(cfg.get("rviz", "false")).strip().lower() in ("true", "1", "yes")
    rviz_config = Path(share) / "config" / "slam_localization.rviz"
    if rviz_enabled and not rviz_config.is_file():
        S.fail("RViz 定位配置缺失", str(rviz_config))

    probe_report = run_dir / "probe_report.json"
    info_path = S.write_launch_info(
        run_dir,
        {
            "kind": "hei_mid360s_slam_live_localization_readonly",
            "mode": C.MODE_LOCALIZATION,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "field_config_path": config_path,
            "database_path": str(db_path),
            "database_sha256_before": db_sha256_before,
            "database_expected_unmodified": True,
            "run_dir": str(run_dir),
            "frames": {"map": C.MAP_FRAME, "odom": C.ODOM_FRAME, "base": C.BASE_FRAME},
            "initial_pose_topic": "/initialpose",
            "versions": C.versions(),
            "rtabmap_params_file": {"path": str(params_file), "sha256": C.sha256_file(params_file)},
            "lio_config_file": {"path": str(lio_config), "sha256": C.sha256_file(lio_config)},
            "live_sensor_time": True,
            "no_robot_base_arm_lift": True,
        },
    )

    actions = [
        LogInfo(msg="[hei_robot_slam] LIVE 只读定位（使用已有地图；不写建图数据库）"),
        LogInfo(
            msg=(
                f"  field_config: {config_path}\n"
                f"  database    : {db_path}\n"
                f"  db_sha256   : {db_sha256_before}\n"
                f"  run_dir     : {run_dir}\n"
                f"  params      : {params_file}\n"
                f"  initial pose: publish /initialpose in map frame when needed"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_launch)),
            launch_arguments={
                "config_path": config_path,
                "pointcloud_format": "custom",
                "publish_freq": "10.0",
                "launch_rviz": "false",
            }.items(),
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[str(lio_config), {"use_sim_time": False}],
        ),
        Node(
            package="hei_robot_slam",
            executable="slam_cloud_adapter",
            name="slam_cloud_adapter",
            output="screen",
            parameters=[{"input_qos": "reliable", "report_period_s": 10.0, "use_sim_time": False}],
            remappings=[("input_cloud", C.LIO_BODY_CLOUD_TOPIC), ("output_cloud", C.SLAM_CLOUD_TOPIC)],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[str(params_file), {"database_path": str(db_path), "use_sim_time": False}],
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
                parameters=[{"report_path": str(probe_report), "report_period_s": 5.0}],
            )
        )

    if rviz_enabled:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        )

    actions.append(LogInfo(msg=f"[hei_robot_slam] launch_info 已写入 {info_path}"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", description="必填：MID-360S 现场网络配置 JSON（IP 不进仓库）"),
            DeclareLaunchArgument(
                "database_path", description="必填：已有建图数据库；Mem/LocalizationReadOnly=true，只读使用"
            ),
            DeclareLaunchArgument(
                "allow_live_hardware",
                default_value="false",
                description="必填 opt-in：确认要访问真实 MID-360S 硬件（默认 false 拒绝启动）",
            ),
            DeclareLaunchArgument(
                "uncalibrated",
                default_value="false",
                description="必填 opt-in：确认外参只有厂商名义值（传 true）",
            ),
            DeclareLaunchArgument("output_root", default_value=str(C.DATA_ROOT), description="日志/探针输出根目录"),
            DeclareLaunchArgument("session", default_value="", description="运行目录名（空=时间戳）"),
            DeclareLaunchArgument("probe", default_value="false", description="是否启动只读取证探针"),
            DeclareLaunchArgument("rviz", default_value="false", description="是否启动 RViz2"),
            DeclareLaunchArgument("rtabmap_params_file", default_value="", description="覆盖 RTAB-Map 定位参数文件"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
