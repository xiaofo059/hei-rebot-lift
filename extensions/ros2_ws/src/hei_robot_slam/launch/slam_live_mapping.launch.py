"""可选 live 建图（笔记本直连 MID-360S）——**必须显式开启，默认不可用**。

本轮没有硬件授权、没有现场采集任务，因此该 launch **未在真实硬件上执行**。
它只启动：livox_ros_driver2（唯一 driver 实例）+ FAST-LIO + XYZI 适配 + RTAB-Map 建图；
**不启动**机器人底盘/机械臂/升降/导航，也不发布 base_link/odom。

安全闸门（缺一拒绝启动）：

* allow_live_hardware:=true —— 显式确认要访问真实 MID-360S 硬件；
* uncalibrated:=true —— 外参只是厂商名义值；
* config_path —— 现场网络配置（含 host_ip/lidar_ip），由 hei_robot_bringup 的
  mid360s_make_config 生成，IP 不进仓库；
* 仍要求 ROS_DOMAIN_ID=89 + ROS_LOCALHOST_ONLY=1，避免与其他实机会话串流。
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
            "live 硬件建图未获显式授权",
            "该 launch 会打开真实 MID-360S 传感器。确认现场条件后传 allow_live_hardware:=true。",
        )
    S.require_uncalibrated_optin(cfg)
    S.require_offline_isolation()

    config_path = str(cfg.get("config_path", "")).strip()
    if not config_path:
        S.fail(
            "缺少必填现场配置 config_path",
            "先生成现场网络配置（IP 不进仓库）：\n"
            "  ros2 run hei_robot_bringup mid360s_make_config \\\n"
            "      --host-ip <本机有线网卡IP> --lidar-ip <雷达IP> \\\n"
            "      --output extensions/config/local/mid360s_config.json",
        )
    if not Path(config_path).expanduser().is_file():
        S.fail("现场配置不存在", config_path)

    output_root = str(cfg.get("output_root", str(C.DATA_ROOT))).strip() or str(C.DATA_ROOT)
    session = str(cfg.get("session", "")).strip()
    try:
        run_dir = C.resolve_run_dir(output_root, session or None)
    except C.SlamContractError as exc:
        S.fail("无法创建唯一运行输出目录", str(exc))
    db_path = C.mapping_database_path(run_dir)
    try:
        C.validate_mapping_database_target(db_path)
    except C.SlamContractError as exc:
        S.fail("建图数据库路径不安全", str(exc))

    share = get_package_share_directory("hei_robot_slam")
    params_file = S.resolve_param_file(cfg, share, C.MODE_MAPPING)
    # 与离线 launch 一致：复用 hei_robot_localization 的参数门控（名义外参、
    # extrinsic_est_en=false 等），不允许 live 走另一套未校验的 FAST-LIO 参数。
    lio_config = (
        Path(get_package_share_directory("hei_robot_localization"))
        / "config"
        / "mid360s_fastlio_uncalibrated.yaml"
    )
    try:
        load_lio_params(str(lio_config))
    except ValueError as exc:
        S.fail("FAST-LIO 参数不符合输入契约", f"{lio_config}: {exc}")
    bringup_launch = Path(get_package_share_directory("hei_robot_bringup")) / "launch" / "mid360s_bringup.launch.py"
    if not bringup_launch.is_file():
        S.fail("找不到 hei_robot_bringup 的 MID-360S 启动文件", str(bringup_launch))

    probe_enabled = str(cfg.get("probe", "false")).strip().lower() in ("true", "1", "yes")
    rviz_enabled = str(cfg.get("rviz", "false")).strip().lower() in ("true", "1", "yes")
    rviz_config = Path(share) / "config" / "slam_mapping.rviz"

    probe_report = run_dir / "probe_report.json"
    info_path = S.write_launch_info(
        run_dir,
        {
            "kind": "hei_mid360s_slam_live_mapping",
            "mode": C.MODE_MAPPING,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "field_config_path": config_path,
            "run_dir": str(run_dir),
            "database_path": str(db_path),
            "frames": {"map": C.MAP_FRAME, "odom": C.ODOM_FRAME, "base": C.BASE_FRAME},
            "tf_contract": C.tf_contract(),
            "cloud_contract": C.cloud_contract(),
            "versions": C.versions(),
            "rtabmap_params_file": {"path": str(params_file), "sha256": C.sha256_file(params_file)},
            "no_robot_base_arm_lift": True,
        },
    )

    actions = [
        LogInfo(msg="[hei_robot_slam] LIVE 硬件建图（仅 MID-360S；不启动底盘/机械臂/升降）"),
        LogInfo(
            msg=(
                f"  field_config: {config_path}\n"
                f"  run_dir     : {run_dir}\n"
                f"  database    : {db_path}（全新，不 -d）"
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
            parameters=[str(lio_config), {"use_sim_time": False}],  # live：真实传感器时间，全链路 false
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
                "allow_live_hardware",
                default_value="false",
                description="必填 opt-in：确认要访问真实 MID-360S 硬件（默认 false 拒绝启动）",
            ),
            DeclareLaunchArgument(
                "uncalibrated",
                default_value="false",
                description="必填 opt-in：确认外参只是厂商名义值、现场未标定（传 true）",
            ),
            DeclareLaunchArgument("output_root", default_value=str(C.DATA_ROOT), description="输出根目录"),
            DeclareLaunchArgument("session", default_value="", description="本次运行目录名（空=时间戳）"),
            DeclareLaunchArgument("probe", default_value="false", description="是否启动只读取证探针"),
            DeclareLaunchArgument("rviz", default_value="false", description="是否启动 RViz2"),
            DeclareLaunchArgument("rtabmap_params_file", default_value="", description="覆盖 RTAB-Map 参数文件"),
            OpaqueFunction(function=_launch_setup),
        ]
    )