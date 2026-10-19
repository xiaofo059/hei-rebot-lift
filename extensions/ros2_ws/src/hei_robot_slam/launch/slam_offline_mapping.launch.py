"""离线三维 SLAM 建图（FAST-LIO2 前端 + RTAB-Map 0.23.13 后端），只回放已有 bag。

安全与取证契约（缺一即拒绝启动）：

* **必须显式 uncalibrated:=true**：外参只有 MID-360S 厂商名义值，现场未标定；
* **必须离线隔离**：ROS_DOMAIN_ID=89 且 ROS_LOCALHOST_ONLY=1（前端/后端同域）；
* **每次运行写全新唯一目录**：output_root/<session>/rtabmap_mapping.db；已存在即拒绝；
* **绝不允许 -d / delete_db_on_start**：RTAB-Map 无 -d 时会打开并追加旧库，所以只允许新路径；
* frame 分工固定：FAST-LIO 独占 camera_init->body，RTAB-Map 独占 map->camera_init；
  本包不发布 TF、不伪造 base_link/odom；
* 输入固定：/cloud_registered_body 经 XYZI 适配后给 RTAB；单一 odom 来源（FAST-LIO）。

典型用法（新终端）：

  export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
  source /opt/ros/humble/setup.bash
  source extensions/third_party/livox_ws/install/local_setup.bash
  source extensions/third_party/fast_lio_ws/install/local_setup.bash
  source extensions/third_party/rtabmap_local/env.sh
  source extensions/ros2_ws/install/local_setup.bash
  ros2 launch hei_robot_slam slam_offline_mapping.launch.py \
      bag_path:=/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456 \
      uncalibrated:=true probe:=true
"""

from __future__ import annotations

import os
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


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001
    cfg = context.launch_configurations
    S.require_uncalibrated_optin(cfg)
    S.require_offline_isolation()

    bag_dir, run_dir, db_path = S.resolve_bag_and_run_dir(cfg)
    rate, start_offset = S.parse_replay_arguments(cfg)

    share = get_package_share_directory("hei_robot_slam")
    params_file = S.resolve_param_file(cfg, share, C.MODE_MAPPING)

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
    rviz_config = Path(share) / "config" / "slam_mapping.rviz"
    if rviz_enabled and not rviz_config.is_file():
        S.fail("RViz 配置缺失", str(rviz_config))

    probe_report = run_dir / "probe_report.json"
    info_path = S.write_launch_info(
        run_dir,
        {
            "kind": "hei_mid360s_slam_offline_mapping",
            "mode": C.MODE_MAPPING,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "bag_path": str(bag_dir),
            "run_dir": str(run_dir),
            "database_path": str(db_path),
            "replay": {
                "rate": rate,
                "start_offset_s": start_offset,
                "delay_s": C.BAG_PLAYER_DELAY_S,
                "use_sim_time": True,
                "bag_clock_hz": C.SIM_CLOCK_HZ,
            },
            "frames": {"map": C.MAP_FRAME, "odom": C.ODOM_FRAME, "base": C.BASE_FRAME},
            "tf_contract": C.tf_contract(),
            "cloud_contract": C.cloud_contract(),
            "versions": C.versions(),
            "rtabmap_params_file": {"path": str(params_file), "sha256": C.sha256_file(params_file)},
            "lio_config_file": {"path": str(lio_config), "sha256": C.sha256_file(lio_config)},
            "probe_enabled": probe_enabled,
            "rviz_enabled": rviz_enabled,
            "no_auto_delete": True,
        },
    )

    actions = [
        LogInfo(msg="[hei_robot_slam] 离线三维 SLAM 建图（FAST-LIO2 -> RTAB-Map 0.23.13）"),
        LogInfo(
            msg=(
                f"  bag        : {bag_dir}\n"
                f"  run_dir    : {run_dir}\n"
                f"  database   : {db_path}（全新，不 -d）\n"
                f"  params     : {params_file}\n"
                f"  rate       : {rate}    start_offset: {start_offset}s\n"
                f"  TF         : fastlio={C.ODOM_FRAME}->{C.BASE_FRAME}；rtabmap={C.MAP_FRAME}->{C.ODOM_FRAME}\n"
                f"  time       : use_sim_time=true（bag --clock {C.SIM_CLOCK_HZ} Hz，与源包同时间域）"
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
        actions.append(
            ExecuteProcess(
                cmd=S.build_bag_play_command(bag_dir, rate, start_offset),
                output="screen",
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
            DeclareLaunchArgument(
                "uncalibrated",
                default_value="false",
                description="必填 opt-in：确认外参只是厂商名义值、现场未标定（传 true）",
            ),
            DeclareLaunchArgument("rate", default_value="1.0", description="回放倍速（建议 1.0 保证不丢帧）"),
            DeclareLaunchArgument("start_offset", default_value="0.0", description="回放起点（秒）"),
            DeclareLaunchArgument(
                "output_root",
                default_value=str(C.DATA_ROOT),
                description="输出根目录；每次运行在其下新建唯一 session 子目录",
            ),
            DeclareLaunchArgument("session", default_value="", description="本次运行目录名（空=时间戳）"),
            DeclareLaunchArgument("play_bag", default_value="true", description="是否在 launch 内启动 ros2 bag play"),
            DeclareLaunchArgument("probe", default_value="false", description="是否启动只读取证探针（写 run_dir/probe_report.json）"),
            DeclareLaunchArgument("rviz", default_value="false", description="是否启动 RViz2（需要可用显示）"),
            DeclareLaunchArgument("rtabmap_params_file", default_value="", description="覆盖 RTAB-Map 参数文件（空=包内 mapping 配置）"),
            DeclareLaunchArgument("lio_config_file", default_value="", description="覆盖 FAST-LIO 参数文件（空=hei_robot_localization 包内 uncalibrated 配置）"),
            OpaqueFunction(function=_launch_setup),
        ]
    )