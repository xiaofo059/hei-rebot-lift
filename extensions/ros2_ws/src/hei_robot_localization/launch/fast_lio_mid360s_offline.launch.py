"""MID-360S 离线 FAST-LIO2 启动（只回放已有 bag，不接任何硬件）。

安全与取证契约
--------------
* **必须显式 uncalibrated:=true**：外参只有厂商名义值，现场未标定。没有这个参数就
  拒绝启动，避免“默认看起来已标定”的误读。
* **必须离线隔离**：要求 ROS_DOMAIN_ID=88 且 ROS_LOCALHOST_ONLY=1，避免与实机/其他
  会话串流；不满足直接拒绝启动。
* **不发布任何 TF**：camera_init -> body 的唯一发布者是 fastlio_mapping（源码硬编码，
  laserMapping.cpp:648-658）。本包不发布 base_link、不发布 map，也不做 RTAB-Map 的
  map->odom（那是后续 04-SLAM 的职责）。
* 只接受**已存在**的 bag 目录（含 metadata.yaml）；本 launch 不会启动 driver、
  不会访问网卡，也不会触发任何运动。
"""

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hei_robot_localization.mid360s_lio_params import launch_notice, load_and_validate

#: 离线实验强制隔离的 domain（任务规定）
OFFLINE_DOMAIN_ID = "88"
#: 回放前留给 FAST-LIO 初始化订阅的时间
PLAYER_DELAY_S = "5"


def _fail(message: str) -> None:
    raise RuntimeError(
        "\n" + "=" * 72 + "\n离线 LIO 启动被拒绝\n" + message + "\n" + "=" * 72 + "\n"
    )


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001
    cfg = context.launch_configurations

    # ---- 1) 未标定必须显式 opt-in
    uncal = str(cfg.get("uncalibrated", "")).strip().lower()
    if uncal not in ("true", "1", "yes"):
        _fail(
            "外参只有 Livox Mid-360S 手册的名义值（t_IL = [-0.011, -0.02329, +0.04412] m，"
            "R_IL = I），现场未标定。\n"
            "  本 launch 要求显式声明你知道这一点：\n"
            "    uncalibrated:=true\n"
            "  并把结果只当作 uncalibrated 离线实验，不能当作已标定 LIO。"
        )

    # ---- 2) 离线隔离
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    localhost = os.environ.get("ROS_LOCALHOST_ONLY", "")
    if domain != OFFLINE_DOMAIN_ID or localhost != "1":
        _fail(
            "本 launch 只允许离线隔离运行：\n"
            f"  ROS_DOMAIN_ID      = {domain!r}（要求 {OFFLINE_DOMAIN_ID!r}）\n"
            f"  ROS_LOCALHOST_ONLY = {localhost!r}（要求 '1'）\n"
            "请在新终端执行：\n"
            f"  export ROS_DOMAIN_ID={OFFLINE_DOMAIN_ID}\n"
            "  export ROS_LOCALHOST_ONLY=1"
        )

    # ---- 3) bag 必须是已有的原始包
    bag_path = str(cfg.get("bag_path", "")).strip()
    if not bag_path:
        _fail("缺少必填参数 bag_path（要回放的原始 MID-360S rosbag 目录）。")
    bag_dir = Path(bag_path).expanduser()
    if not bag_dir.is_dir():
        _fail(f"bag_path 不是目录：{bag_dir}")
    if not (bag_dir / "metadata.yaml").is_file():
        _fail(f"{bag_dir} 缺少 metadata.yaml，不像 rosbag2 目录")
    bag_path = str(bag_dir.resolve())

    # ---- 4) 回放参数
    try:
        rate = float(cfg.get("rate", "1.0"))
    except ValueError:
        _fail("rate 必须是数字（> 0）")
    if not math.isfinite(rate) or not (rate > 0.0):
        _fail(f"rate 必须是有限正数，实际 {rate!r}")
    try:
        start_offset = float(cfg.get("start_offset", "0.0"))
    except ValueError:
        _fail("start_offset 必须是数字（秒）")
    if not math.isfinite(start_offset) or start_offset < 0.0:
        _fail(f"start_offset 必须是有限非负数，实际 {start_offset!r}")

    # ---- 5) 参数文件必须符合实验契约
    config_file = str(cfg.get("config_file", "")).strip()
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory("hei_robot_localization"),
            "config",
            "mid360s_fastlio_uncalibrated.yaml",
        )
    try:
        load_and_validate(config_file)
    except ValueError as exc:
        _fail(f"参数文件不符合实验契约：{exc}")

    actions = [
        LogInfo(msg=launch_notice()),
        LogInfo(
            msg=(
                "[hei_robot_localization] 离线回放\n"
                f"  bag        : {bag_path}\n"
                f"  rate       : {rate}    start_offset: {start_offset}s\n"
                f"  参数文件   : {config_file}\n"
                f"  ROS_DOMAIN_ID={domain}  ROS_LOCALHOST_ONLY={localhost}"
            )
        ),
        # 唯一 LIO 进程；TF 发布者就是它（camera_init -> body）
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[config_file, {"use_sim_time": False}],
        ),
    ]

    if str(cfg.get("audit", "true")).strip().lower() in ("true", "1", "yes"):
        audit_params = {
            "window_seconds": float(cfg.get("audit_seconds", "10.0")),
            "report_path": str(cfg.get("audit_report", "")).strip(),
        }
        actions.append(
            Node(
                package="hei_robot_localization",
                executable="lio_input_audit",
                name="lio_input_audit",
                output="screen",
                parameters=[audit_params],
            )
        )

    if str(cfg.get("play_bag", "true")).strip().lower() in ("true", "1", "yes"):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--rate",
                    str(rate),
                    "--start-offset",
                    str(start_offset),
                    "--delay",
                    PLAYER_DELAY_S,
                    "--disable-keyboard-controls",
                    "--read-ahead-queue-size",
                    "200",
                ],
                output="screen",
            )
        )

    if str(cfg.get("rviz", "false")).strip().lower() in ("true", "1", "yes"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("fast_lio"), "rviz", "fastlio.rviz"
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                description="必填：要回放的原始 MID-360S rosbag 目录（含 metadata.yaml）",
            ),
            DeclareLaunchArgument(
                "uncalibrated",
                default_value="false",
                description="必填 opt-in：确认外参只是厂商名义值、现场未标定（传 true）",
            ),
            DeclareLaunchArgument("rate", default_value="1.0", description="回放倍速"),
            DeclareLaunchArgument(
                "start_offset", default_value="0.0", description="回放起点（秒）"
            ),
            DeclareLaunchArgument(
                "play_bag", default_value="true", description="是否在 launch 内启动 ros2 bag play"
            ),
            DeclareLaunchArgument(
                "audit", default_value="true", description="是否启动只读输入审计节点"
            ),
            DeclareLaunchArgument(
                "audit_seconds", default_value="10.0", description="审计统计窗口（秒）"
            ),
            DeclareLaunchArgument(
                "audit_report",
                default_value="",
                description="审计 JSON 输出路径（空=只打日志）",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="FAST-LIO 参数文件（空=包内 uncalibrated 参数）",
            ),
            DeclareLaunchArgument("rviz", default_value="false", description="是否启动 RViz"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
