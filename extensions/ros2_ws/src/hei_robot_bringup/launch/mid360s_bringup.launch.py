"""MID-360S 启动（自研 bringup，单一 driver 实例）。

设计约束（任务 HEI-LIDAR-20261016-02-PREP 步骤 d）：
  * ``config_path`` 为**必填**参数，没有默认值：现场 IP 不进仓库、也不写死在 launch 里；
  * 缺少或非法时，launch 明确失败并指出缺少哪个现场参数、如何生成；
  * 默认 10 Hz、``sensor_msgs/PointCloud2``；可选 Livox ``CustomMsg``；RViz 可选；
  * 只有**一个** driver 实例；
  * 原始帧 ``lidar_link``；**不发布** base_link / IMU 外参（未实测的变换一律不发）。

已知上游限制（未修改第三方源码）：
  ``livox_ros_driver2`` 1.2.6 的 ``src/lddc.cpp:481`` 把 IMU 帧 ID 硬编码为
  ``livox_frame``，不受本 launch 的 ``frame_id`` 影响；点云/CustomMsg 才使用
  ``frame_id``。因此 IMU 消息的 ``frame_id`` 仍是 ``livox_frame``。本 launch
  不会用 identity TF 把它“补”成 lidar_link——那是未经测量的外参。
  下游（如 FAST-LIO2）应显式配置 lidar→IMU 外参，或另行评估上游补丁。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from hei_robot_bringup.mid360s_config import (
    DEFAULT_FRAME_ID,
    DEFAULT_IMU_TOPIC,
    DEFAULT_LIDAR_TOPIC,
    IMU_FRAME_ID_UPSTREAM,
    XFER_FORMAT_BY_NAME,
    ConfigError,
    load_and_validate,
)

#: driver 参数范围（src/livox_ros_driver2.cpp:74-81 会把频率夹到 [0.5, 100]）
PUBLISH_FREQ_MIN = 0.5
PUBLISH_FREQ_MAX = 100.0


def _fail(message: str) -> None:
    """以清晰的中文信息中止 launch。"""
    raise RuntimeError("\n" + "=" * 72 + f"\nMID-360S bringup 启动失败\n{message}\n" + "=" * 72 + "\n")


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001
    config_path = context.launch_configurations.get("config_path", "").strip()
    if not config_path:
        _fail(
            "缺少必填现场参数 config_path。\n"
            "  本 launch 没有任何示例 IP 默认值，必须先按现场实测地址生成配置：\n"
            "    ros2 run hei_robot_bringup mid360s_make_config \\\n"
            "        --host-ip <本机有线网卡IP> --lidar-ip <雷达IP> \\\n"
            "        --output extensions/config/local/mid360s_config.json\n"
            "  然后：\n"
            "    ros2 launch hei_robot_bringup mid360s_bringup.launch.py \\\n"
            "        config_path:=extensions/config/local/mid360s_config.json"
        )

    try:
        field = load_and_validate(config_path)
    except ConfigError as exc:
        _fail(f"现场配置不可用：\n  {exc}")

    fmt = context.launch_configurations.get("pointcloud_format", "pointcloud2").strip()
    if fmt not in XFER_FORMAT_BY_NAME:
        _fail(
            f"pointcloud_format='{fmt}' 非法；可选：{', '.join(sorted(XFER_FORMAT_BY_NAME))}\n"
            "  pointcloud2 = sensor_msgs/PointCloud2（默认，xfer_format=0）\n"
            "  custom      = livox_ros_driver2/CustomMsg（xfer_format=1，含逐点 offset_time）"
        )

    try:
        freq = float(context.launch_configurations.get("publish_freq", "10.0"))
    except ValueError:
        _fail("publish_freq 必须是数字（Hz）")
    if not (PUBLISH_FREQ_MIN <= freq <= PUBLISH_FREQ_MAX):
        _fail(
            f"publish_freq={freq} 超出允许范围 [{PUBLISH_FREQ_MIN}, {PUBLISH_FREQ_MAX}]；"
            "driver 会静默夹取，故在此直接拒绝。"
        )

    frame_id = context.launch_configurations.get("frame_id", DEFAULT_FRAME_ID).strip()
    if not frame_id:
        _fail("frame_id 不能为空")

    driver_params = {
        "xfer_format": XFER_FORMAT_BY_NAME[fmt],
        "multi_topic": 0,        # 0 = 所有雷达共用 /livox/lidar 与 /livox/imu
        "data_src": 0,           # 0 = kSourceRawLidar
        "publish_freq": freq,
        "output_data_type": 0,   # 0 = kOutputToRos
        "frame_id": frame_id,
        "user_config_path": str(field.path),
    }

    actions = [
        LogInfo(
            msg=(
                f"[hei_robot_bringup] MID-360S bringup\n"
                f"  现场配置 : {field.path}\n"
                f"  host_ip  : {field.host_ip}\n"
                f"  lidar_ip : {field.lidar_ip}\n"
                f"  点云格式 : {fmt} (xfer_format={XFER_FORMAT_BY_NAME[fmt]})\n"
                f"  发布频率 : {freq} Hz\n"
                f"  点云帧   : {frame_id}   话题 {DEFAULT_LIDAR_TOPIC}\n"
                f"  IMU 帧   : {IMU_FRAME_ID_UPSTREAM}（上游硬编码，非本 launch 的 frame_id）"
                f"   话题 {DEFAULT_IMU_TOPIC}\n"
                "  注意：未发布任何 base_link/IMU 外参 TF。"
            )
        ),
        # 唯一的 driver 实例
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[driver_params],
        ),
    ]

    rviz_config = context.launch_configurations.get("rviz_config", "").strip()
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=(["-d", rviz_config] if rviz_config else []),
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    # 注意：这里不能用 FindPackageShare(...).perform(None)——在没有 launch context 时
    # （例如 `ros2 launch <pkg> <file> --show-args`）会抛 AttributeError。
    # 用 PathJoinSubstitution 交给 launch 惰性解析。
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("hei_robot_bringup"), "rviz", "mid360s.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                description=(
                    "必填：现场配置 JSON 路径（含 host_ip/lidar_ip）。无默认值，"
                    "用 mid360s_make_config 生成。"
                ),
            ),
            DeclareLaunchArgument(
                "publish_freq",
                default_value="10.0",
                description="点云发布频率 Hz（0.5–100，默认 10）",
            ),
            DeclareLaunchArgument(
                "pointcloud_format",
                default_value="pointcloud2",
                description="pointcloud2（默认）或 custom（Livox CustomMsg，含逐点 offset_time）",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=DEFAULT_FRAME_ID,
                description=f"原始点云帧 ID（默认 {DEFAULT_FRAME_ID}）",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="是否同时启动 RViz2（默认 false）",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 配置路径（Fixed Frame = lidar_link）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
