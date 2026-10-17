"""MID-360S 现场配置的生成与校验（单一事实来源）。

本模块同时被以下入口使用，避免端口结构或校验规则出现多份真相：
  * ``hei_robot_bringup.make_config``（配置生成 CLI）
  * ``launch/mid360s_bringup.launch.py``（启动前校验现场配置）
  * ``hei_robot_bringup.bag_qc``（仅使用话题/类型常量）

端口结构与 ``livox_ros_driver2`` 1.2.6 自带的 ``config/MID360s_config.json``
完全一致（S 型使用 ``Mid360s`` 顶层键与数组形式的 ``host_net_info``）。
本模块**不包含任何示例 IP**：现场地址必须由使用者显式提供。

依据（冻结版本 livox_ros_driver2 1.2.6）：
  * config/MID360s_config.json           —— 端口与键结构
  * src/lddc.cpp:481                     —— IMU 帧 ID 被上游硬编码（见 IMU_FRAME_ID_UPSTREAM）
  * src/livox_ros_driver2.cpp:57,136-142 —— 参数名与默认值
"""

from __future__ import annotations

import ipaddress
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# --------------------------------------------------------------------------
# 官方常量（与 livox_ros_driver2 1.2.6 config/MID360s_config.json 一致）
# --------------------------------------------------------------------------

#: driver 侧收包端口（雷达 → host），S 型与 MID360 相同
LIDAR_NET_PORTS: dict[str, int] = {
    "cmd_data_port": 56100,
    "push_msg_port": 56200,
    "point_data_port": 56300,
    "imu_data_port": 56400,
    "log_data_port": 56500,
}

#: host 侧收包端口（S 型官方结构：host_net_info 为数组，逐项含 host_ip）
HOST_NET_PORTS: dict[str, int] = {
    "cmd_data_port": 56101,
    "push_msg_port": 56201,
    "point_data_port": 56301,
    "imu_data_port": 56401,
    "log_data_port": 56501,
}

#: lidar_summary_info.lidar_type。官方 MID360 与 MID360s 配置均写 8；
#: 它是 driver 内部的位标志（src/lds_lidar.cpp:107 ``lidar_type & kLivoxLidarType``），
#: **不是** SDK 的 LivoxLidarDeviceType 枚举值（S 型在 SDK 中为 35）。不要改成 35。
LIDAR_SUMMARY_TYPE = 8

#: 原始点云外参保持 identity：这是雷达内部输出点云的出厂外参字段，
#: 不是 base_link/IMU 的实测外参。base/imu 外参必须另行标定，本任务不发布。
IDENTITY_EXTRINSIC: dict[str, float] = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

#: driver 发布话题（multi_topic=0 时，见 src/lddc.cpp:580,630）
DEFAULT_LIDAR_TOPIC = "/livox/lidar"
DEFAULT_IMU_TOPIC = "/livox/imu"

#: 默认点云格式：sensor_msgs/msg/PointCloud2（xfer_format=0）
POINTCLOUD2_TYPE = "sensor_msgs/msg/PointCloud2"
#: 可选 Livox 自定义格式（xfer_format=1）
CUSTOMMSG_TYPE = "livox_ros_driver2/msg/CustomMsg"
IMU_TYPE = "sensor_msgs/msg/Imu"

#: 参数名 -> xfer_format 取值
XFER_FORMAT_BY_NAME: dict[str, int] = {"pointcloud2": 0, "custom": 1}

#: MID-360S 原始点云帧 ID（本任务约定，不发布 base/imu 外参）
DEFAULT_FRAME_ID = "lidar_link"

#: 上游事实：src/lddc.cpp:481 ``imu_msg.header.frame_id = "livox_frame"`` 为硬编码，
#: 不受 frame_id 参数影响；点云（lddc.cpp:263,355,421）才使用 frame_id。
#: 因此 IMU 消息的 frame 与**配置的点云 frame** 不同。
#: 注意：frame_id 与 topic 名称本来就无需同名，这不是“frame 与 topic 不一致”的缺陷。
#: 下游按 frame 解析 TF 时必须显式处理该差异。
IMU_FRAME_ID_UPSTREAM = "livox_frame"

# --------------------------------------------------------------------------
# 上游 IMU 单位与字段事实（driver 1.2.6 冻结源码核对，2026-10-16）
#
#   * src/comm/pub_handler.cpp:119-124 —— gyro/acc 从原始报文**原样拷贝**，无任何换算；
#   * src/lddc.cpp:489-494 —— 原样写入 sensor_msgs/Imu，且**不设置**
#     orientation / covariance（全仓库 src/ 无 orientation|covariance 赋值）；
#   * src/comm/lidar_imu_data_queue.h:37-42 —— driver 自述 acc 单位 **g**、
#     gyro 单位 **rad/s**（注释注明 "Based on the IMU Data Type in Livox
#     communication protocol"）。
#
# 结论：原始 bag 中 sensor_msgs/Imu.linear_acceleration 是 **g**，不是 SI 的 m/s²；
# orientation 恒为默认值，不能当作有效姿态。
# 本任务只录原始值，**不做** SI 转换（转换属后续阶段的显式设计决策）。
# --------------------------------------------------------------------------
IMU_ANGULAR_VELOCITY_UNIT = "rad/s"
IMU_LINEAR_ACCELERATION_UNIT = "g"
IMU_LINEAR_ACCELERATION_IS_SI = False
IMU_ORIENTATION_PUBLISHED = False
#: 官方协议参考（由 Codex 提供）：MID-360 系列以太网协议
IMU_PROTOCOL_REFERENCE = (
    "https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/"
    "livox_eth_protocol_mid360.html"
)


class ConfigError(ValueError):
    """现场配置或命令行输入不合法。"""


# --------------------------------------------------------------------------
# IPv4 校验
# --------------------------------------------------------------------------


def validate_ipv4(value: str, *, field: str) -> str:
    """校验一个必须可用的 IPv4 主机地址，返回规范化字符串。

    拒绝：非 IPv4、组播、未指定(0.0.0.0)、广播(255.255.255.255)、环回。
    这些地址都不是“本机某网卡的可路由单播地址”，配进 driver 只会静默收不到数据。
    """
    if value is None or str(value).strip() == "":
        raise ConfigError(f"{field} 不能为空；现场 IP 没有默认值，必须显式提供")
    text = str(value).strip()
    try:
        addr = ipaddress.IPv4Address(text)
    except ipaddress.AddressValueError as exc:
        raise ConfigError(f"{field}='{text}' 不是合法的 IPv4 地址（{exc}）") from exc

    if addr.is_multicast:
        raise ConfigError(f"{field}='{text}' 是组播地址，不能作为雷达/主机单播地址")
    if addr.is_unspecified:
        raise ConfigError(f"{field}='{text}' 是未指定地址 0.0.0.0，必须写明真实网卡地址")
    if text == "255.255.255.255":
        raise ConfigError(f"{field}='{text}' 是广播地址，不能作为单播地址")
    if addr.is_loopback:
        raise ConfigError(f"{field}='{text}' 是环回地址，雷达链路无法通过环回通信")
    return text


def validate_pair(host_ip: str, lidar_ip: str) -> tuple[str, str]:
    """校验 host/lidar 两个地址及它们的组合关系。"""
    host = validate_ipv4(host_ip, field="host_ip")
    lidar = validate_ipv4(lidar_ip, field="lidar_ip")
    if host == lidar:
        raise ConfigError(f"host_ip 与 lidar_ip 不能相同（都是 {host}）")
    return host, lidar


# --------------------------------------------------------------------------
# 配置构造
# --------------------------------------------------------------------------


def build_config(host_ip: str, lidar_ip: str) -> dict[str, Any]:
    """构造 MID-360S 现场配置字典（结构等同官方 MID360s_config.json）。"""
    host, lidar = validate_pair(host_ip, lidar_ip)
    return {
        "lidar_summary_info": {"lidar_type": LIDAR_SUMMARY_TYPE},
        "Mid360s": {
            "lidar_net_info": dict(LIDAR_NET_PORTS),
            "host_net_info": [{"host_ip": host, **HOST_NET_PORTS}],
        },
        "lidar_configs": [
            {
                "ip": lidar,
                "pcl_data_type": 1,
                "pattern_mode": 0,
                "extrinsic_parameter": dict(IDENTITY_EXTRINSIC),
            }
        ],
    }


def dumps_config(config: dict[str, Any]) -> str:
    """序列化配置（UTF-8、缩进 2、结尾换行，便于 diff 与人工核对）。"""
    return json.dumps(config, indent=2, ensure_ascii=False) + "\n"


# --------------------------------------------------------------------------
# 配置读取与校验（launch 使用）
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class FieldConfig:
    """从现场 JSON 中提取的关键字段。"""

    path: Path
    host_ip: str
    lidar_ip: str
    lidar_type: int


def load_and_validate(path: str | Path) -> FieldConfig:
    """读取并校验现场配置；任何问题都抛出带可操作提示的 ConfigError。"""
    p = Path(path).expanduser()
    if not p.is_file():
        raise ConfigError(
            f"现场配置文件不存在：{p}\n"
            "  请先由操作者生成（不要手抄示例 IP）：\n"
            "    ros2 run hei_robot_bringup mid360s_make_config \\\n"
            "        --host-ip <本机有线网卡IP> --lidar-ip <雷达IP> \\\n"
            "        --output extensions/config/local/mid360s_config.json"
        )
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"现场配置不是合法 JSON：{p}（{exc}）") from exc

    if not isinstance(data, dict):
        raise ConfigError(f"现场配置顶层必须是 JSON 对象：{p}")

    if "Mid360s" not in data:
        keys = ", ".join(sorted(data.keys())) or "(空)"
        raise ConfigError(
            f"现场配置缺少 'Mid360s' 顶层键（当前键：{keys}）。\n"
            "  注意：MID360（非 S）配置使用 'MID360' 键，不能混用；请用本包的生成命令重建。"
        )

    block = data["Mid360s"]
    if not isinstance(block, dict):
        raise ConfigError("'Mid360s' 必须是对象")
    host_info = block.get("host_net_info")
    if not isinstance(host_info, list) or not host_info:
        raise ConfigError(
            "'Mid360s.host_net_info' 必须是至少含一项的数组（S 型结构）。\n"
            "  MID360（非 S）用的是对象形式，套用会静默收不到点云。"
        )
    host_ip = host_info[0].get("host_ip") if isinstance(host_info[0], dict) else None
    if not host_ip:
        raise ConfigError("'Mid360s.host_net_info[0].host_ip' 缺失；该字段为必填")

    lidar_cfgs = data.get("lidar_configs")
    if not isinstance(lidar_cfgs, list) or not lidar_cfgs:
        raise ConfigError("'lidar_configs' 必须是至少含一项的数组")
    lidar_ip = lidar_cfgs[0].get("ip") if isinstance(lidar_cfgs[0], dict) else None
    if not lidar_ip:
        raise ConfigError("'lidar_configs[0].ip' 缺失；该字段为必填")

    # 复用同一套地址校验：现场手改过的配置同样要被拦住
    host_ip, lidar_ip = validate_pair(str(host_ip), str(lidar_ip))

    lidar_type = data.get("lidar_summary_info", {}).get("lidar_type", LIDAR_SUMMARY_TYPE)
    if lidar_type != LIDAR_SUMMARY_TYPE:
        raise ConfigError(
            f"lidar_summary_info.lidar_type={lidar_type!r}，官方 MID360/MID360s 配置均为 "
            f"{LIDAR_SUMMARY_TYPE}；它是 driver 内部位标志，不要写成 SDK 的 S 型枚举 35。"
        )

    return FieldConfig(path=p.resolve(), host_ip=host_ip, lidar_ip=lidar_ip, lidar_type=lidar_type)
