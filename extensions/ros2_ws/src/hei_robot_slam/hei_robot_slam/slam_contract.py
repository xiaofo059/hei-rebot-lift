"""HEI ReBot Lift MID-360S 三维 SLAM 契约（RTAB-Map 0.23.13 / ROS 2 Humble）。

本模块是**唯一事实来源**，只做纯函数与常量，不 import ROS，便于无 ROS 环境回归。
launch / CLI / 测试都必须调用这里，禁止各自复制一份 frame、模式或 DB 规则。

契约要点（与任务 HEI-LIDAR-04-SLAM-LAPTOP-BASE-20261019 对应）：

* frame：前端 FAST-LIO 独占 ``camera_init -> body``；后端 RTAB-Map 独占
  ``map -> camera_init``。本包**不发布任何 TF**，也不伪造 ``base_link`` / ``odom``。
* 输入：RTAB-Map 只吃与 ``/Odometry`` 同 stamp 的去畸变单帧 body 点云，不吃累计地图、
  不重复去畸变；点云先经最薄 XYZI 字段适配（见 ``cloud_xyzi``），避免把 FAST-LIO 未估计的
  零法线字段当成有效法线。
* 单一里程计来源：只用 FAST-LIO ``/Odometry``（经 ``odom_frame_id=camera_init`` 的 TF），
  不启动 rtabmap_odom / ICP odometry 节点。
* 模式：``mapping`` 必须写**全新**数据库（禁止 -d / delete_db_on_start）；
  ``localization`` 必须指向**已存在**数据库并设 ``Mem/LocalizationReadOnly=true``，
  运行前后 SHA256 必须一致。
* 离线隔离：``ROS_DOMAIN_ID=89`` 且 ``ROS_LOCALHOST_ONLY=1``（前端/后端同域）。
"""

from __future__ import annotations

import hashlib
import os
import time
from pathlib import Path
from typing import Any, Mapping

# ---------------------------------------------------------------------------
# 版本锁定（本机实测：ROS 2 Humble apt 包，见 docs/MID360S_SLAM.md 依赖证据）
# ---------------------------------------------------------------------------
RTABMAP_VERSION = "0.23.13"
RTABMAP_ROS_DEB_PIN = "0.23.13-1jammy.20261011.111917"  # ros-humble-rtabmap-slam=
RTABMAP_PKG = "rtabmap_slam"
RTABMAP_EXECUTABLE = "rtabmap"

# ---------------------------------------------------------------------------
# frame / topic 契约（禁止在别处另写一份）
# ---------------------------------------------------------------------------
MAP_FRAME = "map"
ODOM_FRAME = "camera_init"
BASE_FRAME = "body"

LIO_ODOM_TOPIC = "/Odometry"
LIO_BODY_CLOUD_TOPIC = "/cloud_registered_body"
SLAM_CLOUD_TOPIC = "/slam/lidar_cloud_xyzi"
RTABMAP_NODE_NAME = "rtabmap"

#: 前端唯一允许的 TF 对（由 FAST-LIO 发布，本包不得重复发布）
LIO_TF_PAIR = (ODOM_FRAME, BASE_FRAME)
#: 后端唯一允许新增的 TF 对（由 rtabmap_slam 发布）
BACKEND_TF_PAIR = (MAP_FRAME, ODOM_FRAME)
#: 本任务明确不存在的 frame（出现即视为契约破坏）
FORBIDDEN_FRAMES = ("base_link", "odom", "base_footprint")

#: RTAB-Map 接收点云必须携带的字段（intensity 为 MID-360S 反射率，保留语义）
CLOUD_SOURCE_FIELDS = ("x", "y", "z", "intensity")
CLOUD_ADAPTED_FIELDS = ("x", "y", "z", "intensity")
#: FAST-LIO 打包但并未估计的法线字段：适配层必须丢弃，不能让 RTAB 误信零法线
UNESTIMATED_NORMAL_FIELDS = ("normal_x", "normal_y", "normal_z", "curvature")

# ---------------------------------------------------------------------------
# 输出目录 / 模式 / 离线隔离
# ---------------------------------------------------------------------------
DATA_ROOT = Path("/home/robo/hei-data/mid360s_slam_20261019")
OFFLINE_DOMAIN_ID = "89"
OFFLINE_LOCALHOST_ONLY = "1"
BAG_PLAYER_DELAY_S = "5"
#: 离线回放：bag player 用 --clock 100 发布 /clock，全链路 use_sim_time=true
SIM_CLOCK_HZ = "100"

MODE_MAPPING = "mapping"
MODE_LOCALIZATION = "localization"
MODES = (MODE_MAPPING, MODE_LOCALIZATION)

DEFAULT_PARAM_FILES = {
    MODE_MAPPING: "rtabmap_3d_lidar_mapping.yaml",
    MODE_LOCALIZATION: "rtabmap_3d_lidar_localization.yaml",
}

#: 建图模式数据库文件名（每次运行放在唯一 run 目录下，绝不复用）
MAPPING_DB_NAME = "rtabmap_mapping.db"

#: 任何情况下都不允许出现的自动删除数据库参数/CLI 片段
FORBIDDEN_DB_TOKENS = ("delete_db_on_start", "--delete_db_on_start", " -d ", "-d ");


class SlamContractError(ValueError):
    """契约不满足。launch / CLI 必须以此拒绝启动（不做“尽力启动”）。"""


# ---------------------------------------------------------------------------
# 文件系统与取证工具
# ---------------------------------------------------------------------------
def sha256_file(path: str | os.PathLike[str]) -> str:
    """流式 SHA256，用于 DB/PCD 运行前后一致性取证。"""
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _require_absolute(path: str | os.PathLike[str], what: str) -> Path:
    p = Path(path).expanduser()
    if not p.is_absolute():
        raise SlamContractError(f"{what}必须是绝对路径，收到 {str(path)!r}")
    return p


def check_bag_dir(path: str | os.PathLike[str]) -> Path:
    """只接受**已存在**的 rosbag2 目录（含 metadata.yaml），不做任何写入。"""
    bag = _require_absolute(path, "bag_path ").resolve()
    if not bag.is_dir():
        raise SlamContractError(f"bag_path 不是目录：{bag}")
    if not (bag / "metadata.yaml").is_file():
        raise SlamContractError(f"{bag} 缺少 metadata.yaml，不像 rosbag2 目录")
    return bag


def unique_session_stamp(now: float | None = None) -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime(now if now is not None else time.time()))


def resolve_run_dir(
    output_root: str | os.PathLike[str],
    session: str | None = None,
    now: float | None = None,
) -> Path:
    """为一次运行创建**全新唯一**输出目录；已存在则拒绝（绝不覆盖/复用）。

    目录结构：``<output_root>/<session>/``，默认 session 为本地时间戳。
    """
    root = _require_absolute(output_root, "output_root ").resolve()
    name = session.strip() if isinstance(session, str) else ""
    if not name:
        name = unique_session_stamp(now)
    if name in (".", "..") or "/" in name or "\\" in name:
        raise SlamContractError(f"session 必须是单层目录名，收到 {name!r}")
    run_dir = root / name
    if run_dir.exists():
        raise SlamContractError(
            f"输出目录已存在，拒绝复用/覆盖：{run_dir}\n"
            "  每次运行必须是唯一新目录；请换 session 名或先人工归档旧目录。"
        )
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def mapping_database_path(run_dir: str | os.PathLike[str]) -> Path:
    return Path(run_dir).resolve() / MAPPING_DB_NAME


def validate_mapping_database_target(db_path: str | os.PathLike[str]) -> Path:
    """建图目标数据库必须**不存在**（RTAB-Map 无 -d 时会打开并追加旧库）。"""
    db = _require_absolute(db_path, "database_path ").resolve()
    if db.exists():
        raise SlamContractError(
            f"建图目标数据库已存在，拒绝追加/覆盖：{db}\n"
            "  建图必须写全新 DB（本包不允许 -d / delete_db_on_start）。"
        )
    if not db.parent.is_dir():
        raise SlamContractError(f"数据库父目录不存在：{db.parent}")
    return db


def validate_localization_database(db_path: str | os.PathLike[str]) -> Path:
    """定位数据库必须已存在且非空；本包只读使用（Mem/LocalizationReadOnly=true）。"""
    db = _require_absolute(db_path, "database_path ").resolve()
    if not db.is_file():
        raise SlamContractError(f"定位数据库不存在或不是普通文件：{db}")
    if db.stat().st_size <= 0:
        raise SlamContractError(f"定位数据库是空文件：{db}")
    return db


def check_offline_isolation(environ: Mapping[str, str] | None = None) -> None:
    """离线实验强制隔离：Domain 89 + localhost only（前端/后端必须在同一域）。"""
    env = os.environ if environ is None else environ
    domain = str(env.get("ROS_DOMAIN_ID", ""))
    localhost = str(env.get("ROS_LOCALHOST_ONLY", ""))
    if domain != OFFLINE_DOMAIN_ID or localhost != OFFLINE_LOCALHOST_ONLY:
        raise SlamContractError(
            "本 launch 只允许离线隔离运行（防止与实机/其他会话串流）：\n"
            f"  ROS_DOMAIN_ID      = {domain!r}（要求 {OFFLINE_DOMAIN_ID!r}）\n"
            f"  ROS_LOCALHOST_ONLY = {localhost!r}（要求 {OFFLINE_LOCALHOST_ONLY!r}）\n"
            "请在新终端执行：\n"
            f"  export ROS_DOMAIN_ID={OFFLINE_DOMAIN_ID}\n"
            "  export ROS_LOCALHOST_ONLY=1"
        )


# ---------------------------------------------------------------------------
# 参数校验（ROS wrapper 参数 + RTAB-Map 内部参数，按本机 0.23.13 实测名称）
# ---------------------------------------------------------------------------
def load_param_yaml(path: str | os.PathLike[str]) -> dict[str, Any]:
    """读取 ROS 2 参数文件，返回 ``/**/ros__parameters`` 下的扁平字典。"""
    import yaml  # 延迟导入，纯契约函数不依赖 YAML

    p = Path(path)
    if not p.is_file():
        raise SlamContractError(f"参数文件不存在：{p}")
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SlamContractError(f"参数文件顶层不是映射：{p}")
    root = data.get("/**")
    if not isinstance(root, dict) or not isinstance(root.get("ros__parameters"), dict):
        raise SlamContractError(f"参数文件缺少 /**/ros__parameters 节：{p}")
    params = dict(root["ros__parameters"])
    for key, value in params.items():
        if isinstance(value, dict):
            raise SlamContractError(f"RTAB-Map 参数必须保持扁平键，发现嵌套键 {key!r}")
    return params


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1", "yes")


def _require(params: Mapping[str, Any], key: str) -> Any:
    if key not in params:
        raise SlamContractError(f"缺少必需参数 {key!r}")
    return params[key]


def _require_value(params: Mapping[str, Any], key: str, expected: Any) -> None:
    actual = _require(params, key)
    if isinstance(expected, bool):
        ok = _as_bool(actual) is expected
    else:
        ok = str(actual) == str(expected)
    if not ok:
        raise SlamContractError(f"参数 {key!r} 必须为 {expected!r}，实际 {actual!r}")


def _reject_auto_delete(params: Mapping[str, Any]) -> None:
    for key in ("delete_db_on_start", "database_path_in_memory"):
        if key in params and _as_bool(params[key]):
            raise SlamContractError(f"禁止参数 {key}={params[key]!r}：本包绝不允许自动删除/丢弃数据库")
    if "args" in params:
        args = str(params["args"])
        if "-d" in args.split() or "delete_db_on_start" in args:
            raise SlamContractError(f"禁止在 args 中传自动删除数据库：{args!r}")


def _validate_common_parameters(params: Mapping[str, Any]) -> None:
    _reject_auto_delete(params)

    # frame 契约：三帧必须与 LIO/RTAB 分工完全一致
    _require_value(params, "frame_id", BASE_FRAME)
    _require_value(params, "odom_frame_id", ODOM_FRAME)
    _require_value(params, "map_frame_id", MAP_FRAME)

    # 输入：只用去畸变单帧 3D 点云；无相机、无 2D scan、无第二个里程计来源
    _require_value(params, "subscribe_scan_cloud", True)
    _require_value(params, "subscribe_scan", False)
    for key in ("subscribe_depth", "subscribe_rgb", "subscribe_rgbd", "subscribe_stereo"):
        if key in params:
            _require_value(params, key, False)
    _require_value(params, "subscribe_odom", False)
    _require_value(params, "subscribe_odom_info", False)
    _require_value(params, "odom_sensor_sync", False)

    # 真实 6DoF：禁止任何 3DoF/4DoF 降维，禁止仅按 topic 名假装改 frame
    _require_value(params, "Reg/Force3DoF", "false")
    _require_value(params, "Icp/Force4DoF", "false")

    # 3D 激光几何注册（无图像）：ICP 注册 + point-to-plane
    _require_value(params, "Reg/Strategy", "1")
    _require_value(params, "Icp/Strategy", "1")
    _require_value(params, "Icp/PointToPlane", "true")
    if float(_require(params, "Icp/MaxCorrespondenceDistance")) <= 0.0:
        raise SlamContractError("Icp/MaxCorrespondenceDistance 必须 > 0")

    # 回环策略按官方 lidar3d 例：限制 proximity 搜索，不为了桌面包强行造回环
    _require_value(params, "RGBD/ProximityMaxGraphDepth", "0")   # 0 = 忽略深度上限
    _require_value(params, "RGBD/ProximityPathMaxNeighbors", "1")
    _require_value(params, "RGBD/ProximityAngle", "0")           # 0 = 不设 one-to-one proximity 角度上限
    # 纯激光关键适配：false(默认)时空 guess 会走视觉验证路径（无图像 -> Missing visual features，无法估计）；
    # true 用 odom 相对位姿作为 ICP guess。0.23.13 源码 Rtabmap.cpp:2873 与本机实测确认。
    _require_value(params, "RGBD/ProximityOdomGuess", "true")
    _require_value(params, "RGBD/CreateOccupancyGrid", "false")
    if float(_require(params, "RGBD/LinearUpdate")) <= 0.0:
        raise SlamContractError("RGBD/LinearUpdate 必须 > 0（否则不会新增图节点）")
    if float(_require(params, "RGBD/AngularUpdate")) < 0.0:
        raise SlamContractError("RGBD/AngularUpdate 必须 >= 0")

    # 优化器：默认 GTSAM 6DoF；GravitySigma 只作为弱先验，不把 6DoF 压成 3DoF
    _require_value(params, "Optimizer/Strategy", "2")


def validate_mapping_parameters(params: Mapping[str, Any]) -> None:
    """建图模式参数契约。"""
    _validate_common_parameters(params)
    _require_value(params, "Mem/IncrementalMemory", "true")
    _require_value(params, "Mem/InitWMWithAllNodes", "false")
    _require_value(params, "Mem/LocalizationReadOnly", "false")


def validate_localization_parameters(params: Mapping[str, Any]) -> None:
    """定位模式参数契约：只读原地图，绝不清空/重写。"""
    _validate_common_parameters(params)
    _require_value(params, "Mem/IncrementalMemory", "false")
    _require_value(params, "Mem/InitWMWithAllNodes", "true")
    _require_value(params, "Mem/LocalizationReadOnly", "true")


def validate_parameters_for_mode(params: Mapping[str, Any], mode: str) -> None:
    if mode == MODE_MAPPING:
        validate_mapping_parameters(params)
    elif mode == MODE_LOCALIZATION:
        validate_localization_parameters(params)
    else:
        raise SlamContractError(f"未知模式 {mode!r}，只允许 {MODES}")


# ---------------------------------------------------------------------------
# 取证摘要（写进 launch 日志 / evidence JSON）
# ---------------------------------------------------------------------------
def tf_contract() -> dict[str, Any]:
    return {
        "frontend_publisher": "fastlio_mapping",
        "frontend_tf": list(LIO_TF_PAIR),
        "backend_publisher": "rtabmap_slam/rtabmap",
        "backend_tf": list(BACKEND_TF_PAIR),
        "forbidden_frames": list(FORBIDDEN_FRAMES),
        "no_base_link_no_odom": True,
    }


def cloud_contract() -> dict[str, Any]:
    return {
        "lio_body_cloud_topic": LIO_BODY_CLOUD_TOPIC,
        "adapter_topic": SLAM_CLOUD_TOPIC,
        "required_fields": list(CLOUD_ADAPTED_FIELDS),
        "dropped_unestimated_fields": list(UNESTIMATED_NORMAL_FIELDS),
        "preserve": ["header.stamp", "header.frame_id", "point xyz", "intensity"],
        "no_deskew": True,
        "no_accumulated_map": True,
    }


def versions() -> dict[str, str]:
    return {
        "rtabmap_version": RTABMAP_VERSION,
        "rtabmap_ros_deb_pin": RTABMAP_ROS_DEB_PIN,
        "rtabmap_pkg": RTABMAP_PKG,
    }