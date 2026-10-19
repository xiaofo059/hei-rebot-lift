"""launch 侧共用逻辑（纯 Python，不 import launch，便于无 ROS 环境回归）。

只放三件事：统一的失败横幅、运行目录/参数文件解析、launch_info.json 取证落盘。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Mapping

from hei_robot_slam import slam_contract as C


def fail(title: str, message: str) -> None:
    """以清晰中文信息中止 launch（不做“尽力启动”）。"""
    raise RuntimeError("\n" + "=" * 72 + f"\n{title}\n{message}\n" + "=" * 72 + "\n")


def require_offline_isolation() -> None:
    try:
        C.check_offline_isolation(os.environ)
    except C.SlamContractError as exc:
        fail("离线三维 SLAM 启动被拒绝（隔离条件不满足）", str(exc))


def parse_replay_arguments(cfg: Mapping[str, str]) -> tuple[float, float]:
    """解析并校验 rate / start_offset（有限、rate>0、offset>=0）。"""
    try:
        rate = float(cfg.get("rate", "1.0"))
    except (TypeError, ValueError):
        fail("离线回放参数非法", f"rate 必须是数字，收到 {cfg.get('rate')!r}")
    try:
        start_offset = float(cfg.get("start_offset", "0.0"))
    except (TypeError, ValueError):
        fail("离线回放参数非法", f"start_offset 必须是数字，收到 {cfg.get('start_offset')!r}")
    if not math.isfinite(rate) or rate <= 0.0:
        fail("离线回放参数非法", f"rate 必须是有限正数，收到 {rate!r}")
    if not math.isfinite(start_offset) or start_offset < 0.0:
        fail("离线回放参数非法", f"start_offset 必须是有限非负数，收到 {start_offset!r}")
    return rate, start_offset


def resolve_bag_and_run_dir(cfg: Mapping[str, str]) -> tuple[Path, Path, Path]:
    """校验 bag、创建唯一运行目录、返回 (bag_dir, run_dir, db_path)。"""
    bag_raw = str(cfg.get("bag_path", "")).strip()
    if not bag_raw:
        fail("缺少必填参数 bag_path", "请传要回放的原始 MID-360S rosbag 目录（含 metadata.yaml）")
    try:
        bag_dir = C.check_bag_dir(bag_raw)
    except C.SlamContractError as exc:
        fail("bag 目录不可用", str(exc))

    output_root = str(cfg.get("output_root", str(C.DATA_ROOT))).strip() or str(C.DATA_ROOT)
    session = str(cfg.get("session", "")).strip()
    try:
        run_dir = C.resolve_run_dir(output_root, session or None)
    except C.SlamContractError as exc:
        fail("无法创建唯一运行输出目录", str(exc))

    db_path = C.mapping_database_path(run_dir)
    try:
        C.validate_mapping_database_target(db_path)
    except C.SlamContractError as exc:
        fail("建图数据库路径不安全", str(exc))
    return bag_dir, run_dir, db_path


def resolve_localization_inputs(cfg: Mapping[str, str]) -> tuple[Path, Path, Path]:
    """校验 bag、已存在 DB（只读）并创建唯一运行目录。"""
    bag_raw = str(cfg.get("bag_path", "")).strip()
    if not bag_raw:
        fail("缺少必填参数 bag_path", "定位冒烟需要回放已有 bag")
    try:
        bag_dir = C.check_bag_dir(bag_raw)
    except C.SlamContractError as exc:
        fail("bag 目录不可用", str(exc))

    db_raw = str(cfg.get("database_path", "")).strip()
    if not db_raw:
        fail("缺少必填参数 database_path", "定位模式必须显式指定已存在的建图数据库（只读）")
    try:
        db_path = C.validate_localization_database(db_raw)
    except C.SlamContractError as exc:
        fail("定位数据库不可用", str(exc))

    output_root = str(cfg.get("output_root", str(C.DATA_ROOT))).strip() or str(C.DATA_ROOT)
    session = str(cfg.get("session", "")).strip()
    try:
        run_dir = C.resolve_run_dir(output_root, session or None)
    except C.SlamContractError as exc:
        fail("无法创建唯一运行输出目录", str(exc))
    return bag_dir, db_path, run_dir


def resolve_live_localization_inputs(cfg: Mapping[str, str]) -> tuple[Path, Path]:
    """校验实时只读定位的已有 DB，并创建唯一日志/探针目录。"""
    db_raw = str(cfg.get("database_path", "")).strip()
    if not db_raw:
        fail("缺少必填参数 database_path", "实时定位必须显式指定已存在的建图数据库（只读）")
    try:
        db_path = C.validate_localization_database(db_raw)
    except C.SlamContractError as exc:
        fail("定位数据库不可用", str(exc))

    output_root = str(cfg.get("output_root", str(C.DATA_ROOT))).strip() or str(C.DATA_ROOT)
    session = str(cfg.get("session", "")).strip()
    try:
        run_dir = C.resolve_run_dir(output_root, session or None)
    except C.SlamContractError as exc:
        fail("无法创建唯一运行输出目录", str(exc))
    return db_path, run_dir


def require_uncalibrated_optin(cfg: Mapping[str, str]) -> None:
    """外参只有厂商名义值：必须显式 uncalibrated:=true。"""
    value = str(cfg.get("uncalibrated", "")).strip().lower()
    if value not in ("true", "1", "yes"):
        fail(
            "外参未标定，必须显式 opt-in",
            "外参只有 Livox Mid-360S 手册名义值（t_IL=[-0.011,-0.02329,+0.04412] m，R_IL=I），"
            "\n现场未标定。请显式传 uncalibrated:=true，且结果只能当作 uncalibrated 实验。",
        )


def resolve_param_file(cfg: Mapping[str, str], share_dir: str, mode: str) -> Path:
    """定位 RTAB-Map 参数文件（默认用包内版本），并做模式契约校验。"""
    override = str(cfg.get("rtabmap_params_file", "")).strip()
    path = Path(override).expanduser() if override else Path(share_dir) / "config" / C.DEFAULT_PARAM_FILES[mode]
    if not path.is_file():
        fail("RTAB-Map 参数文件不存在", f"mode={mode} 期望 {path}")
    try:
        params = C.load_param_yaml(path)
        C.validate_parameters_for_mode(params, mode)
    except (C.SlamContractError, ValueError) as exc:
        fail("RTAB-Map 参数不满足契约", str(exc))
    return path


def write_launch_info(run_dir: str | os.PathLike[str], payload: dict[str, Any]) -> Path:
    """把本次 launch 的输入/契约摘要写到运行目录（证据，不含大文件）。"""
    target = Path(run_dir) / "launch_info.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    tmp.replace(target)
    return target


def build_bag_play_command(bag_dir: str | os.PathLike[str], rate: float, start_offset: float) -> list[str]:
    return [
        "ros2",
        "bag",
        "play",
        str(bag_dir),
        "--rate",
        f"{rate:g}",
        "--start-offset",
        f"{start_offset:g}",
        "--delay",
        C.BAG_PLAYER_DELAY_S,
        "--clock",
        C.SIM_CLOCK_HZ,
        "--disable-keyboard-controls",
        "--read-ahead-queue-size",
        "200",
    ]
