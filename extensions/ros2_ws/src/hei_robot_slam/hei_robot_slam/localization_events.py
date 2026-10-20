"""定位连续取证的纯逻辑（不 import ROS，便于无 ROS 单测）。

设计目标（任务 HEI-SLAM-OBS-20261020）：

* 把 RTAB-Map 定位过程中的**后端事件证据**按时间线落盘（JSONL），并维护**累计** summary；
* 区分「当前帧原始字段」与「历史累计」：逐帧 proximity/loop id 为 0 不清空历史 accepted 计数；
* 区分 pending（"Localization was good, but waiting ..."）与 accepted（msg 级 proximity/loop id，
  或后端日志中的实际 link 加入事件）；**不接受**仅凭字样判最终成功；
* 记录 initialpose 事件并做**仅时间关联**（明确不是因果）；
* map -> camera_init 的 TF 首值与变化（平移 m / 四元数角度 rad，q 与 -q 等价），
  相同值重复发布不重复落盘；TF 变化本身**不代表** accepted；
* 后端日志（rtabmap core UWARN 直写 stderr，通常不发布到 /rosout）可用
  backend_log_path 只读 tail 补齐 pending 证据；默认不配置时 summary 明确
  pending 日志可观测性不足；
* 只读、增量、无历史常驻内存：JSONL 每条 flush，summary 只保留累计与最近/最后事件。

依赖：仅 Python 标准库。事件接收时间由调用方传入（wall/monotonic ns），便于测试。
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

SCHEMA_VERSION = "hei.localization_event.v1"
SUMMARY_SCHEMA_VERSION = "hei.localization_summary.v1"

EVENT_INFO = "info"
EVENT_INITIALPOSE = "initialpose"
EVENT_ROSOUT = "rosout"
EVENT_BACKEND_LOG = "backend_log"
EVENT_TF = "tf_map_to_camera_init"
EVENT_LOCALIZATION_POSE = "localization_pose"

SOURCE_INFO = "ros2_info_topic"
SOURCE_INITIALPOSE = "ros2_initialpose_topic"
SOURCE_ROSOUT = "ros2_rosout"
SOURCE_BACKEND_LOG = "backend_log_tail"
SOURCE_TF = "ros2_tf"
SOURCE_LOCALIZATION_POSE = "ros2_localization_pose"

#: 最近事件在内存中保留的最大条数（summary 只保留这些，JSONL 才是全量）
MAX_RECENT_ACCEPTED = 20
MAX_RECENT_PENDING = 20
MAX_RECENT_INITIALPOSE = 20
MAX_RECENT_TF_CHANGES = 20
MAX_RECENT_BACKEND_RELEVANT = 30
MAX_RECENT_ROSOUT_RELEVANT = 30
MAX_RECENT_LOCALIZATION_POSE = 20

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
LOG_TIMESTAMP_RE = re.compile(
    r"\((?P<year>\d{4})-(?P<month>\d{2})-(?P<day>\d{2}) "
    r"(?P<hour>\d{2}):(?P<minute>\d{2}):(?P<second>\d{2})\.(?P<millis>\d{3})\)"
)
NEAREST_ID_RE = re.compile(r"nearest id = (?P<nearest>\d+)")

#: /info stats 中与定位/候选/运动状态相关的键（缺字段记 unknown，不伪造值）
STAT_KEYS = {
    "small_movement": "Memory/Small_movement/",
    "fast_movement": "Memory/Fast_movement/",
    "accepted_hypothesis_id": "Loop/Accepted_hypothesis_id/",
    "loop_id": "Loop/Id/",
    "highest_hypothesis_id": "Loop/Highest_hypothesis_id/",
    "highest_hypothesis_value": "Loop/Highest_hypothesis_value/",
    "hypothesis_ratio": "Loop/Hypothesis_ratio/",
    "rejected_hypothesis": "Loop/RejectedHypothesis/",
    "suppressed_hypothesis_id": "Loop/Suppressed_hypothesis_id/",
    "optimization_iterations": "Loop/Optimization_iterations/",
    "optimization_error": "Loop/Optimization_error/",
    "space_paths": "Proximity/Space_paths/",
    "space_scan_paths_checked": "Proximity/Space_scan_paths_checked/",
    "space_visual_paths_checked": "Proximity/Space_visual_paths_checked/",
    "space_detections_icp_multi": "Proximity/Space_detections_added_icp_multi/",
    "space_detections_icp_global": "Proximity/Space_detections_added_icp_global/",
    "space_detections_visual": "Proximity/Space_detections_added_visually/",
    "space_last_detection_id": "Proximity/Space_last_detection_id/",
    "closest_node_distance_m": "Memory/Closest_node_distance/m",
    "closest_node_angle_rad": "Memory/Closest_node_angle/rad",
    "odom_cache_poses": "Memory/Odom_cache_poses/",
    "odom_cache_links": "Memory/Odom_cache_links/",
}

#: 视为「候选活动」的 count 型 stats 键
CANDIDATE_COUNT_KEYS = (
    "space_detections_icp_multi",
    "space_detections_icp_global",
    "space_detections_visual",
    "space_paths",
    "space_scan_paths_checked",
    "space_visual_paths_checked",
)

#: pending 文本特征（rtabmap core UWARN）
PENDING_TEXT_MARKER = "Localization was good"
PENDING_WAITING_MARKER = "waiting for another one"
REJECTED_TEXT_MARKERS = ("Localization was rejected", "rejected by graph optimization")
MAP_CORRECTION_MARKER = "Update map correction"
INITIALPOSE_LOG_MARKER = "initialpose received"
MAP_LOADED_MARKER = "Localization mode"
READONLY_MARKER = "Database is read-only"
VISUAL_MISSING_MARKER = "Missing visual features"
CLOSURE_ADDED_MARKERS = (
    "[Scan matching] Add local loop closure in SPACE",
    "[Visual] Add local loop closure in SPACE",
)
LINK_ADDED_CANDIDATE_CATEGORY = "candidate_link_added"

RONSOUT_RELEVANT_CATEGORIES = {
    "localization_pending_confirmation",
    "localization_rejected",
    "initialpose_received_log",
    "map_correction_restore",
    "map_loaded_localization_mode",
    "database_readonly",
    "visual_registration_missing",
    LINK_ADDED_CANDIDATE_CATEGORY,
}


def strip_ansi(text: str) -> str:
    """去掉 rtabmap 核心日志里的 ANSI 颜色控制符。"""
    return ANSI_ESCAPE_RE.sub("", str(text))


def _finite_float(value: Any) -> tuple[float | None, bool]:
    """返回 (有限浮点或 None, 是否非有限/非法)。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None, True
    if not math.isfinite(number):
        return None, True
    return number, False


def sanitize_stats(
    keys: Any, values: Any
) -> tuple[dict[str, float | None], dict[str, Any]]:
    """把 /info 的 stats_keys/stats_values 安全转成 dict，非有限值置 None 并计数。"""
    key_list = [str(item) for item in (keys or [])]
    value_list = list(values or [])
    stats: dict[str, float | None] = {}
    nonfinite = 0
    invalid = 0
    for index, key in enumerate(key_list):
        if index >= len(value_list):
            stats[key] = None
            invalid += 1
            continue
        number, bad = _finite_float(value_list[index])
        stats[key] = number
        if bad:
            nonfinite += 1
    return stats, {
        "keys": len(key_list),
        "values": len(value_list),
        "length_mismatch": len(key_list) != len(value_list),
        "nonfinite_or_invalid_values": nonfinite,
        "missing_values": invalid,
    }


def parse_log_timestamp_ns(text: str) -> int | None:
    """解析 rtabmap 核心日志 "(2026-10-20 14:56:17.274)" 本地时间戳为 epoch ns。"""
    match = LOG_TIMESTAMP_RE.search(str(text))
    if not match:
        return None
    try:
        moment = datetime(
            int(match.group("year")),
            int(match.group("month")),
            int(match.group("day")),
            int(match.group("hour")),
            int(match.group("minute")),
            int(match.group("second")),
            int(match.group("millis")) * 1000,
        )
    except ValueError:
        return None
    try:
        return int(moment.timestamp() * 1e9)
    except (OverflowError, OSError, ValueError):
        return None


def iso_from_wall_ns(wall_ns: int | None) -> str | None:
    if wall_ns is None:
        return None
    try:
        return datetime.fromtimestamp(int(wall_ns) / 1e9).astimezone().isoformat(timespec="milliseconds")
    except (OverflowError, OSError, ValueError):
        return None


def translation_norm(translation: Mapping[str, Any] | None) -> float | None:
    if not isinstance(translation, Mapping):
        return None
    x, bad_x = _finite_float(translation.get("x"))
    y, bad_y = _finite_float(translation.get("y"))
    z, bad_z = _finite_float(translation.get("z"))
    if bad_x or bad_y or bad_z:
        return None
    return math.sqrt(x * x + y * y + z * z)


def translation_delta_m(
    first: Mapping[str, Any] | None, second: Mapping[str, Any] | None
) -> float | None:
    if not isinstance(first, Mapping) or not isinstance(second, Mapping):
        return None
    values: list[float] = []
    for axis in ("x", "y", "z"):
        a, bad_a = _finite_float(first.get(axis))
        b, bad_b = _finite_float(second.get(axis))
        if bad_a or bad_b:
            return None
        values.append(a - b)
    return math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)


def _normalized_quaternion(
    quaternion: Mapping[str, Any] | None,
) -> tuple[float, float, float, float] | None:
    if not isinstance(quaternion, Mapping):
        return None
    values: list[float] = []
    for key in ("x", "y", "z", "w"):
        number, bad = _finite_float(quaternion.get(key))
        if bad:
            return None
        values.append(number)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        return None
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def quaternion_angle_rad(
    first: Mapping[str, Any] | None, second: Mapping[str, Any] | None
) -> float | None:
    """两姿态的相对旋转角（rad）；q 与 -q 视为同一旋转。"""
    q1 = _normalized_quaternion(first)
    q2 = _normalized_quaternion(second)
    if q1 is None or q2 is None:
        return None
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _quat_canonical(quaternion: Mapping[str, Any] | None) -> tuple[float, float, float, float] | None:
    """规范四元数符号（w >= 0），避免 q 与 -q 被当成不同值。"""
    normalized = _normalized_quaternion(quaternion)
    if normalized is None:
        return None
    x, y, z, w = normalized
    if w < 0.0:
        return (-x, -y, -z, -w)
    return normalized


def _stat_lookup(stats: Mapping[str, float | None], key: str) -> float | None:
    """兼容带/不带尾部斜杠的 stats 键名（不同 rtabmap 构建键名有差异）。"""
    if key in stats:
        return stats[key]
    stripped = key.rstrip("/")
    if stripped in stats:
        return stats[stripped]
    with_slash = stripped + "/"
    if with_slash in stats:
        return stats[with_slash]
    return None


def _stat_value(stats: Mapping[str, float | None], name: str) -> float | None:
    return _stat_lookup(stats, STAT_KEYS[name])


def classify_rosout_message(name: Any, message: Any) -> dict[str, Any] | None:
    """分类 /rosout 或后端日志文本中的定位相关行；非定位相关返回 None。"""
    text = strip_ansi(str(message or "")).strip()
    if not text:
        return None
    node_name = str(name or "")
    category = None
    if PENDING_TEXT_MARKER in text and PENDING_WAITING_MARKER in text:
        category = "localization_pending_confirmation"
    elif any(marker in text for marker in REJECTED_TEXT_MARKERS):
        category = "localization_rejected"
    elif INITIALPOSE_LOG_MARKER in text:
        category = "initialpose_received_log"
    elif MAP_CORRECTION_MARKER in text:
        category = "map_correction_restore"
    elif MAP_LOADED_MARKER in text:
        category = "map_loaded_localization_mode"
    elif READONLY_MARKER in text:
        category = "database_readonly"
    elif VISUAL_MISSING_MARKER in text:
        category = "visual_registration_missing"
    elif any(marker in text for marker in CLOSURE_ADDED_MARKERS):
        # 仅是后端 link 加入的候选证据；之后仍可能被图优化拒绝或延迟确认，不能计 accepted。
        category = LINK_ADDED_CANDIDATE_CATEGORY
    if category is None:
        return None
    if node_name and "rtabmap" not in node_name.lower():
        # /rosout 上其它节点的同类字样不作为本任务证据
        return None
    nearest_match = NEAREST_ID_RE.search(text)
    return {
        "category": category,
        "localization_relevant": category in RONSOUT_RELEVANT_CATEGORIES,
        "node_name": node_name or None,
        "raw": text[:1000],
        "parsed_stamp_ns": parse_log_timestamp_ns(text),
        "nearest_id": int(nearest_match.group("nearest")) if nearest_match else None,
    }


def classify_backend_log_line(text: str) -> dict[str, Any] | None:
    """分类后端日志单行；返回 None 表示与 rtabmap 无关（不落盘）。"""
    cleaned = strip_ansi(text).strip().rstrip("\r")
    if not cleaned:
        return None
    classified = classify_rosout_message("rtabmap", cleaned)
    if classified is not None:
        return classified
    if not any(token in cleaned for token in ("Rtabmap.cpp", "Memory.cpp", "MapsManager.cpp", "rtabmap")):
        return None
    return {
        "category": "rtabmap_other",
        "localization_relevant": False,
        "node_name": "rtabmap",
        "raw": cleaned[:1000],
        "parsed_stamp_ns": parse_log_timestamp_ns(cleaned),
        "nearest_id": None,
    }


def _int_or_none(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _sequence(value: Any) -> list[Any]:
    try:
        return list(value or [])
    except TypeError:
        return []


def build_info_data(info: Mapping[str, Any]) -> dict[str, Any]:
    """把探针转换后的 /info 纯数据转为「原始 frame + 派生帧字段」事件数据。"""
    stats, stats_meta = sanitize_stats(info.get("stats_keys"), info.get("stats_values"))
    derived: dict[str, Any] = {}

    proximity_raw = _int_or_none(info.get("proximity_detection_id"))
    loop_closure_raw = _int_or_none(info.get("loop_closure_id"))
    landmark_raw = _int_or_none(info.get("landmark_id"))
    proximity_id = proximity_raw if proximity_raw is not None and proximity_raw > 0 else None
    loop_closure_id = loop_closure_raw if loop_closure_raw is not None and loop_closure_raw > 0 else None
    landmark_id = landmark_raw if landmark_raw is not None and landmark_raw > 0 else None
    accepted_ids = {
        "proximity_detection_id": proximity_id,
        "loop_closure_id": loop_closure_id,
        "landmark_id": landmark_id,
    }
    derived["accepted_ids"] = accepted_ids
    derived["accepted_any"] = any(value is not None for value in accepted_ids.values())
    derived["id_fields_unknown"] = [
        name for name, value in accepted_ids.items() if value is None
    ]
    derived["stats_accepted_hypothesis_id"] = _stat_value(stats, "accepted_hypothesis_id")
    derived["stats_loop_id"] = _stat_value(stats, "loop_id")
    derived["bayes_accepted_hypothesis_frame"] = bool(
        derived["stats_accepted_hypothesis_id"] not in (None, 0.0)
    )

    movement = {
        "small_movement": _stat_value(stats, "small_movement"),
        "fast_movement": _stat_value(stats, "fast_movement"),
    }
    derived["movement"] = movement
    derived["small_movement_frame"] = bool(movement["small_movement"] not in (None, 0.0))

    candidate_fields = {name: _stat_value(stats, name) for name in CANDIDATE_COUNT_KEYS}
    candidate_fields["highest_hypothesis_id"] = _stat_value(stats, "highest_hypothesis_id")
    candidate_fields["highest_hypothesis_value"] = _stat_value(stats, "highest_hypothesis_value")
    candidate_fields["hypothesis_ratio"] = _stat_value(stats, "hypothesis_ratio")
    candidate_fields["space_last_detection_id"] = _stat_value(stats, "space_last_detection_id")
    candidate_fields["closest_node_distance_m"] = _stat_value(stats, "closest_node_distance_m")
    candidate_fields["closest_node_angle_rad"] = _stat_value(stats, "closest_node_angle_rad")
    count_candidates = [
        value
        for name, value in candidate_fields.items()
        if name in CANDIDATE_COUNT_KEYS and value not in (None, 0.0)
    ]
    highest_id = candidate_fields.get("highest_hypothesis_id")
    derived["candidate"] = candidate_fields
    derived["candidate_frame"] = bool(count_candidates) or bool(highest_id not in (None, 0.0))
    derived["candidate_fields_known"] = {
        name: value is not None for name, value in candidate_fields.items()
    }

    rejected = {
        "rejected_hypothesis": _stat_value(stats, "rejected_hypothesis"),
        "suppressed_hypothesis_id": _stat_value(stats, "suppressed_hypothesis_id"),
    }
    derived["rejected"] = rejected
    derived["rejected_frame"] = bool(
        rejected["rejected_hypothesis"] not in (None, 0.0)
        or rejected["suppressed_hypothesis_id"] not in (None, 0.0)
    )

    icp = {
        "space_detections_added_icp_multi": _stat_value(stats, "space_detections_icp_multi"),
        "space_detections_added_icp_global": _stat_value(stats, "space_detections_icp_global"),
        "space_scan_paths_checked": _stat_value(stats, "space_scan_paths_checked"),
        "fitness": None,
        "note": "ICP fitness/inliers 不在 /info stats 中；unknown 不伪造",
    }
    derived["icp"] = icp
    derived["optimization"] = {
        "iterations": _stat_value(stats, "optimization_iterations"),
        "error": _stat_value(stats, "optimization_error"),
    }
    known_candidate_count = sum(1 for value in derived["candidate_fields_known"].values() if value)
    derived["unknown_fields"] = {
        "candidate_fields_missing": [
            name for name, known in derived["candidate_fields_known"].items() if not known
        ],
        "candidate_fields_known_count": known_candidate_count,
        "candidate_fields_total": len(derived["candidate_fields_known"]),
        "icp_fitness": "unknown_not_in_info_stats",
    }

    id_unknown = [
        name
        for name, value in (
            ("ref_id", info.get("ref_id")),
            ("loop_closure_id", info.get("loop_closure_id")),
            ("proximity_detection_id", info.get("proximity_detection_id")),
            ("landmark_id", info.get("landmark_id")),
        )
        if _int_or_none(value) is None
    ]
    raw = {
        "header_frame_id": str(info.get("header_frame_id") or ""),
        # 缺失/非法 ID 保持 null（unknown），不 coerce 成 0
        "ref_id": _int_or_none(info.get("ref_id")),
        "loop_closure_id": loop_closure_raw,
        "proximity_detection_id": proximity_raw,
        "landmark_id": landmark_raw,
        "loop_closure_transform": info.get("loop_closure_transform"),
        "wm_state_length": len(_sequence(info.get("wm_state"))),
        "stats": stats,
        "stats_meta": stats_meta,
        "unknown_fields": id_unknown,
    }
    if "header_stamp_ns" in info:
        raw["header_stamp_ns"] = info.get("header_stamp_ns")
    return {"raw": raw, "derived": derived}


def initialpose_event_data(payload: Mapping[str, Any]) -> dict[str, Any]:
    position = payload.get("position") if isinstance(payload.get("position"), Mapping) else {}
    orientation = payload.get("orientation") if isinstance(payload.get("orientation"), Mapping) else {}
    covariance = _sequence(payload.get("covariance"))
    position_values = {
        axis: _finite_float(position.get(axis))[0] for axis in ("x", "y", "z")
    }
    orientation_values = {
        axis: _finite_float(orientation.get(axis))[0] for axis in ("x", "y", "z", "w")
    }
    valid = (
        all(value is not None for value in position_values.values())
        and all(value is not None for value in orientation_values.values())
    )
    return {
        "frame_id": str(payload.get("frame_id") or ""),
        "position": position_values,
        "orientation": orientation_values,
        "covariance_diag": [
            _finite_float(covariance[index])[0] if index < len(covariance) else None
            for index in (0, 7, 14, 21, 28, 35)
        ],
        "valid": valid,
        "unknown_fields": ([] if valid else ["position_or_orientation_nonfinite_or_missing"]),
    }


def localization_pose_event_data(payload: Mapping[str, Any]) -> dict[str, Any]:
    data = initialpose_event_data(payload)
    data["kind_note"] = "pose topic output; not accepted evidence, not precision proof"
    return data


def tf_change_event_data(
    translation: Mapping[str, Any] | None,
    quaternion: Mapping[str, Any] | None,
    previous_translation: Mapping[str, Any] | None,
    previous_quaternion: Mapping[str, Any] | None,
    first_value: bool,
    stamp_ns: int | None,
) -> dict[str, Any]:
    """map->camera_init 的首值/变化事件数据；q 与 -q 等价，变化量含平移 m 与角度 rad。"""
    translation_norm_value = translation_norm(translation)
    rotation_from_identity = quaternion_angle_rad(
        {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}, quaternion
    )
    data: dict[str, Any] = {
        "first_value": bool(first_value),
        "stamp_ns": stamp_ns,
        "translation": {
            axis: _finite_float((translation or {}).get(axis))[0] for axis in ("x", "y", "z")
        },
        "quaternion": {
            axis: _finite_float((quaternion or {}).get(axis))[0] for axis in ("x", "y", "z", "w")
        },
        "translation_norm_m": translation_norm_value,
        "rotation_from_identity_rad": rotation_from_identity,
        "translation_delta_m": None if first_value else translation_delta_m(previous_translation, translation),
        "rotation_delta_rad": None if first_value else quaternion_angle_rad(previous_quaternion, quaternion),
        "quaternion_sign_note": "q and -q treated as same rotation",
        "acceptance_note": "TF change alone is not localization acceptance",
    }
    return data


class LocalizationObservationState:
    """有界内存的定位事件累计器：JSONL 全量由 writer 落盘，这里只保留累计与近期事件。"""

    def __init__(self, run_tag: str = "", rosout_subscribed: bool = True) -> None:
        self._run_tag = str(run_tag or "")
        self._rosout_subscribed = bool(rosout_subscribed)
        self._seq = 0
        self._started_wall_ns: int | None = None
        self._cumulative: dict[str, Any] = {
            "info_events": 0,
            "info_frames_with_accepted_ids": 0,
            "backend_accepted_events": 0,
            "backend_accepted_by_kind": {"proximity_detection_id": 0, "loop_closure_id": 0, "landmark_id": 0},
            "bayes_accepted_hypothesis_frames": 0,
            "small_movement_frames": 0,
            "small_movement_frames_without_candidate": 0,
            "candidate_frames": 0,
            "rejected_frames": 0,
            "optimization_frames": 0,
            "pending_confirmation_events": 0,
            "pending_by_source": {"rosout": 0, "backend_log": 0},
            "initialpose_events": 0,
            "initialpose_received_log_events": 0,
            "initialpose_received_log_by_source": {"rosout": 0, "backend_log": 0},
            "map_correction_restore_events": 0,
            "map_loaded_events": 0,
            "database_readonly_events": 0,
            "visual_registration_missing_events": 0,
            "candidate_link_added_events": 0,
            "localization_pose_events": 0,
            "tf_updates": 0,
            "tf_change_events": 0,
            "rosout_rtabmap_lines_seen": 0,
            "rosout_localization_events": 0,
            "backend_log_lines_read": 0,
            "backend_log_relevant_events": 0,
            "backend_log_other_events": 0,
            "backend_log_truncation_events": 0,
            "backend_log_rotation_events": 0,
            "backend_log_missing_events": 0,
            "backend_log_reappeared_events": 0,
            "backend_log_overflow_events": 0,
            "backend_log_error_events": 0,
        }
        self._recent: dict[str, deque] = {
            "accepted": deque(maxlen=MAX_RECENT_ACCEPTED),
            "pending": deque(maxlen=MAX_RECENT_PENDING),
            "initialpose": deque(maxlen=MAX_RECENT_INITIALPOSE),
            "tf_changes": deque(maxlen=MAX_RECENT_TF_CHANGES),
            "backend_relevant": deque(maxlen=MAX_RECENT_BACKEND_RELEVANT),
            "rosout_relevant": deque(maxlen=MAX_RECENT_ROSOUT_RELEVANT),
            "localization_pose": deque(maxlen=MAX_RECENT_LOCALIZATION_POSE),
            "candidate_link_added": deque(maxlen=MAX_RECENT_PENDING),
        }
        self._last: dict[str, Any] = {
            "accepted_event": None,
            "pending_event": None,
            "initialpose_event": None,
            "tf_change_event": None,
            "backend_log_event": None,
            "rosout_localization_event": None,
            "candidate_link_added_event": None,
            "localization_pose_event": None,
            "info_event": None,
        }
        self._latest_info: dict[str, Any] | None = None
        self._last_info_accepted = False
        self._initial_pose_events: list[dict[str, Any]] = []
        self._pending_since_last_accept = 0
        self._pending_after_initialpose = 0
        self._tf_previous: dict[str, Any] | None = None
        self._tf_updates = 0
        self._tf_changes = 0
        self._backend_log_state: dict[str, Any] = {
            "configured": False,
            "status": "not_configured",
            "note": "backend_log_path 未配置：rtabmap core UWARN（如 Localization was good waiting / "
                    "Update map correction）通常不发布到 /rosout，pending 日志可观测性不足",
        }
        self._writer_state: dict[str, Any] = {}
        self._info_seq: int | None = None
        self._pending_seq: int | None = None
        self._last_accept_seq: int | None = None
        self._events_since_accept = 0
        self._initialpose_at_last_accept: dict[str, Any] | None = None
        self._event_errors: int = 0

    # ---- helpers ---------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _start_if_needed(self, recv_wall_ns: int) -> None:
        if self._started_wall_ns is None:
            self._started_wall_ns = int(recv_wall_ns)

    def _event(
        self,
        event: str,
        source: str,
        data: Mapping[str, Any],
        recv_wall_ns: int,
        recv_mono_ns: int,
        source_stamp_ns: int | None,
    ) -> dict[str, Any]:
        self._start_if_needed(recv_wall_ns)
        seq = self._next_seq()
        return {
            "schema": SCHEMA_VERSION,
            "seq": seq,
            "event": event,
            "source": source,
            "run_tag": self._run_tag or None,
            "source_stamp_ns": source_stamp_ns,
            "source_stamp_unknown": source_stamp_ns is None,
            "recv_wall_ns": int(recv_wall_ns),
            "recv_mono_ns": int(recv_mono_ns),
            "recv_wall_iso": iso_from_wall_ns(recv_wall_ns),
            "data": dict(data),
        }

    def _initialpose_correlation(self, recv_wall_ns: int) -> dict[str, Any] | None:
        if not self._initial_pose_events:
            return None
        latest = self._initial_pose_events[-1]
        delta_ns = int(recv_wall_ns) - int(latest["recv_wall_ns"])
        return {
            "initialpose_event_seq": latest["seq"],
            "initialpose_recv_wall_ns": latest["recv_wall_ns"],
            "initialpose_source_stamp_ns": latest["source_stamp_ns"],
            "delta_s": round(delta_ns / 1e9, 9),
            "association": "time_only_not_causal",
            "note": "仅按接收时间关联到最近一次 /initialpose，不构成因果证明",
        }

    def _record_accepted_event(self, event: dict[str, Any], kind: str, recv_wall_ns: int) -> None:
        accepted = dict(event["data"].get("accepted_ids") or {})
        record = {
            "seq": event["seq"],
            "kind": kind,
            "ref_id": event["data"].get("derived", {}).get("ref_id"),
            "accepted_ids": accepted,
            "recv_wall_ns": event["recv_wall_ns"],
            "source_stamp_ns": event["source_stamp_ns"],
            "correlated_initial_pose": self._initialpose_correlation(recv_wall_ns),
        }
        self._recent["accepted"].append(record)
        self._last["accepted_event"] = record
        self._last_accept_seq = event["seq"]
        self._events_since_accept = 0
        self._pending_since_last_accept = 0
        self._initialpose_at_last_accept = record["correlated_initial_pose"]

    def _record_pending_event(
        self, event: dict[str, Any], category: str, source: str
    ) -> None:
        recv_wall_ns = int(event["recv_wall_ns"])
        record = {
            "seq": event["seq"],
            "category": category,
            "source": source,
            "raw": event["data"].get("raw"),
            "parsed_stamp_ns": event["data"].get("parsed_stamp_ns"),
            "nearest_id": event["data"].get("nearest_id"),
            "recv_wall_ns": recv_wall_ns,
            "correlated_initial_pose": self._initialpose_correlation(recv_wall_ns),
            "accepted": False,
            "note": "pending candidate; not accepted evidence",
        }
        self._recent["pending"].append(record)
        self._last["pending_event"] = record
        self._pending_seq = event["seq"]
        self._pending_since_last_accept += 1
        if self._initial_pose_events:
            self._pending_after_initialpose += 1
        self._events_since_accept += 1

    # ---- event handlers --------------------------------------------------
    def on_info(
        self, info: Mapping[str, Any], recv_wall_ns: int, recv_mono_ns: int
    ) -> dict[str, Any]:
        data = build_info_data(info)
        stamp_ns = info.get("header_stamp_ns")
        try:
            stamp_ns = int(stamp_ns) if stamp_ns is not None else None
        except (TypeError, ValueError):
            stamp_ns = None
        data["derived"]["ref_id"] = data["raw"]["ref_id"]
        data["derived"]["event_notes"] = {
            "accepted_ids_are_backend_event_evidence_only": True,
            "bayes_accepted_hypothesis_is_not_geometry_acceptance": True,
        }
        event = self._event(EVENT_INFO, SOURCE_INFO, data, recv_wall_ns, recv_mono_ns, stamp_ns)
        self._cumulative["info_events"] += 1
        derived = data["derived"]
        if derived["accepted_any"]:
            self._cumulative["info_frames_with_accepted_ids"] += 1
            self._cumulative["backend_accepted_events"] += 1
            for key in ("proximity_detection_id", "loop_closure_id", "landmark_id"):
                if derived["accepted_ids"].get(key) is not None:
                    self._cumulative["backend_accepted_by_kind"][key] += 1
            self._record_accepted_event(event, "info_accepted_ids", recv_wall_ns)
        if derived["bayes_accepted_hypothesis_frame"]:
            self._cumulative["bayes_accepted_hypothesis_frames"] += 1
        if derived["small_movement_frame"]:
            self._cumulative["small_movement_frames"] += 1
            if not derived["candidate_frame"]:
                self._cumulative["small_movement_frames_without_candidate"] += 1
        if derived["candidate_frame"]:
            self._cumulative["candidate_frames"] += 1
        if derived["rejected_frame"]:
            self._cumulative["rejected_frames"] += 1
        iterations = derived["optimization"]["iterations"]
        if iterations is not None and iterations > 0:
            self._cumulative["optimization_frames"] += 1
        self._latest_info = {
            "seq": event["seq"],
            "recv_wall_ns": recv_wall_ns,
            "source_stamp_ns": stamp_ns,
            "ref_id": data["raw"]["ref_id"],
            "accepted_ids": derived["accepted_ids"],
            "accepted_any": derived["accepted_any"],
            "bayes_accepted_hypothesis_id": derived["stats_accepted_hypothesis_id"],
            "bayes_accepted_hypothesis_frame": derived["bayes_accepted_hypothesis_frame"],
            "small_movement_frame": derived["small_movement_frame"],
            "candidate_frame": derived["candidate_frame"],
            "rejected_frame": derived["rejected_frame"],
            "optimization_iterations": iterations,
            "unknown_fields": derived["unknown_fields"],
        }
        self._last["info_event"] = self._latest_info
        self._last_info_accepted = derived["accepted_any"]
        self._info_seq = event["seq"]
        self._events_since_accept += 0 if derived["accepted_any"] else 1
        return event

    def on_initialpose(
        self, payload: Mapping[str, Any], recv_wall_ns: int, recv_mono_ns: int
    ) -> dict[str, Any]:
        data = initialpose_event_data(payload)
        stamp_ns = payload.get("header_stamp_ns")
        try:
            stamp_ns = int(stamp_ns) if stamp_ns is not None else None
        except (TypeError, ValueError):
            stamp_ns = None
        event = self._event(
            EVENT_INITIALPOSE, SOURCE_INITIALPOSE, data, recv_wall_ns, recv_mono_ns, stamp_ns
        )
        self._cumulative["initialpose_events"] += 1
        record = {
            "seq": event["seq"],
            "recv_wall_ns": int(recv_wall_ns),
            "source_stamp_ns": stamp_ns,
            "frame_id": data["frame_id"],
            "position": data["position"],
            "orientation": data["orientation"],
            "valid": data["valid"],
        }
        self._initial_pose_events.append(record)
        if len(self._initial_pose_events) > MAX_RECENT_INITIALPOSE:
            self._initial_pose_events.pop(0)
        self._recent["initialpose"].append(record)
        self._last["initialpose_event"] = record
        self._pending_after_initialpose = 0
        return event

    def on_rosout(
        self,
        name: Any,
        message: Any,
        level: Any,
        recv_wall_ns: int,
        recv_mono_ns: int,
        source_stamp_ns: int | None = None,
    ) -> dict[str, Any] | None:
        self._cumulative["rosout_rtabmap_lines_seen"] += 1
        classified = classify_rosout_message(name, message)
        if classified is None:
            return None
        self._cumulative["rosout_localization_events"] += 1
        data = {
            "category": classified["category"],
            "localization_relevant": classified["localization_relevant"],
            "node_name": classified["node_name"],
            "level": _int_or_none(level),
            "raw": classified["raw"],
            "parsed_stamp_ns": classified["parsed_stamp_ns"],
            "nearest_id": classified["nearest_id"],
            "source_note": "/rosout 仅覆盖通过 ROS 日志发布的 rtabmap 行；core UWARN 可能不在其中",
        }
        event = self._event(
            EVENT_ROSOUT, SOURCE_ROSOUT, data, recv_wall_ns, recv_mono_ns, source_stamp_ns
        )
        self._apply_rosout_category(event, classified, recv_wall_ns)
        return event

    def _apply_rosout_category(
        self, event: dict[str, Any], classified: Mapping[str, Any], recv_wall_ns: int
    ) -> None:
        category = str(classified["category"])
        self._last["rosout_localization_event"] = event
        if category == "localization_pending_confirmation":
            self._cumulative["pending_confirmation_events"] += 1
            self._cumulative["pending_by_source"]["rosout"] += 1
            self._recent["rosout_relevant"].append(
                {"seq": event["seq"], "category": category, "source": "rosout", "raw": classified["raw"]}
            )
            self._record_pending_event(event, category, "rosout")
        elif category == "initialpose_received_log":
            self._cumulative["initialpose_received_log_events"] += 1
            self._cumulative["initialpose_received_log_by_source"]["rosout"] += 1
        elif category == "map_correction_restore":
            self._cumulative["map_correction_restore_events"] += 1
        elif category == "map_loaded_localization_mode":
            self._cumulative["map_loaded_events"] += 1
        elif category == "database_readonly":
            self._cumulative["database_readonly_events"] += 1
        elif category == "visual_registration_missing":
            self._cumulative["visual_registration_missing_events"] += 1
        elif category == LINK_ADDED_CANDIDATE_CATEGORY:
            self._record_candidate_link_added(event, classified, recv_wall_ns)

    def _record_candidate_link_added(
        self, event: dict[str, Any], classified: Mapping[str, Any], recv_wall_ns: int
    ) -> None:
        """后端日志 link 加入行：候选证据，不改 last accepted、不清 pending、不计最终接受。"""
        self._cumulative["candidate_link_added_events"] += 1
        record = {
            "seq": event["seq"],
            "kind": "candidate_link_added",
            "raw": classified["raw"],
            "recv_wall_ns": int(recv_wall_ns),
            "source_stamp_ns": event["source_stamp_ns"],
            "accepted": False,
            "correlated_initial_pose": self._initialpose_correlation(recv_wall_ns),
            "note": "后端 link 加入候选；后续仍可能被图优化拒绝或延迟确认，不是 accepted/精度证据",
        }
        self._recent["candidate_link_added"].append(record)
        self._last["candidate_link_added_event"] = record

    def on_backend_log_line(
        self,
        line: str,
        recv_wall_ns: int,
        recv_mono_ns: int,
        parsed_stamp_ns: int | None = None,
    ) -> dict[str, Any] | None:
        self._cumulative["backend_log_lines_read"] += 1
        classified = classify_backend_log_line(line)
        if classified is None:
            return None
        data = {
            "category": classified["category"],
            "localization_relevant": classified["localization_relevant"],
            "raw": classified["raw"],
            "parsed_stamp_ns": classified["parsed_stamp_ns"],
            "nearest_id": classified["nearest_id"],
            "source_note": "backend_log_path 只读增量 tail；该日志非 ROS topic 证据",
        }
        source_stamp = classified["parsed_stamp_ns"]
        if source_stamp is None:
            source_stamp = parsed_stamp_ns
        event = self._event(
            EVENT_BACKEND_LOG,
            SOURCE_BACKEND_LOG,
            data,
            recv_wall_ns,
            recv_mono_ns,
            source_stamp,
        )
        self._last["backend_log_event"] = event
        if classified["localization_relevant"]:
            self._cumulative["backend_log_relevant_events"] += 1
            self._recent["backend_relevant"].append(
                {
                    "seq": event["seq"],
                    "category": classified["category"],
                    "raw": classified["raw"],
                    "parsed_stamp_ns": classified["parsed_stamp_ns"],
                }
            )
            category = str(classified["category"])
            if category == "localization_pending_confirmation":
                self._cumulative["pending_confirmation_events"] += 1
                self._cumulative["pending_by_source"]["backend_log"] += 1
                self._record_pending_event(event, category, "backend_log")
            elif category == "initialpose_received_log":
                self._cumulative["initialpose_received_log_events"] += 1
                self._cumulative["initialpose_received_log_by_source"]["backend_log"] += 1
            elif category == "map_correction_restore":
                self._cumulative["map_correction_restore_events"] += 1
            elif category == "map_loaded_localization_mode":
                self._cumulative["map_loaded_events"] += 1
            elif category == "database_readonly":
                self._cumulative["database_readonly_events"] += 1
            elif category == "visual_registration_missing":
                self._cumulative["visual_registration_missing_events"] += 1
            elif category == LINK_ADDED_CANDIDATE_CATEGORY:
                self._record_candidate_link_added(event, classified, recv_wall_ns)
        else:
            self._cumulative["backend_log_other_events"] += 1
        return event

    def on_backend_log_meta(
        self,
        meta_kind: str,
        detail: Mapping[str, Any] | None,
        recv_wall_ns: int,
        recv_mono_ns: int,
    ) -> dict[str, Any]:
        mapping = {
            "truncation": "backend_log_truncation_events",
            "rotation": "backend_log_rotation_events",
            "missing": "backend_log_missing_events",
            "reappeared": "backend_log_reappeared_events",
            "line_overflow": "backend_log_overflow_events",
            "open_error": "backend_log_error_events",
        }
        counter_key = mapping.get(meta_kind)
        if counter_key:
            self._cumulative[counter_key] += 1
        data = {
            "category": f"backend_log_{meta_kind}",
            "meta_event": True,
            "detail": dict(detail or {}),
            "source_note": "backend_log tail 结构性事件（截断/轮转/缺失/超长行），只读观察",
        }
        event = self._event(EVENT_BACKEND_LOG, SOURCE_BACKEND_LOG, data, recv_wall_ns, recv_mono_ns, None)
        self._last["backend_log_event"] = event
        return event

    def on_tf_map_to_camera_init(
        self,
        translation: Mapping[str, Any] | None,
        quaternion: Mapping[str, Any] | None,
        recv_wall_ns: int,
        recv_mono_ns: int,
        source_stamp_ns: int | None = None,
        translation_epsilon_m: float = 1e-4,
        rotation_epsilon_rad: float = 1e-4,
    ) -> dict[str, Any] | None:
        """记录 map->camera_init 首值/变化；返回 None 表示与上次相同（不重复落盘）。"""
        self._tf_updates += 1
        self._cumulative["tf_updates"] = self._tf_updates
        canonical = _quat_canonical(quaternion)
        current = {
            "translation": {axis: _finite_float((translation or {}).get(axis))[0] for axis in ("x", "y", "z")},
            "quaternion": {axis: _finite_float((quaternion or {}).get(axis))[0] for axis in ("x", "y", "z", "w")},
        }
        first = self._tf_previous is None
        changed = first
        if not first:
            delta = translation_delta_m(self._tf_previous["translation"], current["translation"])
            angle = quaternion_angle_rad(self._tf_previous["quaternion"], current["quaternion"])
            if delta is None or angle is None:
                changed = True
            else:
                changed = delta > translation_epsilon_m or angle > rotation_epsilon_rad
        if not changed:
            return None
        previous = self._tf_previous or {}
        data = tf_change_event_data(
            current["translation"],
            current["quaternion"],
            previous.get("translation"),
            previous.get("quaternion"),
            first,
            source_stamp_ns,
        )
        data["canonical_quaternion"] = canonical
        event = self._event(EVENT_TF, SOURCE_TF, data, recv_wall_ns, recv_mono_ns, source_stamp_ns)
        self._tf_previous = current
        self._tf_changes += 1
        self._cumulative["tf_change_events"] = self._tf_changes
        record = {
            "seq": event["seq"],
            "first_value": first,
            "stamp_ns": source_stamp_ns,
            "translation": current["translation"],
            "quaternion": current["quaternion"],
            "translation_norm_m": data["translation_norm_m"],
            "translation_delta_m": data["translation_delta_m"],
            "rotation_delta_rad": data["rotation_delta_rad"],
            "recv_wall_ns": int(recv_wall_ns),
            "acceptance_note": "TF change alone is not localization acceptance",
        }
        self._recent["tf_changes"].append(record)
        self._last["tf_change_event"] = record
        return event

    def on_localization_pose(
        self, payload: Mapping[str, Any], recv_wall_ns: int, recv_mono_ns: int
    ) -> dict[str, Any]:
        data = localization_pose_event_data(payload)
        stamp_ns = payload.get("header_stamp_ns")
        try:
            stamp_ns = int(stamp_ns) if stamp_ns is not None else None
        except (TypeError, ValueError):
            stamp_ns = None
        event = self._event(
            EVENT_LOCALIZATION_POSE,
            SOURCE_LOCALIZATION_POSE,
            data,
            recv_wall_ns,
            recv_mono_ns,
            stamp_ns,
        )
        self._cumulative["localization_pose_events"] += 1
        record = {
            "seq": event["seq"],
            "recv_wall_ns": int(recv_wall_ns),
            "source_stamp_ns": stamp_ns,
            "frame_id": data["frame_id"],
            "position": data["position"],
            "orientation": data["orientation"],
        }
        self._recent["localization_pose"].append(record)
        self._last["localization_pose_event"] = record
        return event

    # ---- summary ---------------------------------------------------------
    def set_backend_log_state(self, state: Mapping[str, Any]) -> None:
        self._backend_log_state = dict(state)

    def set_writer_state(self, state: Mapping[str, Any]) -> None:
        self._writer_state = dict(state)

    def _derived_state(self) -> dict[str, Any]:
        # 只有 /info 发布的非零 proximity/loop id 计 accepted（CoreWrapper.cpp 2429 使用这些 ID）；
        # 后端日志 Add local loop closure 行只是候选 link 证据。
        accepted_total = int(self._cumulative["backend_accepted_events"])
        latest_initialpose_seq = self._initial_pose_events[-1]["seq"] if self._initial_pose_events else None
        accepted_since_latest_initialpose = None
        if latest_initialpose_seq is not None:
            accepted_since_latest_initialpose = bool(
                self._last_accept_seq is not None and self._last_accept_seq > latest_initialpose_seq
            )
        if self._latest_info is not None and self._last_info_accepted:
            state_hint = "accepted_ids_in_current_frame"
        elif accepted_total > 0:
            state_hint = "historical_accepted_evidence_no_current_frame_event"
        elif self._cumulative["pending_confirmation_events"] > 0:
            state_hint = "pending_confirmation_only"
        elif self._latest_info is not None:
            state_hint = "no_accept_evidence_in_current_frames"
        elif self._cumulative["tf_change_events"] > 0:
            state_hint = "map_tf_change_only_no_accept_evidence"
        elif self._cumulative["initialpose_events"] > 0:
            state_hint = "initial_pose_seen_no_accept_evidence"
        else:
            state_hint = "no_data"
        no_current = {
            "info_frame_present": self._latest_info is not None,
            "accepted_in_current_frame": bool(self._last_info_accepted),
            "note": "逐帧 accepted id 为 0 是正常现象，不清空历史累计；零帧不等于失败",
        }
        return {
            "state_hint": state_hint,
            "historical_backend_accepted_total": accepted_total,
            "historical_accepted_persists_after_zero_frame": bool(
                accepted_total > 0 and not self._last_info_accepted
            ),
            "no_current_event": no_current,
            "initialpose_seen": bool(self._cumulative["initialpose_events"]),
            "latest_initialpose_seq": latest_initialpose_seq,
            "last_accepted_seq": self._last_accept_seq,
            "accepted_since_latest_initialpose": accepted_since_latest_initialpose,
            "pending_current_initialpose_attempt": (
                None if latest_initialpose_seq is None else (not accepted_since_latest_initialpose)
            ),
            "initialpose_acceptance_note": (
                "新 /initialpose 后历史累计 accepted 仍保留；"
                "accepted_since_latest_initialpose=False 表示这次初值之后尚无接受事件"
            ),
            "last_accept_correlated_initialpose": self._initialpose_at_last_accept,
            "candidate_link_added_total": int(self._cumulative["candidate_link_added_events"]),
            "pending_since_last_accept": int(self._pending_since_last_accept),
            "events_since_last_accept": int(self._events_since_accept),
            "tf_change_is_not_acceptance": True,
            "small_movement_without_candidate_is_not_failure": True,
            "pending_log_observability": self._pending_observability(),
            "accuracy_note": "accepted 是后端事件证据；不是地图/定位精度验收，不能替代现场测量",
            "accepted_semantics": {
                "info_ids": "msg 级 proximity_detection_id/loop_closure_id 为后端接受 id 事件证据",
                "bayes_hypothesis": "Loop/Accepted_hypothesis_id 是 Bayes 接受假设，不等于最终几何接受",
                "candidate_link_added_log": "后端日志 Add local loop closure 行是 link 加入候选证据，"
                "仍可能被图优化拒绝/延迟确认；不计 accepted，不是精度通过",
                "tf_or_pose": "TF 变化与 /localization_pose 发布都不代表 accepted",
            },
        }

    def _pending_observability(self) -> dict[str, Any]:
        backend = dict(self._backend_log_state)
        configured = bool(backend.get("configured"))
        if configured and backend.get("status") == "active":
            coverage = "rosout_plus_backend_log_tail"
            complete = False
            note = (
                "已配置 backend_log_path 只读 tail；rtabmap core UWARN 可由此补齐，"
                "但日志来源仍非 ROS topic 证据，且不保证所有内部 WARN 都写入该文件"
            )
        elif configured:
            coverage = "backend_log_configured_not_active"
            complete = False
            note = "backend_log_path 已配置但当前 tail 非 active；pending 观测不完整"
        else:
            coverage = "rosout_only_partial"
            complete = False
            note = (
                "未配置 backend_log_path：rtabmap core UWARN（Localization was good waiting / "
                "Update map correction 等）通常直接写 stderr、不发布到 /rosout；"
                "pending 日志可观测性不足，相关计数可能偏低"
            )
        return {
            "rosout_subscribed": bool(self._rosout_subscribed),
            "backend_log": backend,
            "pending_coverage": coverage,
            "complete": complete,
            "note": note,
        }

    def summary(self, final: bool = False, generated_wall_ns: int | None = None) -> dict[str, Any]:
        self._writer_state = dict(self._writer_state)
        recent = {name: list(values) for name, values in self._recent.items()}
        return {
            "schema": SUMMARY_SCHEMA_VERSION,
            "run_tag": self._run_tag or None,
            "final": bool(final),
            "generated_wall_ns": int(generated_wall_ns) if generated_wall_ns is not None else None,
            "generated_wall_iso": iso_from_wall_ns(generated_wall_ns),
            "started_wall_ns": self._started_wall_ns,
            "event_sequence_last": self._seq,
            "writer": dict(self._writer_state),
            "pending_observability": self._pending_observability(),
            "cumulative": dict(self._cumulative),
            "derived": self._derived_state(),
            "current": {
                "latest_info": self._latest_info,
                "latest_tf_change": self._last["tf_change_event"],
                "latest_pending": self._last["pending_event"],
            },
            "last": dict(self._last),
            "recent": recent,
        }


class EventTimelineWriter:
    """独占唯一 JSONL 时间线 + summary 文件；不覆盖既有文件，逐条 flush。"""

    def __init__(self, report_path: str | os.PathLike[str], run_tag: str = "", max_suffixes: int = 100) -> None:
        self._report_path = Path(report_path).expanduser()
        self._run_tag = str(run_tag or "")
        self._handle = None
        self._events_written = 0
        self._write_errors = 0
        self._last_error: str | None = None
        self._closed = False
        self._jsonl_path: Path | None = None
        self._summary_path: Path | None = None
        self._report_path.parent.mkdir(parents=True, exist_ok=True)
        base_stem = self._report_path.stem
        for index in range(max_suffixes):
            suffix = "" if index == 0 else f"__{index + 1:02d}"
            jsonl_path = self._report_path.parent / f"{base_stem}.localization_events{suffix}.jsonl"
            summary_path = self._report_path.parent / f"{base_stem}.localization_summary{suffix}.json"
            if jsonl_path.exists() or summary_path.exists():
                continue
            try:
                descriptor = os.open(str(jsonl_path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
            except FileExistsError:
                continue
            try:
                summary_descriptor = os.open(
                    str(summary_path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644
                )
            except FileExistsError:
                os.close(descriptor)
                try:
                    os.unlink(str(jsonl_path))
                except OSError:
                    pass
                continue
            os.close(summary_descriptor)
            self._jsonl_path = jsonl_path
            self._summary_path = summary_path
            self._handle = os.fdopen(descriptor, "w", encoding="utf-8")
            return
        raise RuntimeError(
            f"无法创建独占 JSONL/summary 文件对（{max_suffixes} 个候选均已存在）：{self._report_path}"
        )

    @property
    def jsonl_path(self) -> Path | None:
        return self._jsonl_path

    @property
    def summary_path(self) -> Path | None:
        return self._summary_path

    @property
    def state(self) -> dict[str, Any]:
        return {
            "jsonl_path": str(self._jsonl_path) if self._jsonl_path else None,
            "summary_path": str(self._summary_path) if self._summary_path else None,
            "events_written": self._events_written,
            "write_errors": self._write_errors,
            "last_error": self._last_error,
            "closed": self._closed,
            "summary_reserved": self._summary_path is not None,
            "run_tag": self._run_tag or None,
        }

    def write(self, event: Mapping[str, Any]) -> bool:
        if self._handle is None or self._closed:
            self._write_errors += 1
            self._last_error = "writer_closed"
            return False
        try:
            line = json.dumps(dict(event), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            self._handle.write(line + "\n")
            self._handle.flush()
            self._events_written += 1
            return True
        except (OSError, TypeError, ValueError) as exc:
            self._write_errors += 1
            self._last_error = f"{type(exc).__name__}: {exc}"
            return False

    def write_summary(self, summary: Mapping[str, Any]) -> bool:
        if self._summary_path is None:
            return False
        try:
            payload = json.dumps(dict(summary), ensure_ascii=False, allow_nan=False, indent=1)
            temporary = self._summary_path.with_suffix(self._summary_path.suffix + ".tmp")
            temporary.write_text(payload + "\n", encoding="utf-8")
            os.replace(temporary, self._summary_path)
            return True
        except (OSError, TypeError, ValueError) as exc:
            self._write_errors += 1
            self._last_error = f"summary {type(exc).__name__}: {exc}"
            return False

    def close(self) -> dict[str, Any]:
        if self._handle is not None and not self._closed:
            try:
                self._handle.flush()
                os.fsync(self._handle.fileno())
            except OSError as exc:
                self._write_errors += 1
                self._last_error = f"flush {type(exc).__name__}: {exc}"
            finally:
                try:
                    self._handle.close()
                except OSError:
                    pass
        self._closed = True
        return self.state


class BackendLogTailer:
    """只读增量 tail 后端日志：从启动时 EOF 开始；显式报告截断/轮转/缺失/超长行。"""

    def __init__(
        self,
        path: str | os.PathLike[str],
        max_bytes_per_poll: int = 262144,
        max_line_bytes: int = 8192,
    ) -> None:
        self._path = Path(path).expanduser()
        self._max_bytes_per_poll = max(1024, int(max_bytes_per_poll))
        self._max_line_bytes = max(256, int(max_line_bytes))
        self._offset = 0
        self._inode: int | None = None
        self._buffer = b""
        self._status = "pending_open"
        self._lines_read = 0
        self._bytes_read = 0
        self._truncations = 0
        self._rotations = 0
        self._missing = 0
        self._reappeared = 0
        self._overflows = 0
        self._open_errors = 0

    @property
    def path(self) -> Path:
        return self._path

    @property
    def state(self) -> dict[str, Any]:
        return {
            "configured": True,
            "path": str(self._path),
            "status": self._status,
            "lines_read": self._lines_read,
            "bytes_read": self._bytes_read,
            "truncations": self._truncations,
            "rotations": self._rotations,
            "missing_events": self._missing,
            "reappeared_events": self._reappeared,
            "line_overflow_events": self._overflows,
            "open_errors": self._open_errors,
            "note": "只读增量 tail；从启动时 EOF 开始，不修改源日志",
        }

    def start(self) -> list[dict[str, Any]]:
        meta: list[dict[str, Any]] = []
        try:
            stat = self._path.stat()
        except FileNotFoundError:
            self._status = "missing"
            self._missing += 1
            meta.append({"kind": "missing", "detail": {"reason": "file_not_found_at_start"}})
            return meta
        except OSError as exc:
            self._status = "error"
            self._open_errors += 1
            meta.append({"kind": "open_error", "detail": {"error": str(exc)}})
            return meta
        self._status = "active"
        self._inode = int(stat.st_ino)
        self._offset = int(stat.st_size)
        return meta

    def poll(self) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        try:
            stat = self._path.stat()
        except FileNotFoundError:
            if self._status != "missing":
                self._status = "missing"
                self._missing += 1
                items.append({"kind": "missing", "detail": {"reason": "file_disappeared"}})
            return items
        except OSError as exc:
            self._status = "error"
            self._open_errors += 1
            items.append({"kind": "open_error", "detail": {"error": str(exc)}})
            return items
        if self._status == "missing":
            self._reappeared += 1
            items.append({"kind": "reappeared", "detail": {"inode": int(stat.st_ino)}})
            self._inode = int(stat.st_ino)
            self._offset = 0
            self._buffer = b""
        elif self._status in ("pending_open", "error"):
            self._inode = int(stat.st_ino)
            self._offset = int(stat.st_size)
        if self._inode is not None and int(stat.st_ino) != self._inode:
            self._rotations += 1
            items.append({"kind": "rotation", "detail": {"old_inode": self._inode, "new_inode": int(stat.st_ino)}})
            self._inode = int(stat.st_ino)
            self._offset = 0
            self._buffer = b""
        if int(stat.st_size) < self._offset:
            self._truncations += 1
            items.append(
                {"kind": "truncation", "detail": {"previous_offset": self._offset, "current_size": int(stat.st_size)}}
            )
            self._offset = 0
            self._buffer = b""
        if int(stat.st_size) == self._offset:
            self._status = "active"
            return items
        try:
            with open(self._path, "rb") as handle:
                handle.seek(self._offset)
                chunk = handle.read(self._max_bytes_per_poll)
                self._offset = handle.tell()
        except OSError as exc:
            self._open_errors += 1
            self._status = "error"
            items.append({"kind": "open_error", "detail": {"error": str(exc)}})
            return items
        self._status = "active"
        self._bytes_read += len(chunk)
        data = self._buffer + chunk
        *lines, self._buffer = data.split(b"\n")
        for raw_line in lines:
            text = raw_line.decode("utf-8", errors="replace").rstrip("\r")
            if len(raw_line) > self._max_line_bytes:
                self._overflows += 1
                items.append({"kind": "line_overflow", "detail": {"line_bytes": len(raw_line), "max_line_bytes": self._max_line_bytes}})
                continue
            self._lines_read += 1
            items.append({"kind": "line", "line": text})
        if len(self._buffer) > self._max_line_bytes:
            self._overflows += 1
            items.append({"kind": "line_overflow", "detail": {"line_bytes": len(self._buffer), "max_line_bytes": self._max_line_bytes}})
            self._buffer = b""
        return items


def now_times_ns() -> tuple[int, int]:
    return time.time_ns(), time.monotonic_ns()
