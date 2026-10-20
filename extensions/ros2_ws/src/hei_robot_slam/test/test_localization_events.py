"""localization_events 纯逻辑回归（无 ROS）。

覆盖任务 HEI-SLAM-OBS-20261020 验收点：
* accepted 一帧后连续零帧累计不清零；
* 小运动无候选不判失败；
* 延迟确认（Localization was good waiting）不是 accepted；
* initialpose 仅时间关联；
* TF 四元数 q/-q 等价、首值/变化、重复不落盘；
* 缺字段/NaN 安全（不伪造、不产生非法 JSON）；
* JSONL 独占不覆盖、summary 不被孤立文件覆盖、关停 flush/close；
* 后端日志只读 tail 的 EOF/截断/轮转/缺失/超长行行为；
* 后端 link 加入日志只算候选，不计 accepted。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import pytest

from hei_robot_slam import localization_events as LE

SMALL_KEY = "Memory/Small_movement/"
FAST_KEY = "Memory/Fast_movement/"
REJECTED_KEY = "Loop/RejectedHypothesis/"
ICP_MULTI_KEY = "Proximity/Space_detections_added_icp_multi/"
ACCEPTED_HYP_KEY = "Loop/Accepted_hypothesis_id/"
OPT_ITER_KEY = "Loop/Optimization_iterations/"

DEFAULT_STATS_KEYS = [SMALL_KEY, FAST_KEY, REJECTED_KEY, ICP_MULTI_KEY, ACCEPTED_HYP_KEY, OPT_ITER_KEY]
DEFAULT_STATS_VALUES = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def make_info(**overrides):
    base = {
        "header_stamp_ns": 1_000_000_000,
        "header_frame_id": "map",
        "ref_id": 7,
        "loop_closure_id": 0,
        "proximity_detection_id": 0,
        "landmark_id": 0,
        "loop_closure_transform": {},
        "wm_state": [1, 2, 3],
        "stats_keys": list(DEFAULT_STATS_KEYS),
        "stats_values": list(DEFAULT_STATS_VALUES),
    }
    base.update(overrides)
    return base


def pending_text():
    return (
        "\x1b[33m[ WARN] (2026-10-20 14:56:17.274) Rtabmap.cpp:3911::process() "
        "Localization was good, but waiting for another one to be more accurate "
        "(RGBD/MaxOdomCacheSize>0)\x1b[0m\r"
    )


def accepted_proximity_info(**overrides):
    return make_info(proximity_detection_id=29, **overrides)


def initialpose_payload(wall_ns: int = 0, **overrides):
    payload = {
        "header_stamp_ns": 500,
        "frame_id": "map",
        "position": {"x": 0.1, "y": 0.2, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "covariance": [0.0] * 36,
    }
    payload.update(overrides)
    return payload


# ---------------------------------------------------------------------------
# 事件信封 / accepted 累计
# ---------------------------------------------------------------------------
def test_event_envelope_contains_source_stamp_and_receive_times():
    state = LE.LocalizationObservationState(run_tag="t")
    event = state.on_info(make_info(), recv_wall_ns=111, recv_mono_ns=222)
    assert event["schema"] == LE.SCHEMA_VERSION
    assert event["seq"] == 1
    assert event["event"] == LE.EVENT_INFO
    assert event["source"] == LE.SOURCE_INFO
    assert event["source_stamp_ns"] == 1_000_000_000
    assert event["source_stamp_unknown"] is False
    assert event["recv_wall_ns"] == 111
    assert event["recv_mono_ns"] == 222
    assert event["recv_wall_iso"]
    assert event["run_tag"] == "t"


def test_missing_source_stamp_marks_unknown():
    state = LE.LocalizationObservationState()
    info = make_info()
    info.pop("header_stamp_ns")
    event = state.on_info(info, 1, 2)
    assert event["source_stamp_ns"] is None
    assert event["source_stamp_unknown"] is True


def test_accepted_then_zero_frame_keeps_cumulative_but_current_is_zero():
    state = LE.LocalizationObservationState()
    state.on_info(accepted_proximity_info(), 10, 1)
    state.on_info(make_info(), 20, 2)
    summary = state.summary(final=True, generated_wall_ns=30)
    assert summary["cumulative"]["info_events"] == 2
    assert summary["cumulative"]["backend_accepted_events"] == 1
    assert summary["cumulative"]["info_frames_with_accepted_ids"] == 1
    derived = summary["derived"]
    assert derived["historical_backend_accepted_total"] == 1
    assert derived["historical_accepted_persists_after_zero_frame"] is True
    assert derived["no_current_event"]["accepted_in_current_frame"] is False
    assert derived["state_hint"] == "historical_accepted_evidence_no_current_frame_event"
    json.dumps(summary, allow_nan=False)


def test_info_frame_raw_ids_and_derived_are_separate_from_cumulative():
    state = LE.LocalizationObservationState()
    event = state.on_info(accepted_proximity_info(), 10, 1)
    assert event["data"]["raw"]["proximity_detection_id"] == 29
    assert event["data"]["derived"]["accepted_ids"]["proximity_detection_id"] == 29
    assert event["data"]["derived"]["accepted_any"] is True
    summary = state.summary(final=False, generated_wall_ns=11)
    assert summary["derived"]["historical_backend_accepted_total"] == 1


def test_bayes_accepted_hypothesis_alone_is_not_backend_accepted():
    stats_values = list(DEFAULT_STATS_VALUES)
    stats_values[DEFAULT_STATS_KEYS.index(ACCEPTED_HYP_KEY)] = 5.0
    state = LE.LocalizationObservationState()
    state.on_info(make_info(stats_values=stats_values), 10, 1)
    summary = state.summary(final=False, generated_wall_ns=11)
    assert summary["cumulative"]["bayes_accepted_hypothesis_frames"] == 1
    assert summary["cumulative"]["backend_accepted_events"] == 0
    assert summary["derived"]["historical_backend_accepted_total"] == 0


def test_recent_events_are_bounded_but_cumulative_is_not():
    state = LE.LocalizationObservationState()
    for index in range(30):
        state.on_info(accepted_proximity_info(), 10 + index, index)
    summary = state.summary(final=False, generated_wall_ns=100)
    assert summary["cumulative"]["backend_accepted_events"] == 30
    assert len(summary["recent"]["accepted"]) <= LE.MAX_RECENT_ACCEPTED


# ---------------------------------------------------------------------------
# 小运动 / pending / 候选
# ---------------------------------------------------------------------------
def test_small_movement_without_candidate_is_not_failure():
    stats_values = list(DEFAULT_STATS_VALUES)
    stats_values[0] = 1.0
    state = LE.LocalizationObservationState()
    state.on_info(make_info(stats_values=stats_values), 10, 1)
    summary = state.summary(final=False, generated_wall_ns=11)
    assert summary["cumulative"]["small_movement_frames"] == 1
    assert summary["cumulative"]["small_movement_frames_without_candidate"] == 1
    assert summary["derived"]["small_movement_without_candidate_is_not_failure"] is True
    assert summary["derived"]["historical_backend_accepted_total"] == 0


def test_rosout_pending_is_not_accepted():
    state = LE.LocalizationObservationState()
    event = state.on_rosout("rtabmap", pending_text(), 30, 100, 200, source_stamp_ns=123)
    assert event is not None
    assert event["data"]["category"] == "localization_pending_confirmation"
    summary = state.summary(final=False, generated_wall_ns=300)
    assert summary["cumulative"]["pending_confirmation_events"] == 1
    assert summary["cumulative"]["pending_by_source"]["rosout"] == 1
    assert summary["cumulative"]["backend_accepted_events"] == 0
    assert summary["last"]["accepted_event"] is None
    assert summary["derived"]["state_hint"] == "pending_confirmation_only"


def test_backend_log_pending_is_not_accepted():
    state = LE.LocalizationObservationState()
    event = state.on_backend_log_line(pending_text(), 100, 1)
    assert event is not None
    assert event["data"]["category"] == "localization_pending_confirmation"
    summary = state.summary(final=False, generated_wall_ns=200)
    assert summary["cumulative"]["pending_by_source"]["backend_log"] == 1
    assert summary["cumulative"]["backend_accepted_events"] == 0


def test_candidate_link_added_is_not_accepted_and_does_not_clear_pending():
    state = LE.LocalizationObservationState()
    state.on_backend_log_line(pending_text(), 100, 1)
    link_line = (
        "\x1b[32m[ INFO] (2026-10-20 15:12:35.308) Rtabmap.cpp:3076::process() "
        "[Scan matching] Add local loop closure in SPACE (40->1) "
        "xyz=0.1,0.2,0.0 rpy=0.0,0.0,0.1\x1b[0m"
    )
    event = state.on_backend_log_line(link_line, 200, 2)
    assert event["data"]["category"] == "candidate_link_added"
    summary = state.summary(final=False, generated_wall_ns=300)
    assert summary["cumulative"]["candidate_link_added_events"] == 1
    assert summary["cumulative"]["backend_accepted_events"] == 0
    assert summary["last"]["accepted_event"] is None
    assert summary["last"]["candidate_link_added_event"] is not None
    assert summary["derived"]["candidate_link_added_total"] == 1
    # 候选 link 行不清理 pending 计数
    assert summary["derived"]["pending_since_last_accept"] == 1


def test_nonlocalization_rosout_is_ignored():
    state = LE.LocalizationObservationState()
    assert state.on_rosout("fastlio_mapping", "hello", 20, 1, 1) is None
    assert state.on_rosout("rtabmap", "rtabmap (12): Rate=1.00s", 20, 1, 1) is None


# ---------------------------------------------------------------------------
# initialpose 时间关联
# ---------------------------------------------------------------------------
def test_initialpose_time_only_correlation():
    state = LE.LocalizationObservationState()
    initial = state.on_initialpose(initialpose_payload(), 1_000_000_000_000, 1)
    assert initial["event"] == LE.EVENT_INITIALPOSE
    state.on_info(accepted_proximity_info(), 1_002_000_000_000, 2)
    summary = state.summary(final=False, generated_wall_ns=1_002_000_000_001)
    accepted = summary["last"]["accepted_event"]
    assert accepted["correlated_initial_pose"] is not None
    correlation = accepted["correlated_initial_pose"]
    assert correlation["initialpose_event_seq"] == initial["seq"]
    assert correlation["association"] == "time_only_not_causal"
    assert correlation["delta_s"] == pytest.approx(2.0)
    assert summary["derived"]["initialpose_seen"] is True


def test_no_initialpose_means_no_correlation():
    state = LE.LocalizationObservationState()
    state.on_info(accepted_proximity_info(), 10, 1)
    summary = state.summary(final=False, generated_wall_ns=11)
    assert summary["last"]["accepted_event"]["correlated_initial_pose"] is None
    assert summary["derived"]["last_accept_correlated_initialpose"] is None


# ---------------------------------------------------------------------------
# TF map->camera_init
# ---------------------------------------------------------------------------
def test_tf_first_value_dedupe_sign_equivalence_and_change():
    state = LE.LocalizationObservationState()
    translation = {"x": 0.1, "y": 0.2, "z": 0.0}
    quaternion = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    first = state.on_tf_map_to_camera_init(translation, quaternion, 10, 1, source_stamp_ns=5)
    assert first is not None and first["data"]["first_value"] is True
    assert first["data"]["translation_norm_m"] == pytest.approx(math.sqrt(0.05))
    # 同值重复（以及 q 与 -q）不重复落盘
    assert state.on_tf_map_to_camera_init(translation, quaternion, 11, 2) is None
    assert state.on_tf_map_to_camera_init(translation, dict(quaternion, w=-1.0), 12, 3) is None
    changed = state.on_tf_map_to_camera_init({"x": 0.2, "y": 0.2, "z": 0.0}, quaternion, 13, 4)
    assert changed is not None and changed["data"]["first_value"] is False
    assert changed["data"]["translation_delta_m"] == pytest.approx(0.1)
    summary = state.summary(final=False, generated_wall_ns=20)
    assert summary["cumulative"]["tf_updates"] == 4
    assert summary["cumulative"]["tf_change_events"] == 2
    assert summary["derived"]["tf_change_is_not_acceptance"] is True


def test_tf_rotation_angle_uses_sign_equivalent_quaternion():
    state = LE.LocalizationObservationState()
    identity = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    half = math.sin(math.pi / 4.0)
    rotated = {"x": 0.0, "y": 0.0, "z": half, "w": math.cos(math.pi / 4.0)}
    state.on_tf_map_to_camera_init({"x": 0.0, "y": 0.0, "z": 0.0}, identity, 1, 1)
    event = state.on_tf_map_to_camera_init({"x": 0.0, "y": 0.0, "z": 0.0}, rotated, 2, 2)
    assert event["data"]["rotation_delta_rad"] == pytest.approx(math.pi / 2.0, abs=1e-6)
    same_rotation = {"x": 0.0, "y": 0.0, "z": -half, "w": -math.cos(math.pi / 4.0)}
    assert state.on_tf_map_to_camera_init({"x": 0.0, "y": 0.0, "z": 0.0}, same_rotation, 3, 3) is None


def test_tf_change_only_does_not_imply_accepted():
    state = LE.LocalizationObservationState()
    state.on_tf_map_to_camera_init({"x": 1.0, "y": 0.0, "z": 0.0}, {"w": 1.0}, 10, 1)
    state.on_localization_pose(initialpose_payload(), 20, 2)
    summary = state.summary(final=False, generated_wall_ns=30)
    assert summary["cumulative"]["backend_accepted_events"] == 0
    assert summary["derived"]["historical_backend_accepted_total"] == 0
    assert summary["derived"]["state_hint"] == "map_tf_change_only_no_accept_evidence"


# ---------------------------------------------------------------------------
# 缺字段 / NaN / 键名兼容
# ---------------------------------------------------------------------------
def test_missing_info_ids_stay_null_and_unknown():
    info = make_info()
    for key in ("loop_closure_id", "proximity_detection_id", "landmark_id", "ref_id"):
        info.pop(key)
    state = LE.LocalizationObservationState()
    event = state.on_info(info, 1, 1)
    raw = event["data"]["raw"]
    assert raw["loop_closure_id"] is None
    assert raw["proximity_detection_id"] is None
    assert raw["landmark_id"] is None
    assert raw["ref_id"] is None
    assert event["data"]["derived"]["accepted_any"] is False
    assert set(event["data"]["derived"]["id_fields_unknown"]) == {
        "proximity_detection_id",
        "loop_closure_id",
        "landmark_id",
    }
    json.dumps(event, allow_nan=False)


def test_nonfinite_stats_are_sanitized_not_fabricated():
    stats_values = [float("nan"), float("inf")] + list(DEFAULT_STATS_VALUES[2:])
    state = LE.LocalizationObservationState()
    event = state.on_info(make_info(stats_values=stats_values), 1, 1)
    stats = event["data"]["raw"]["stats"]
    assert stats[SMALL_KEY] is None and stats[FAST_KEY] is None
    assert event["data"]["raw"]["stats_meta"]["nonfinite_or_invalid_values"] == 2
    assert event["data"]["derived"]["small_movement_frame"] is False
    json.dumps(event, allow_nan=False)


def test_stat_key_with_and_without_trailing_slash_are_accepted():
    state = LE.LocalizationObservationState()
    no_slash = make_info(
        stats_keys=["Memory/Small_movement", "Loop/RejectedHypothesis"],
        stats_values=[1.0, 0.0],
    )
    event = state.on_info(no_slash, 1, 1)
    assert event["data"]["derived"]["small_movement_frame"] is True


def test_candidate_rejected_icp_derived_and_unknown_fields():
    stats_values = list(DEFAULT_STATS_VALUES)
    stats_values[DEFAULT_STATS_KEYS.index(ICP_MULTI_KEY)] = 2.0
    stats_values[DEFAULT_STATS_KEYS.index(REJECTED_KEY)] = 1.0
    stats_values[DEFAULT_STATS_KEYS.index(OPT_ITER_KEY)] = 3.0
    state = LE.LocalizationObservationState()
    event = state.on_info(make_info(stats_values=stats_values), 1, 1)
    derived = event["data"]["derived"]
    assert derived["candidate_frame"] is True
    assert derived["candidate"]["space_detections_icp_multi"] == 2.0
    assert derived["rejected_frame"] is True
    assert derived["optimization"]["iterations"] == 3.0
    assert derived["icp"]["fitness"] is None
    assert derived["unknown_fields"]["icp_fitness"] == "unknown_not_in_info_stats"
    unknown = derived["unknown_fields"]
    assert unknown["candidate_fields_total"] == len(derived["candidate_fields_known"])
    assert unknown["candidate_fields_known_count"] == sum(
        1 for known in derived["candidate_fields_known"].values() if known
    )


# ---------------------------------------------------------------------------
# JSONL writer / summary 文件
# ---------------------------------------------------------------------------
def test_writer_does_not_overwrite_existing_pair_or_orphan_summary(tmp_path: Path):
    report = tmp_path / "probe_report.json"
    base_jsonl = tmp_path / "probe_report.localization_events.jsonl"
    base_summary = tmp_path / "probe_report.localization_summary.json"
    base_jsonl.write_text("EXISTING_JSONL\n", encoding="utf-8")
    base_summary.write_text("EXISTING_SUMMARY\n", encoding="utf-8")
    writer = LE.EventTimelineWriter(report, run_tag="run")
    assert writer.jsonl_path.name == "probe_report.localization_events__02.jsonl"
    assert writer.summary_path.name == "probe_report.localization_summary__02.json"
    state = LE.LocalizationObservationState(run_tag="run")
    event = state.on_info(accepted_proximity_info(), 1, 1)
    assert writer.write(event) is True
    assert writer.write_summary(state.summary(final=True, generated_wall_ns=2)) is True
    writer.close()
    assert base_jsonl.read_text(encoding="utf-8") == "EXISTING_JSONL\n"
    assert base_summary.read_text(encoding="utf-8") == "EXISTING_SUMMARY\n"
    lines = writer.jsonl_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1 and json.loads(lines[0])["seq"] == 1
    payload = json.loads(writer.summary_path.read_text(encoding="utf-8"))
    assert payload["schema"] == LE.SUMMARY_SCHEMA_VERSION


def test_writer_skips_only_orphan_summary(tmp_path: Path):
    report = tmp_path / "probe.json"
    orphan_summary = tmp_path / "probe.localization_summary.json"
    orphan_summary.write_text("ORPHAN\n", encoding="utf-8")
    writer = LE.EventTimelineWriter(report)
    assert writer.jsonl_path.name == "probe.localization_events__02.jsonl"
    assert writer.summary_path.name == "probe.localization_summary__02.json"
    writer.close()
    assert orphan_summary.read_text(encoding="utf-8") == "ORPHAN\n"


def test_writer_close_flushes_and_write_after_close_reports_error(tmp_path: Path):
    writer = LE.EventTimelineWriter(tmp_path / "probe.json", run_tag="r")
    state = LE.LocalizationObservationState()
    writer.write(state.on_info(make_info(), 1, 1))
    closed = writer.close()
    assert closed["closed"] is True
    assert closed["events_written"] == 1
    assert writer.write(state.on_info(make_info(), 2, 2)) is False
    assert writer.state["write_errors"] >= 1
    assert writer.state["last_error"] == "writer_closed"


def test_writer_summary_reserved_at_init(tmp_path: Path):
    writer = LE.EventTimelineWriter(tmp_path / "probe.json")
    assert writer.summary_path.is_file()
    writer.close()


# ---------------------------------------------------------------------------
# backend log tailer
# ---------------------------------------------------------------------------
def test_tailer_starts_at_eof_and_reads_only_new_lines(tmp_path: Path):
    log = tmp_path / "backend.log"
    log.write_text("OLD pending Localization was good waiting\n", encoding="utf-8")
    tailer = LE.BackendLogTailer(log)
    assert tailer.start() == []
    assert tailer.poll() == []
    with open(log, "a", encoding="utf-8") as handle:
        handle.write("NEW pending Localization was good waiting\n")
        handle.flush()
    items = tailer.poll()
    assert [item["kind"] for item in items] == ["line"]
    assert "NEW pending" in items[0]["line"]
    assert tailer.state["lines_read"] == 1


def test_tailer_reports_truncation_rotation_missing_reappeared(tmp_path: Path):
    log = tmp_path / "backend.log"
    log.write_text("a\nb\n", encoding="utf-8")
    tailer = LE.BackendLogTailer(log)
    tailer.start()
    # truncation：文件变短 -> 显式 truncation 事件并从头读
    log.write_text("c\n", encoding="utf-8")
    items = tailer.poll()
    kinds = [item["kind"] for item in items]
    assert "truncation" in kinds
    assert any(item.get("line") == "c" for item in items if item["kind"] == "line")
    # rotation：换 inode（原子替换）-> rotation 事件
    replacement = tmp_path / "replacement.log"
    replacement.write_text("d\n", encoding="utf-8")
    os.replace(replacement, log)
    items = tailer.poll()
    assert any(item["kind"] == "rotation" for item in items)
    # missing -> reappeared
    log.unlink()
    items = tailer.poll()
    assert [item["kind"] for item in items] == ["missing"]
    log.write_text("e\n", encoding="utf-8")
    items = tailer.poll()
    kinds = [item["kind"] for item in items]
    assert "reappeared" in kinds
    assert any(item.get("line") == "e" for item in items if item["kind"] == "line")
    state = tailer.state
    assert state["truncations"] >= 1
    assert state["rotations"] >= 1
    assert state["missing_events"] >= 1


def test_tailer_handles_partial_lines_and_overflow(tmp_path: Path):
    log = tmp_path / "backend.log"
    log.write_text("", encoding="utf-8")
    tailer = LE.BackendLogTailer(log, max_line_bytes=64)
    tailer.start()
    with open(log, "a", encoding="utf-8") as handle:
        handle.write("partial no newline")
    assert tailer.poll() == []
    with open(log, "a", encoding="utf-8") as handle:
        handle.write(" end\n")
    items = tailer.poll()
    assert [item["kind"] for item in items] == ["line"]
    assert "partial no newline end" in items[0]["line"]
    with open(log, "a", encoding="utf-8") as handle:
        handle.write("x" * 300 + "\n")
    items = tailer.poll()
    assert any(item["kind"] == "line_overflow" for item in items)


# ---------------------------------------------------------------------------
# pending observability 状态
# ---------------------------------------------------------------------------
def test_pending_observability_default_is_insufficient():
    state = LE.LocalizationObservationState()
    summary = state.summary(final=False, generated_wall_ns=1)
    observability = summary["pending_observability"]
    assert observability["pending_coverage"] == "rosout_only_partial"
    assert observability["complete"] is False
    assert "backend_log_path" in observability["note"]


def test_pending_observability_active_backend_log_is_never_complete():
    state = LE.LocalizationObservationState()
    state.set_backend_log_state(
        {"configured": True, "status": "active", "path": "/tmp/x.log", "lines_read": 1}
    )
    summary = state.summary(final=False, generated_wall_ns=1)
    observability = summary["pending_observability"]
    assert observability["pending_coverage"] == "rosout_plus_backend_log_tail"
    assert observability["complete"] is False


# ---------------------------------------------------------------------------
# 新 initialpose 后的“当前尝试”语义（历史累计保留）
# ---------------------------------------------------------------------------
def test_new_initialpose_keeps_history_but_marks_current_attempt_not_accepted():
    state = LE.LocalizationObservationState()
    state.on_info(accepted_proximity_info(), 1_000_000_000_000, 1)
    early = state.summary(final=False, generated_wall_ns=1_000_000_000_001)
    assert early["cumulative"]["backend_accepted_events"] == 1
    assert early["derived"]["accepted_since_latest_initialpose"] is None
    assert early["derived"]["pending_current_initialpose_attempt"] is None
    # 新的初值（在此之前存在历史 accepted，但这次初值之后还没有接受）
    state.on_initialpose(initialpose_payload(), 1_000_010_000_000, 2)
    state.on_info(make_info(), 1_000_020_000_000, 3)
    summary = state.summary(final=False, generated_wall_ns=1_000_030_000_000)
    derived = summary["derived"]
    assert summary["cumulative"]["backend_accepted_events"] == 1  # 历史累计不被清零
    assert derived["latest_initialpose_seq"] is not None
    assert derived["last_accepted_seq"] is not None
    assert derived["accepted_since_latest_initialpose"] is False
    assert derived["pending_current_initialpose_attempt"] is True
    assert derived["historical_backend_accepted_total"] == 1
    # 新初值之后出现接受 -> 当前尝试 accepted
    state.on_info(accepted_proximity_info(), 1_000_040_000_000, 4)
    after = state.summary(final=False, generated_wall_ns=1_000_050_000_000)
    assert after["cumulative"]["backend_accepted_events"] == 2
    assert after["derived"]["accepted_since_latest_initialpose"] is True
    assert after["derived"]["pending_current_initialpose_attempt"] is False


def test_zero_optimization_iterations_not_counted_as_optimization_frame():
    zero = make_info(stats_values=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    state = LE.LocalizationObservationState()
    state.on_info(zero, 1, 1)
    summary = state.summary(final=False, generated_wall_ns=2)
    assert summary["cumulative"]["optimization_frames"] == 0
    positive = make_info(stats_values=[0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
    state.on_info(positive, 3, 2)
    summary = state.summary(final=False, generated_wall_ns=4)
    assert summary["cumulative"]["optimization_frames"] == 1
    missing = make_info(stats_keys=[], stats_values=[])
    state.on_info(missing, 5, 3)
    summary = state.summary(final=False, generated_wall_ns=6)
    assert summary["cumulative"]["optimization_frames"] == 1


def test_backend_log_open_error_has_separate_counter():
    state = LE.LocalizationObservationState()
    event = state.on_backend_log_meta("open_error", {"error": "permission"}, 1, 1)
    summary = state.summary(final=False, generated_wall_ns=2)
    assert event["data"]["category"] == "backend_log_open_error"
    assert summary["cumulative"]["backend_log_error_events"] == 1
    assert summary["cumulative"]["backend_log_overflow_events"] == 0
