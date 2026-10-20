"""只读取证探针：核对 /Odometry 与 body 点云的 stamp/frame/字段/法线/非有限/TF 契约。

本节点**只订阅**，不发任何 topic、不发 TF、不改数据。用途：

* 证明 RTAB-Map 的输入是与 /Odometry 同 stamp 的去畸变单帧 body 云；
* 证明 FAST-LIO 发布的 body 云确实携带未估计的零法线字段（XYZI 适配的必要性）；
* 证明适配后字段恰为 XYZI 且 stamp/frame/点数语义保持；
* 证明 TF 只有 camera_init->body（前端）与 map->camera_init（后端），无 base_link/odom。

报告按 report_period_s 周期原子写盘；节点被 SIGINT 正常关停时写 final 报告。
"""

from __future__ import annotations

import json
import os
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import Log as RosLog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rtabmap_msgs.msg import Info as RtabmapInfo
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import tf2_ros

from hei_robot_slam import localization_events as LE
from hei_robot_slam import slam_contract as C
from hei_robot_slam.cloud_xyzi import field_names, structured_points


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


def _qos(reliability: str, durability: str = "volatile", depth: int = 100) -> QoSProfile:
    rel = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }.get(reliability, ReliabilityPolicy.SYSTEM_DEFAULT)
    dur = {
        "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        "volatile": DurabilityPolicy.VOLATILE,
    }.get(durability, DurabilityPolicy.VOLATILE)
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth, reliability=rel, durability=dur)


def _cloud_geometry(msg: PointCloud2) -> dict[str, Any]:
    """点云几何/字段统计（不修改消息）。"""
    names = field_names(msg)
    result: dict[str, Any] = {
        "frame_id": str(msg.header.frame_id),
        "stamp_ns": _stamp_ns(msg.header.stamp),
        "fields": names,
        "width": int(msg.width),
        "height": int(msg.height),
        "point_step": int(msg.point_step),
        "points": int(msg.width) * int(msg.height),
    }
    if not all(name in names for name in ("x", "y", "z")):
        result["missing_xyz"] = True
        return result
    points = structured_points(msg)
    x = np.asarray(points["x"], dtype=np.float64)
    y = np.asarray(points["y"], dtype=np.float64)
    z = np.asarray(points["z"], dtype=np.float64)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    result["nonfinite_xyz"] = int(points.shape[0] - int(finite.sum()))
    if all(name in names for name in ("normal_x", "normal_y", "normal_z")):
        nx = np.asarray(points["normal_x"], dtype=np.float64)
        ny = np.asarray(points["normal_y"], dtype=np.float64)
        nz = np.asarray(points["normal_z"], dtype=np.float64)
        norm2 = nx * nx + ny * ny + nz * nz
        result["has_normal_fields"] = True
        result["nonzero_normal_points"] = int(np.count_nonzero(np.isfinite(norm2) & (norm2 > 0.0)))
    else:
        result["has_normal_fields"] = False
    return result


def _ros_msg_to_plain(value: Any, depth: int = 0) -> Any:
    """把 ROS 消息递归转成 JSON 友好纯数据（用于 /info 全量事件；不改变数值语义）。"""
    if depth > 10:
        return "<max_depth>"
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (bytes, bytearray)):
        return {"bytes_length": len(value)}
    if isinstance(value, (list, tuple)):
        return [_ros_msg_to_plain(item, depth + 1) for item in value]
    fields = getattr(value, "get_fields_and_field_types", None)
    if callable(fields):
        try:
            names = fields().keys()
        except (AttributeError, TypeError):
            return str(value)
        return {str(name): _ros_msg_to_plain(getattr(value, name, None), depth + 1) for name in names}
    return str(value)


def _pose_payload(msg: PoseWithCovarianceStamped) -> dict[str, Any]:
    pose = msg.pose.pose
    return {
        "header_stamp_ns": _stamp_ns(msg.header.stamp),
        "frame_id": str(msg.header.frame_id),
        "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        },
        "covariance": list(msg.pose.covariance),
    }


def _info_payload(msg: RtabmapInfo) -> dict[str, Any]:
    transform = msg.loop_closure_transform
    return {
        "header_stamp_ns": _stamp_ns(msg.header.stamp),
        "header_frame_id": str(msg.header.frame_id),
        "ref_id": int(msg.ref_id),
        "loop_closure_id": int(msg.loop_closure_id),
        "proximity_detection_id": int(msg.proximity_detection_id),
        "landmark_id": int(msg.landmark_id),
        "loop_closure_transform": _ros_msg_to_plain(transform),
        "wm_state": [int(item) for item in msg.wm_state],
        # 只保留定位相关字段 + 全量 stats；posterior/likelihood/odom_cache 等与
        # accepted/pending 判定无关，为控制 JSONL 体积不进入事件。
        "stats_keys": [str(item) for item in msg.stats_keys],
        "stats_values": [float(item) for item in msg.stats_values],
    }


class CloudProbe(Node):
    def __init__(self) -> None:
        super().__init__("slam_cloud_probe")
        self.declare_parameter("report_path", "")
        self.declare_parameter("report_period_s", 2.0)
        self.declare_parameter("odom_topic", C.LIO_ODOM_TOPIC)
        self.declare_parameter("body_cloud_topic", C.LIO_BODY_CLOUD_TOPIC)
        self.declare_parameter("adapted_cloud_topic", C.SLAM_CLOUD_TOPIC)
        self.declare_parameter("localization_pose_topic", "/localization_pose")
        self.declare_parameter("map_cloud_topic", "/cloud_map")
        self.declare_parameter("map_path_topic", "/mapPath")
        self.declare_parameter("assembled_cloud_topic", "")
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("localization_events_enabled", True)
        self.declare_parameter("info_topic", "/info")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("rosout_topic", "/rosout")
        self.declare_parameter("backend_log_path", "")
        self.declare_parameter("backend_log_poll_s", 0.5)
        self.declare_parameter("backend_log_max_bytes_per_poll", 262144)
        self.declare_parameter("backend_log_max_line_bytes", 8192)
        self.declare_parameter("run_tag", "")

        report_path = str(self.get_parameter("report_path").value)
        self._report_path = Path(report_path) if report_path else None
        self._report_period = max(0.5, float(self.get_parameter("report_period_s").value))
        reliability = str(self.get_parameter("qos_reliability").value)
        qos = _qos(reliability)
        tf_qos = _qos(reliability, depth=200)

        self._odom_stamps: deque[int] = deque(maxlen=2000)
        self._body_stamps: deque[int] = deque(maxlen=2000)
        self._odom: dict[str, Any] = {
            "count": 0,
            "frames": set(),
            "first_stamp_ns": None,
            "last_stamp_ns": None,
            "stamp_regressions": 0,
            "nonfinite_position": 0,
            "nonfinite_quaternion": 0,
            "quat_norm_min": None,
            "quat_norm_max": None,
        }
        self._body: dict[str, Any] = {
            "count": 0,
            "points": 0,
            "nonfinite_xyz": 0,
            "frames": set(),
            "first_stamp_ns": None,
            "last_stamp_ns": None,
            "stamp_regressions": 0,
            "fields_last": [],
            "has_normal_fields": False,
            "nonzero_normal_points": 0,
            "stamp_matches_odom": 0,
            "stamp_without_odom": 0,
        }
        self._adapted: dict[str, Any] = {
            "count": 0,
            "points": 0,
            "frames": set(),
            "fields_last": [],
            "fields_ok": 0,
            "fields_bad": 0,
            "stamp_matches_body": 0,
            "stamp_without_body": 0,
            "first_stamp_ns": None,
            "last_stamp_ns": None,
        }
        self._localization_pose: dict[str, Any] = {"count": 0, "frames": set()}
        self._backend_cloud: dict[str, Any] = {
            "count": 0,
            "nonempty_messages": 0,
            "points": 0,
            "frames": set(),
            "last_stamp_ns": None,
        }
        self._backend_path: dict[str, Any] = {
            "count": 0,
            "nonempty_messages": 0,
            "poses": 0,
            "max_poses_in_message": 0,
            "frames": set(),
        }
        self._assembled_cloud: dict[str, Any] = {
            "count": 0,
            "nonempty_messages": 0,
            "points": 0,
            "frames": set(),
        }
        self._tf_pairs: dict[str, int] = {}
        self._tf_pair_stamps: dict[str, dict[str, Any]] = {}
        self._tf_transforms = 0
        self._tf_forbidden: set[str] = set()
        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)
        self._tf_lookup: dict[str, Any] = {
            "latest_attempts": 0,
            "latest_success": 0,
            "latest_failures": 0,
            "latest_last_error": "",
            "latest_translation": None,
            "at_cloud_stamp_attempts": 0,
            "at_cloud_stamp_success": 0,
            "at_cloud_stamp_failures": 0,
            "at_cloud_stamp_last_error": "",
            "at_cloud_stamp_translation": None,
        }

        # ---- 定位连续取证（任务 HEI-SLAM-OBS-20261020）---------------------
        self._run_tag = str(self.get_parameter("run_tag").value).strip()
        self._obs: LE.LocalizationObservationState | None = None
        self._loc_writer: LE.EventTimelineWriter | None = None
        self._backend_tail: LE.BackendLogTailer | None = None
        self._backend_timer = None
        self._loc_events_status: dict[str, Any] = {
            "enabled": False,
            "status": "disabled_no_report_path",
            "error": None,
            "jsonl_path": None,
            "summary_path": None,
        }
        events_enabled = bool(self.get_parameter("localization_events_enabled").value)
        if self._report_path is None:
            self._loc_events_status["status"] = "disabled_no_report_path"
        elif not events_enabled:
            self._loc_events_status["status"] = "disabled_by_parameter"
        else:
            try:
                self._loc_writer = LE.EventTimelineWriter(self._report_path, self._run_tag)
                self._obs = LE.LocalizationObservationState(
                    run_tag=self._run_tag, rosout_subscribed=True
                )
                self._obs.set_writer_state(self._loc_writer.state)
                self._loc_events_status.update(
                    {
                        "enabled": True,
                        "status": "active",
                        "jsonl_path": str(self._loc_writer.jsonl_path),
                        "summary_path": str(self._loc_writer.summary_path),
                    }
                )
            except (OSError, RuntimeError) as exc:
                self._obs = None
                self._loc_writer = None
                self._loc_events_status.update(
                    {"enabled": False, "status": "failed", "error": f"{type(exc).__name__}: {exc}"}
                )
                self.get_logger().error(
                    f"定位事件时间线初始化失败（旧探针功能继续，不写 JSONL）：{exc}"
                )
        if self._obs is not None and self._loc_writer is not None:
            backend_log_path = str(self.get_parameter("backend_log_path").value).strip()
            if backend_log_path:
                self._backend_tail = LE.BackendLogTailer(
                    backend_log_path,
                    max_bytes_per_poll=int(self.get_parameter("backend_log_max_bytes_per_poll").value),
                    max_line_bytes=int(self.get_parameter("backend_log_max_line_bytes").value),
                )
                self._obs.set_backend_log_state(self._backend_tail.state)
                for meta in self._backend_tail.start():
                    self._write_loc_event(
                        self._obs.on_backend_log_meta(
                            str(meta.get("kind", "open_error")),
                            dict(meta.get("detail") or {}),
                            *self._now_ns(),
                        )
                    )
                self._obs.set_backend_log_state(self._backend_tail.state)
                self._backend_timer = self.create_timer(
                    max(0.2, float(self.get_parameter("backend_log_poll_s").value)),
                    self._poll_backend_log,
                )
                self.get_logger().info(
                    "定位取证：backend_log_path 只读 tail 已启动（从启动时 EOF 开始）："
                    f"{backend_log_path}"
                )
            self.get_logger().info(
                "定位取证已启用："
                f"JSONL={self._loc_writer.jsonl_path} summary={self._loc_writer.summary_path} "
                f"backend_log={'enabled' if self._backend_tail else 'disabled(rosout-only-partial)'}"
            )

        self._subs = [
            self.create_subscription(
                Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, qos
            ),
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("body_cloud_topic").value),
                self._body_cb,
                qos,
            ),
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("adapted_cloud_topic").value),
                self._adapted_cb,
                qos,
            ),
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("localization_pose_topic").value),
                self._pose_cb,
                qos,
            ),
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("map_cloud_topic").value),
                self._backend_cloud_cb,
                qos,
            ),
            self.create_subscription(
                NavPath, str(self.get_parameter("map_path_topic").value), self._backend_path_cb, qos
            ),
            self.create_subscription(TFMessage, "/tf", self._tf_cb, tf_qos),
            self.create_subscription(
                TFMessage, "/tf_static", self._tf_static_cb, _qos(reliability, "transient_local", 200)
            ),
        ]
        assembled_topic = str(self.get_parameter("assembled_cloud_topic").value).strip()
        if assembled_topic:
            self._subs.append(
                self.create_subscription(PointCloud2, assembled_topic, self._assembled_cloud_cb, qos)
            )
        if self._obs is not None:
            info_topic = str(self.get_parameter("info_topic").value)
            initialpose_topic = str(self.get_parameter("initialpose_topic").value)
            rosout_topic = str(self.get_parameter("rosout_topic").value)
            self._subs.extend(
                [
                    self.create_subscription(RtabmapInfo, info_topic, self._info_cb, _qos(reliability, depth=20)),
                    self.create_subscription(
                        PoseWithCovarianceStamped, initialpose_topic, self._initialpose_cb, _qos(reliability, depth=10)
                    ),
                    self.create_subscription(
                        RosLog, rosout_topic, self._rosout_cb, _qos("reliable", depth=200)
                    ),
                ]
            )
        self._timer = self.create_timer(self._report_period, self._write_report)
        self.get_logger().info(
            "slam_cloud_probe 启动（只读）：odom/body_cloud/adapted_cloud/tf/localization_pose；"
            f"report={self._report_path}"
        )
        if self._obs is not None and self._loc_writer is not None:
            self.get_logger().info(
                "定位取证文件："
                f"{self._loc_writer.jsonl_path} ；{self._loc_writer.summary_path}"
            )

    # ---- callbacks -------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        state = self._odom
        stamp = _stamp_ns(msg.header.stamp)
        state["count"] += 1
        state["frames"].add(str(msg.header.frame_id))
        if state["last_stamp_ns"] is not None and stamp < state["last_stamp_ns"]:
            state["stamp_regressions"] += 1
        if state["first_stamp_ns"] is None:
            state["first_stamp_ns"] = stamp
        state["last_stamp_ns"] = stamp
        self._odom_stamps.append(stamp)

        position = msg.pose.pose.position
        if not all(np.isfinite([position.x, position.y, position.z])):
            state["nonfinite_position"] += 1
        quat = msg.pose.pose.orientation
        norm = float(np.sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w))
        if not np.isfinite(norm):
            state["nonfinite_quaternion"] += 1
        else:
            state["quat_norm_min"] = norm if state["quat_norm_min"] is None else min(state["quat_norm_min"], norm)
            state["quat_norm_max"] = norm if state["quat_norm_max"] is None else max(state["quat_norm_max"], norm)

    def _body_cb(self, msg: PointCloud2) -> None:
        state = self._body
        geometry = _cloud_geometry(msg)
        stamp = int(geometry["stamp_ns"])
        state["count"] += 1
        state["points"] += int(geometry["points"])
        state["nonfinite_xyz"] += int(geometry.get("nonfinite_xyz", 0))
        state["frames"].add(str(msg.header.frame_id))
        if state["last_stamp_ns"] is not None and stamp < state["last_stamp_ns"]:
            state["stamp_regressions"] += 1
        if state["first_stamp_ns"] is None:
            state["first_stamp_ns"] = stamp
        state["last_stamp_ns"] = stamp
        state["fields_last"] = list(geometry["fields"])
        state["has_normal_fields"] = bool(geometry.get("has_normal_fields", False))
        state["nonzero_normal_points"] += int(geometry.get("nonzero_normal_points", 0))
        if stamp in self._odom_stamps:
            state["stamp_matches_odom"] += 1
        else:
            state["stamp_without_odom"] += 1
        self._body_stamps.append(stamp)

    def _adapted_cb(self, msg: PointCloud2) -> None:
        state = self._adapted
        geometry = _cloud_geometry(msg)
        stamp = int(geometry["stamp_ns"])
        state["count"] += 1
        state["points"] += int(geometry["points"])
        state["frames"].add(str(msg.header.frame_id))
        state["fields_last"] = list(geometry["fields"])
        if tuple(geometry["fields"]) == C.CLOUD_ADAPTED_FIELDS:
            state["fields_ok"] += 1
        else:
            state["fields_bad"] += 1
        if state["first_stamp_ns"] is None:
            state["first_stamp_ns"] = stamp
        state["last_stamp_ns"] = stamp
        if stamp in self._body_stamps:
            state["stamp_matches_body"] += 1
        else:
            state["stamp_without_body"] += 1

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._localization_pose["count"] += 1
        self._localization_pose["frames"].add(str(msg.header.frame_id))
        if self._obs is not None:
            self._write_loc_event(
                self._obs.on_localization_pose(
                    _pose_payload(msg), *self._now_ns()
                )
            )

    # ---- localization observability（HEI-SLAM-OBS-20261020）--------------
    def _now_ns(self) -> tuple[int, int]:
        return LE.now_times_ns()

    def _write_loc_event(self, event: dict[str, Any] | None) -> None:
        if self._obs is None or event is None:
            return
        if self._loc_writer is not None:
            if not self._loc_writer.write(event):
                self.get_logger().warning(
                    f"定位事件 JSONL 写入失败：{self._loc_writer.state.get('last_error')}"
                )
            self._obs.set_writer_state(self._loc_writer.state)

    def _info_cb(self, msg: RtabmapInfo) -> None:
        if self._obs is None:
            return
        self._write_loc_event(self._obs.on_info(_info_payload(msg), *self._now_ns()))

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if self._obs is None:
            return
        self._write_loc_event(self._obs.on_initialpose(_pose_payload(msg), *self._now_ns()))

    def _rosout_cb(self, msg: RosLog) -> None:
        if self._obs is None:
            return
        if "rtabmap" not in str(msg.name).lower():
            return
        self._write_loc_event(
            self._obs.on_rosout(
                msg.name,
                msg.msg,
                msg.level,
                *self._now_ns(),
                source_stamp_ns=_stamp_ns(msg.stamp),
            )
        )

    def _poll_backend_log(self) -> None:
        if self._obs is None or self._backend_tail is None:
            return
        try:
            items = self._backend_tail.poll()
        except Exception as exc:  # noqa: BLE001 - 取证失败不能打断探针核心功能
            self.get_logger().warning(f"backend_log tail 读取失败：{exc}")
            return
        for item in items:
            wall_ns, mono_ns = self._now_ns()
            kind = str(item.get("kind", ""))
            if kind == "line":
                event = self._obs.on_backend_log_line(str(item.get("line", "")), wall_ns, mono_ns)
            else:
                event = self._obs.on_backend_log_meta(kind, dict(item.get("detail") or {}), wall_ns, mono_ns)
            self._write_loc_event(event)
        self._obs.set_backend_log_state(self._backend_tail.state)

    def _observability_snapshot(self, final: bool) -> dict[str, Any]:
        summary = self._observability_summary(final=final)
        if summary is not None:
            summary["event_status"] = dict(self._loc_events_status)
            return summary
        return {
            "schema": LE.SUMMARY_SCHEMA_VERSION,
            "enabled": False,
            "status": self._loc_events_status["status"],
            "error": self._loc_events_status["error"],
            "writer": None,
            "pending_observability": {
                "rosout_subscribed": False,
                "backend_log": {"configured": False, "status": "not_configured"},
                "pending_coverage": "disabled",
                "complete": False,
            },
            "note": "定位连续取证未启用；status 区分 disabled_no_report_path/disabled_by_parameter/failed",
        }

    def _observability_summary(self, final: bool) -> dict[str, Any] | None:
        if self._obs is None:
            return None
        if self._loc_writer is not None:
            self._obs.set_writer_state(self._loc_writer.state)
        return self._obs.summary(final=final, generated_wall_ns=time.time_ns())

    def _write_observability_summary(self, final: bool) -> None:
        summary = self._observability_summary(final)
        if summary is None or self._loc_writer is None:
            return
        summary["event_status"] = dict(self._loc_events_status)
        if not self._loc_writer.write_summary(summary):
            self.get_logger().warning(
                f"定位取证 summary 写入失败：{self._loc_writer.state.get('last_error')}"
            )
        self._obs.set_writer_state(self._loc_writer.state)

    def _record_tf(self, msg: TFMessage) -> None:
        obs_times = self._now_ns() if self._obs is not None else None
        for transform in msg.transforms:
            parent = str(transform.header.frame_id)
            child = str(transform.child_frame_id)
            key = f"{parent}->{child}"
            self._tf_pairs[key] = self._tf_pairs.get(key, 0) + 1
            stamp_ns = _stamp_ns(transform.header.stamp)
            entry = self._tf_pair_stamps.get(key)
            if entry is None:
                self._tf_pair_stamps[key] = {"count": 1, "first_stamp_ns": stamp_ns, "last_stamp_ns": stamp_ns}
            else:
                entry["count"] += 1
                entry["first_stamp_ns"] = min(entry["first_stamp_ns"], stamp_ns)
                entry["last_stamp_ns"] = max(entry["last_stamp_ns"], stamp_ns)
            self._tf_transforms += 1
            for frame in C.FORBIDDEN_FRAMES:
                if frame in (parent, child):
                    self._tf_forbidden.add(frame)
            if (
                self._obs is not None
                and obs_times is not None
                and parent == C.MAP_FRAME
                and child == C.ODOM_FRAME
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._write_loc_event(
                    self._obs.on_tf_map_to_camera_init(
                        {"x": translation.x, "y": translation.y, "z": translation.z},
                        {"x": rotation.x, "y": rotation.y, "z": rotation.z, "w": rotation.w},
                        obs_times[0],
                        obs_times[1],
                        source_stamp_ns=stamp_ns,
                    )
                )

    def _tf_cb(self, msg: TFMessage) -> None:
        self._record_tf(msg)

    def _tf_static_cb(self, msg: TFMessage) -> None:
        self._record_tf(msg)

    def _backend_cloud_cb(self, msg: PointCloud2) -> None:
        geometry = _cloud_geometry(msg)
        state = self._backend_cloud
        state["count"] += 1
        points = int(geometry["points"])
        state["points"] += points
        if points > 0:
            state["nonempty_messages"] += 1
        state["frames"].add(str(msg.header.frame_id))
        state["last_stamp_ns"] = int(geometry["stamp_ns"])

    def _backend_path_cb(self, msg: NavPath) -> None:
        state = self._backend_path
        state["count"] += 1
        poses = len(msg.poses)
        state["poses"] += poses
        state["max_poses_in_message"] = max(state["max_poses_in_message"], poses)
        if poses > 0:
            state["nonempty_messages"] += 1
        state["frames"].add(str(msg.header.frame_id))

    def _assembled_cloud_cb(self, msg: PointCloud2) -> None:
        geometry = _cloud_geometry(msg)
        state = self._assembled_cloud
        state["count"] += 1
        points = int(geometry["points"])
        state["points"] += points
        if points > 0:
            state["nonempty_messages"] += 1
        state["frames"].add(str(msg.header.frame_id))

    def _clock_snapshot(self) -> dict[str, Any]:
        now_ns = int(self.get_clock().now().nanoseconds)
        odom_last = self._odom.get("last_stamp_ns")
        body_last = self._body.get("last_stamp_ns")
        map_to_odom = self._tf_pair_stamps.get(f"{C.MAP_FRAME}->{C.ODOM_FRAME}", {})
        return {
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
            "now_ns": now_ns,
            "now_minus_odom_last_s": (
                round((now_ns - int(odom_last)) / 1e9, 3) if odom_last is not None else None
            ),
            "now_minus_body_cloud_last_s": (
                round((now_ns - int(body_last)) / 1e9, 3) if body_last is not None else None
            ),
            "map_to_odom_tf_last_stamp_ns": map_to_odom.get("last_stamp_ns"),
            "odom_last_stamp_ns": odom_last,
            "tf_and_odom_same_time_domain": (
                None
                if not map_to_odom or odom_last is None
                else abs(int(map_to_odom.get("last_stamp_ns", 0)) - int(odom_last)) < 2_000_000_000
            ),
        }

    # ---- report ---------------------------------------------------------
    def _snapshot(self, final: bool) -> dict[str, Any]:
        def _sorted(values: Any) -> list[str]:
            return sorted(str(value) for value in values)

        body = dict(self._body)
        body["frames"] = _sorted(body["frames"])
        adapted = dict(self._adapted)
        adapted["frames"] = _sorted(adapted["frames"])
        odom = dict(self._odom)
        odom["frames"] = _sorted(odom["frames"])
        pose = dict(self._localization_pose)
        pose["frames"] = _sorted(pose["frames"])
        backend_cloud = dict(self._backend_cloud)
        backend_cloud["frames"] = _sorted(backend_cloud["frames"])
        backend_path = dict(self._backend_path)
        backend_path["frames"] = _sorted(backend_path["frames"])
        assembled = dict(self._assembled_cloud)
        assembled["frames"] = _sorted(assembled["frames"])

        tf_pairs = dict(sorted(self._tf_pairs.items()))
        expected_pairs = [f"{C.ODOM_FRAME}->{C.BASE_FRAME}", f"{C.MAP_FRAME}->{C.ODOM_FRAME}"]
        return {
            "kind": "mid360s_slam_cloud_probe",
            "final": bool(final),
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "contract": {
                "expected_tf_pairs": expected_pairs,
                "forbidden_frames": list(C.FORBIDDEN_FRAMES),
                "adapted_fields": list(C.CLOUD_ADAPTED_FIELDS),
                "lio_body_cloud_topic": C.LIO_BODY_CLOUD_TOPIC,
                "adapter_topic": C.SLAM_CLOUD_TOPIC,
            },
            "odom": odom,
            "body_cloud": body,
            "adapted_cloud": adapted,
            "localization_pose": pose,
            "backend_map_cloud": backend_cloud,
            "backend_map_path": backend_path,
            "assembled_cloud": assembled,
            "tf": {
                "pairs": tf_pairs,
                "pair_stamp_ranges": dict(sorted(self._tf_pair_stamps.items())),
                "transforms": int(self._tf_transforms),
                "forbidden_frames_observed": sorted(self._tf_forbidden),
                "lookup_map_to_body": dict(self._tf_lookup),
            },
            "clock": self._clock_snapshot(),
            "localization_observability": self._observability_snapshot(final=final),
        }

    def _do_tf_lookup(self) -> None:
        """证明 RViz 会用的 map->body 查询在当前时间域可用（不造假 TF）。"""
        state = self._tf_lookup
        state["latest_attempts"] += 1
        try:
            tf = self._tf_buffer.lookup_transform("map", "body", Time())
            state["latest_success"] += 1
            state["latest_translation"] = {
                "x": round(float(tf.transform.translation.x), 4),
                "y": round(float(tf.transform.translation.y), 4),
                "z": round(float(tf.transform.translation.z), 4),
                "stamp_ns": _stamp_ns(tf.header.stamp),
            }
        except Exception as exc:  # noqa: BLE001 - 记录任意 TF 查询失败
            state["latest_failures"] += 1
            state["latest_last_error"] = str(exc)[:300]
        cloud_stamp = self._body.get("last_stamp_ns")
        if cloud_stamp:
            state["at_cloud_stamp_attempts"] += 1
            try:
                tf = self._tf_buffer.lookup_transform("map", "body", Time(nanoseconds=int(cloud_stamp)))
                state["at_cloud_stamp_success"] += 1
                state["at_cloud_stamp_translation"] = {
                    "x": round(float(tf.transform.translation.x), 4),
                    "y": round(float(tf.transform.translation.y), 4),
                    "z": round(float(tf.transform.translation.z), 4),
                    "stamp_ns": _stamp_ns(tf.header.stamp),
                }
            except Exception as exc:  # noqa: BLE001
                state["at_cloud_stamp_failures"] += 1
                state["at_cloud_stamp_last_error"] = str(exc)[:300]

    def _write_report(self) -> None:
        if self._report_path is None:
            return
        try:
            self._do_tf_lookup()
            self._report_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = str(self._report_path) + ".tmp"
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(self._snapshot(final=False), handle, ensure_ascii=False, indent=1)
                handle.write("\n")
            os.replace(tmp, self._report_path)
            self._write_observability_summary(final=False)
        except OSError as exc:
            self.get_logger().warning(f"探针报告写入失败：{exc}")

    def finalize(self) -> None:
        # 关停顺序：最后 poll 后端日志 -> close JSONL -> 更新 writer 状态 -> 写 final summary。
        # summary 写盘不依赖 JSONL 句柄，因此能在 close 之后反映最终 closed/错误状态。
        if self._obs is not None and self._backend_tail is not None:
            try:
                self._poll_backend_log()
            except Exception as exc:  # noqa: BLE001 - 关停阶段不能因取证失败丢报告
                self.get_logger().warning(f"关停前 backend_log 最后 poll 失败：{exc}")
        closed_state = None
        if self._loc_writer is not None:
            closed_state = self._loc_writer.close()
            if self._obs is not None:
                self._obs.set_writer_state(closed_state)
        if self._report_path is None:
            return
        try:
            self._report_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = str(self._report_path) + ".tmp"
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(self._snapshot(final=True), handle, ensure_ascii=False, indent=1)
                handle.write(chr(10))
            os.replace(tmp, self._report_path)
            self.get_logger().info(f"探针最终报告已写入：{self._report_path}")
        except OSError as exc:
            self.get_logger().warning(f"探针最终报告写入失败：{exc}")
        finally:
            self._write_observability_summary(final=True)
        if closed_state is not None:
            self.get_logger().info(
                "定位取证已关闭："
                f"jsonl={closed_state.get('jsonl_path')} "
                f"summary={closed_state.get('summary_path')} "
                f"events={closed_state.get('events_written')} "
                f"errors={closed_state.get('write_errors')}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()