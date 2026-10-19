"""短回放 /info + /localization_pose 取证收集器（只读，不修改地图）。

用途：区分“地图加载与发布冒烟”与“实际几何定位接受”。
证据来自 rtabmap_msgs/Info 的 ref_id / loop_closure_id / proximity_detection_id /
stats 中 Localization 与 Loop 相关键，以及 /localization_pose 的位姿与协方差。
初始位姿：确认 /initialpose 有订阅者后才发布一次（不固定 sleep 假成功）。
"""
from __future__ import annotations

import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info

INTEREST_PREFIXES = ("Localization", "Loop/", "Proximity/", "Odom/", "Keypoint/")


def parse_args() -> dict:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-s", type=float, default=38.0)
    parser.add_argument("--publish-initialpose", action="store_true")
    parser.add_argument("--initialpose-after-s", type=float, default=15.0)
    return vars(parser.parse_args())


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("localization_info_collector")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.infos: list = []
        self.poses: list = []
        self.initial_pose_published_at = None
        self.info_sub = self.create_subscription(Info, "/info", self._info_cb, qos)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/localization_pose", self._pose_cb, qos
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )

    def _stamp(self, header) -> float:
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def _info_cb(self, msg: Info) -> None:
        try:
            self._info_record(msg)
        except Exception as exc:  # noqa: BLE001 - 取证节点不能被单条消息打崩
            self.get_logger().warning(f"info record failed: {exc}")

    def _info_record(self, msg: Info) -> None:
        stats = {}
        for key, value in zip(msg.stats_keys, msg.stats_values):
            if any(str(key).startswith(prefix) for prefix in INTEREST_PREFIXES):
                stats[str(key)] = float(value)
        self.infos.append(
            {
                "stamp": self._stamp(msg.header),
                "ref_id": int(msg.ref_id),
                "loop_closure_id": int(msg.loop_closure_id),
                "proximity_detection_id": int(msg.proximity_detection_id),
                "landmark_id": int(msg.landmark_id),
                "wm_state_length": len(msg.wm_state),
                "stats": stats,
            }
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        try:
            self._pose_record(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"pose record failed: {exc}")

    def _pose_record(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        cov = list(msg.pose.covariance)
        self.poses.append(
            {
                "stamp": self._stamp(msg.header),
                "frame_id": msg.header.frame_id,
                "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w,
                },
                "cov_diag": [cov[0], cov[7], cov[14], cov[21], cov[28], cov[35]],
            }
        )

    def subscriber_count(self) -> int:
        try:
            return len(self.get_subscriptions_info_by_topic("/initialpose"))
        except Exception:
            return 0

    def publish_initial_pose(self) -> bool:
        if self.subscriber_count() == 0:
            return False
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.initial_pose_published_at = self._stamp(msg.header)
        return True


def summarize(infos: list, poses: list) -> dict:
    ref_ids = sorted({item["ref_id"] for item in infos if item["ref_id"] > 0})
    proximity = [item for item in infos if item["proximity_detection_id"] > 0]
    closures = [item for item in infos if item["loop_closure_id"] > 0]
    stats_last = infos[-1]["stats"] if infos else {}
    return {
        "info_count": len(infos),
        "info_with_ref_id_gt0": sum(1 for item in infos if item["ref_id"] > 0),
        "ref_ids_seen": ref_ids,
        "info_with_proximity_detection_id": len(proximity),
        "proximity_detection_ids": [item["proximity_detection_id"] for item in proximity],
        "info_with_loop_closure_id": len(closures),
        "loop_closure_ids": [item["loop_closure_id"] for item in closures],
        "wm_state_lengths": sorted({item["wm_state_length"] for item in infos}),
        "stats_last": stats_last,
        "localization_pose_count": len(poses),
        "localization_pose_frames": sorted({item["frame_id"] for item in poses}),
        "first_pose": poses[0] if poses else None,
        "last_pose": poses[-1] if poses else None,
    }


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Collector()
    started = time.monotonic()
    published = False
    result = {"kind": "localization_info_collector", "args": args}
    try:
        while rclpy.ok() and (time.monotonic() - started) < args["duration_s"]:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - started
            if (
                args["publish_initialpose"]
                and not published
                and elapsed >= args["initialpose_after_s"]
            ):
                published = node.publish_initial_pose()
                if published:
                    result["initialpose_published"] = True
                    result["initialpose_subscriber_count"] = node.subscriber_count()
                    result["initialpose_elapsed_s"] = round(elapsed, 2)
        result["initialpose_published"] = bool(published)
        if not published and args["publish_initialpose"]:
            result["initialpose_subscriber_count"] = node.subscriber_count()
    finally:
        result.update(summarize(node.infos, node.poses))
        result["infos"] = node.infos[-120:]
        result["poses"] = node.poses[-120:]
        with open(args["output"], "w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=1)
            handle.write(chr(10))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())