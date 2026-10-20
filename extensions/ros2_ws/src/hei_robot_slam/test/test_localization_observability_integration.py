"""Domain 91 合成端到端：probe 定位连续取证（synthetic，绝不启动真实算法/驱动）。

本测试只启动：
* 被测的 slam_cloud_probe 节点（只读订阅）；
* 一个合成发布器节点（Info/initialpose/TF/rosout/localization_pose + 后端日志文件追加）。
所有数据均为 synthetic，不代表真实定位；严禁使用 Domain 89（现场会话）。
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

PKG = "hei_robot_slam"
DEFAULT_TEST_DOMAIN_ID = "91"
RESERVED_LIVE_DOMAIN_ID = "89"

PUBLISHER_SCRIPT = r'''
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info
from tf2_ros import TransformBroadcaster

BACKEND_LOG = sys.argv[1]


def pub_qos(depth=20):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_info(node, proximity_id=0, loop_id=0, small=0.0, rejected=0.0, icp=0.0, opt_iter=1.0):
    msg = Info()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.ref_id = 12
    msg.proximity_detection_id = int(proximity_id)
    msg.loop_closure_id = int(loop_id)
    msg.landmark_id = 0
    msg.wm_state = [1, 2, 3]
    msg.stats_keys = [
        "Memory/Small_movement/",
        "Memory/Fast_movement/",
        "Loop/RejectedHypothesis/",
        "Proximity/Space_detections_added_icp_multi/",
        "Loop/Accepted_hypothesis_id/",
        "Loop/Optimization_iterations/",
    ]
    msg.stats_values = [
        float(small),
        0.0,
        float(rejected),
        float(icp),
        5.0 if proximity_id else 0.0,
        float(opt_iter),
    ]
    return msg


def main():
    rclpy.init()
    node = Node("synthetic_localization_publisher")
    initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", pub_qos(10))
    info_pub = node.create_publisher(Info, "/info", pub_qos(20))
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/localization_pose", pub_qos(20))
    rosout_pub = node.create_publisher(Log, "/rosout", pub_qos(200))
    tf_broadcaster = TransformBroadcaster(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

    def publish_tf(x):
        from geometry_msgs.msg import TransformStamped

        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_init"
        transform.transform.translation.x = float(x)
        transform.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(transform)

    def publish_rosout(text, level=30):
        msg = Log()
        msg.stamp = node.get_clock().now().to_msg()
        msg.level = int(level)
        msg.name = "rtabmap"
        msg.msg = text
        rosout_pub.publish(msg)

    def publish_initialpose(x=0.1, y=0.2):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        initialpose_pub.publish(msg)

    def publish_pose():
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.3
        msg.pose.pose.position.y = 0.2
        msg.pose.pose.orientation.w = 1.0
        pose_pub.publish(msg)

    def append_backend(lines):
        with open(BACKEND_LOG, "a", encoding="utf-8") as handle:
            for line in lines:
                handle.write(line + "\n")
            handle.flush()

    spin(1.0)
    publish_initialpose()
    publish_rosout("initialpose received: xyz=0.100000,0.200000,0.000000 rpy=0.0,0.0,0.0", 20)
    publish_rosout(
        "\x1b[33m[ WARN] (2026-10-20 14:56:17.274) Rtabmap.cpp:3911::process() "
        "Localization was good, but waiting for another one to be more accurate "
        "(RGBD/MaxOdomCacheSize>0)\x1b[0m",
        30,
    )
    spin(0.6)
    append_backend(
        [
            "SYNTHETIC pending Localization was good, but waiting for another one",
            "\x1b[32m[ INFO] (2026-10-20 15:12:35.308) Rtabmap.cpp:3076::process() "
            "[Scan matching] Add local loop closure in SPACE (40->1) xyz=0.1,0.2,0.0\x1b[0m",
        ]
    )
    spin(0.8)
    # 截断：写更短内容
    with open(BACKEND_LOG, "w", encoding="utf-8") as handle:
        handle.write("SYNTHETIC after truncation Localization was good waiting\n")
    spin(0.8)
    # 轮转：原子替换成新 inode
    replacement = BACKEND_LOG + ".new"
    with open(replacement, "w", encoding="utf-8") as handle:
        handle.write(
            "SYNTHETIC map correction (2026-10-20 15:12:36.000) "
            "Rtabmap.cpp:1376::process() Update map correction based on last localization "
            "saved in database! correction = xyz=0,0,0 rpy=0,0,0, nearest id = 5\n"
        )
    os.replace(replacement, BACKEND_LOG)
    spin(0.8)
    publish_tf(0.1)
    spin(0.15)
    publish_tf(0.1)
    spin(0.15)
    publish_tf(0.1)
    spin(0.15)
    publish_tf(0.3)  # changed -> TF change event
    spin(0.6)
    info_pub.publish(make_info(node, proximity_id=29, icp=1.0))
    spin(0.6)
    info_pub.publish(make_info(node, small=1.0, rejected=0.0))
    spin(0.6)
    publish_pose()
    spin(0.4)
    info_pub.publish(make_info(node, loop_id=7))
    spin(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
'''


def _test_domain_id() -> str:
    value = str(os.environ.get("HEI_TEST_ROS_DOMAIN_ID", DEFAULT_TEST_DOMAIN_ID)).strip()
    if value == RESERVED_LIVE_DOMAIN_ID:
        pytest.fail(
            "ROS_DOMAIN_ID=89 属于现场定位会话；本任务 ROS 测试禁止使用 89，"
            f"请使用 HEI_TEST_ROS_DOMAIN_ID={DEFAULT_TEST_DOMAIN_ID}"
        )
    return value or DEFAULT_TEST_DOMAIN_ID


def _ros_ready() -> bool:
    return os.environ.get("ROS_DISTRO") is not None and shutil.which("ros2") is not None


def _package_executable(name: str) -> str:
    prefix = subprocess.run(
        ["ros2", "pkg", "prefix", PKG], capture_output=True, text=True, check=True
    ).stdout.strip()
    path = Path(prefix) / "lib" / PKG / name
    assert path.is_file(), f"找不到已安装节点：{path}"
    return str(path)


def _stop_single_process(proc: subprocess.Popen, grace_s: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    try:
        os.kill(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.1) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return False


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_probe_localization_observability_synthetic_end_to_end(tmp_path: Path) -> None:
    domain = _test_domain_id()
    run_dir = tmp_path / "run"
    run_dir.mkdir(parents=True)
    report_path = run_dir / "probe_report.json"
    backend_log = run_dir / "backend_localization.log"
    backend_log.write_text("OLD line before probe start Localization was good waiting\n", encoding="utf-8")
    publisher_script = tmp_path / "synthetic_publisher.py"
    publisher_script.write_text(PUBLISHER_SCRIPT, encoding="utf-8")

    env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY="1")
    probe = subprocess.Popen(
        [
            _package_executable("slam_cloud_probe"),
            "--ros-args",
            "-p",
            f"report_path:={report_path}",
            "-p",
            "run_tag:=synthetic-domain91",
            "-p",
            "use_sim_time:=false",
            "-p",
            "report_period_s:=1.0",
            "-p",
            f"backend_log_path:={backend_log}",
            "-p",
            "backend_log_poll_s:=0.2",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    publisher = None
    try:
        assert _wait_until(lambda: report_path.parent.is_dir() and list(run_dir.glob("*.jsonl")), 10.0), (
            "probe 未创建 JSONL 时间线；probe 输出："
            + (probe.stdout.read() if probe.poll() is not None and probe.stdout else "")
        )
        time.sleep(1.0)  # 等 DDS 发现
        publisher = subprocess.Popen(
            ["/usr/bin/python3", str(publisher_script), str(backend_log)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )
        publisher_rc = publisher.wait(timeout=60)
        assert publisher_rc == 0, "合成发布器失败：\n" + (publisher.stdout.read() if publisher.stdout else "")
        time.sleep(2.0)  # 等 probe 消费/落盘
    finally:
        _stop_single_process(probe)
        if publisher is not None and publisher.poll() is None:
            publisher.kill()
            publisher.wait(timeout=5)

    assert probe.returncode == 0, f"probe 未干净退出：{probe.returncode}"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["kind"] == "mid360s_slam_cloud_probe"
    assert report["final"] is True
    observability = report.get("localization_observability")
    assert isinstance(observability, dict), "旧报告兼容字段缺失 localization_observability"
    writer = observability["writer"]
    assert writer["closed"] is True
    assert writer["events_written"] > 0
    assert writer["write_errors"] == 0
    events_path = Path(writer["jsonl_path"])
    summary_path = Path(writer["summary_path"])
    assert events_path.is_file() and summary_path.is_file()
    assert events_path.parent == report_path.parent

    events = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    assert len(events) == writer["events_written"]
    assert all(item["schema"] == "hei.localization_event.v1" for item in events)
    assert all(item["seq"] == index + 1 for index, item in enumerate(events))
    kinds = [item["event"] for item in events]
    assert kinds.count("info") == 3
    assert kinds.count("initialpose") >= 1
    assert kinds.count("tf_map_to_camera_init") == 2  # 首值 + 一次变化（重复值不落盘）
    assert kinds.count("localization_pose") >= 1
    assert kinds.count("rosout") >= 2
    backend_events = [item for item in events if item["event"] == "backend_log"]
    assert backend_events, "backend_log tail 事件缺失"
    backend_categories = [item["data"].get("category") for item in backend_events]
    assert "localization_pending_confirmation" in backend_categories
    assert "candidate_link_added" in backend_categories
    assert "backend_log_truncation" in backend_categories
    assert "backend_log_rotation" in backend_categories
    assert "map_correction_restore" in backend_categories
    # 启动前的旧日志行不能被当作本次事件
    for item in backend_events:
        assert "OLD line before probe start" not in json.dumps(item, ensure_ascii=False)

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["schema"] == "hei.localization_summary.v1"
    assert summary["final"] is True
    assert summary["writer"]["closed"] is True, "final summary 必须反映 close 后的 writer 状态"
    assert summary["event_status"]["status"] == "active"
    assert observability["event_status"]["status"] == "active"
    assert summary["run_tag"] == "synthetic-domain91"
    cumulative = summary["cumulative"]
    assert cumulative["info_events"] == 3
    assert cumulative["backend_accepted_events"] == 2  # 只有 /info 非零 proximity/loop id 计 accepted
    assert cumulative["info_frames_with_accepted_ids"] == 2
    assert cumulative["pending_confirmation_events"] >= 2
    assert cumulative["pending_by_source"]["rosout"] >= 1
    assert cumulative["pending_by_source"]["backend_log"] >= 1
    assert cumulative["candidate_link_added_events"] == 1
    assert cumulative["initialpose_events"] == 1
    assert cumulative["initialpose_received_log_events"] >= 1
    assert cumulative["map_correction_restore_events"] >= 1
    assert cumulative["backend_log_truncation_events"] >= 1
    assert cumulative["backend_log_rotation_events"] >= 1
    assert cumulative["tf_updates"] == 4
    assert cumulative["tf_change_events"] == 2
    derived = summary["derived"]
    assert derived["historical_backend_accepted_total"] == 2
    assert derived["no_current_event"]["accepted_in_current_frame"] is True
    assert derived["state_hint"] == "accepted_ids_in_current_frame"
    assert derived["tf_change_is_not_acceptance"] is True
    assert derived["candidate_link_added_total"] == 1
    assert summary["pending_observability"]["pending_coverage"] == "rosout_plus_backend_log_tail"
    assert summary["pending_observability"]["complete"] is False
    accepted = [item for item in events if item["data"].get("derived", {}).get("accepted_any")]
    assert len(accepted) == 2
    for item in accepted:
        correlation = item["data"]["derived"].get("ref_id")
        assert correlation == 12
    summary_accepted = summary["last"].get("accepted_event")
    assert summary_accepted is not None
    assert summary_accepted["correlated_initial_pose"] is not None
    assert summary_accepted["correlated_initial_pose"]["association"] == "time_only_not_causal"
    json.dumps(summary, allow_nan=False)
    json.dumps(report, allow_nan=False)
