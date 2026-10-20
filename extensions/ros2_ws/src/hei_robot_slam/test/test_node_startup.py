"""真实节点启动回归：防止 Path 命名遮蔽等只在运行时暴露的崩溃。

本轮教训：cloud_probe.py 里 `from nav_msgs.msg import Path` 覆盖了 pathlib.Path，
导致 probe 一启动就 TypeError 崩溃；launch 父进程仍 exit 0，因此单看 launch exit code
不够。本测试直接启动 console_script 可执行文件并核对真能驻留 + SIGINT 干净退出。
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
#: 现场会话使用 Domain 89；本任务所有 ROS 测试固定默认 Domain 91（可显式覆盖，但拒绝 89）。
DEFAULT_TEST_DOMAIN_ID = "91"
RESERVED_LIVE_DOMAIN_ID = "89"


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
    return str(Path(prefix) / "lib" / PKG / name)


def _stop_single_process(proc: subprocess.Popen, grace_s: float = 8.0) -> None:
    """只对节点进程本身发一次 SIGINT（不 killpg，避免二次信号）。"""
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
            proc.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=4.0)


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_cloud_probe_node_starts_stays_up_and_stops_cleanly(tmp_path: Path) -> None:
    executable = _package_executable("slam_cloud_probe")
    assert Path(executable).is_file(), f"找不到节点可执行文件 {executable}"
    report = tmp_path / "probe_report.json"
    env = dict(os.environ, ROS_DOMAIN_ID=_test_domain_id(), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(
        [
            executable,
            "--ros-args",
            "-p",
            f"report_path:={report}",
            "-p",
            "run_tag:=pytest-node-startup",
            "-p",
            "use_sim_time:=false",
            "-p",
            "report_period_s:=1.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    try:
        time.sleep(4.0)
        alive = proc.poll() is None
        output = "" if alive else (proc.stdout.read() if proc.stdout else "")
    finally:
        _stop_single_process(proc)

    assert alive, "cloud_probe 节点启动后应立即崩溃（本轮 Path 遮蔽 bug 的回归）：\n" + output
    assert proc.returncode == 0, f"节点未能干净退出，returncode={proc.returncode}"
    assert report.is_file(), "节点未写出 probe 报告"
    payload = json.loads(report.read_text(encoding="utf-8"))
    assert payload["kind"] == "mid360s_slam_cloud_probe"
    assert payload["final"] is True
    observability = payload.get("localization_observability")
    assert isinstance(observability, dict), "probe=true 应自动附带定位取证 summary"
    assert observability["schema"] == "hei.localization_summary.v1"
    assert observability["final"] is True
    assert observability["event_status"]["status"] == "active"
    assert observability["writer"]["closed"] is True, "final report 必须反映 JSONL 已关闭"
    events_path = Path(observability["writer"]["jsonl_path"])
    summary_path = Path(observability["writer"]["summary_path"])
    assert events_path.is_file() and summary_path.is_file()
    assert events_path.parent == report.parent
    assert observability["pending_observability"]["pending_coverage"] == "rosout_only_partial"
    lines = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    assert lines == [] or all(item["schema"] == "hei.localization_event.v1" for item in lines)
    final_summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert final_summary["final"] is True
    assert final_summary["writer"]["closed"] is True, "final summary 必须反映 close 后的 writer 状态"
    assert final_summary["writer"]["write_errors"] == 0


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_cloud_probe_events_disabled_status_is_explicit(tmp_path: Path) -> None:
    executable = _package_executable("slam_cloud_probe")
    report = tmp_path / "probe_report.json"
    env = dict(os.environ, ROS_DOMAIN_ID=_test_domain_id(), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(
        [
            executable,
            "--ros-args",
            "-p",
            f"report_path:={report}",
            "-p",
            "localization_events_enabled:=false",
            "-p",
            "use_sim_time:=false",
            "-p",
            "report_period_s:=1.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    try:
        time.sleep(3.0)
        alive = proc.poll() is None
        output = "" if alive else (proc.stdout.read() if proc.stdout else "")
    finally:
        _stop_single_process(proc)
    assert alive, "probe 启动失败：" + output
    assert proc.returncode == 0
    payload = json.loads(report.read_text(encoding="utf-8"))
    observability = payload["localization_observability"]
    assert observability["enabled"] is False
    assert observability["status"] == "disabled_by_parameter"
    assert observability["writer"] is None
    assert not list(report.parent.glob("*.localization_events.jsonl"))


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_cloud_adapter_node_starts_and_stops_cleanly(tmp_path: Path) -> None:
    executable = _package_executable("slam_cloud_adapter")
    env = dict(os.environ, ROS_DOMAIN_ID=_test_domain_id(), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(
        [executable, "--ros-args", "-p", "use_sim_time:=false", "-p", "report_period_s:=1.0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    try:
        time.sleep(3.0)
        alive = proc.poll() is None
        output = "" if alive else (proc.stdout.read() if proc.stdout else "")
    finally:
        _stop_single_process(proc)
    assert alive, "slam_cloud_adapter 启动后崩溃：\n" + output
    assert proc.returncode == 0, f"adapter 未能干净退出，returncode={proc.returncode}"