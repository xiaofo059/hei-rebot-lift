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
    env = dict(os.environ, ROS_DOMAIN_ID="89", ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(
        [
            executable,
            "--ros-args",
            "-p",
            f"report_path:={report}",
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


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_cloud_adapter_node_starts_and_stops_cleanly(tmp_path: Path) -> None:
    executable = _package_executable("slam_cloud_adapter")
    env = dict(os.environ, ROS_DOMAIN_ID="89", ROS_LOCALHOST_ONLY="1")
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
