"""真实节点启动冒烟：lio_input_audit 必须能起来、驻留、并能被正常停止。

这条用 ros2 run 真正启动节点，专门防“构造阶段 AttributeError / API 名写错”这类
只在运行时才暴露的问题。

进程生命周期规则（可移植性关键）：
  * 用 Popen(start_new_session=True) 让 ros2 run 及其子节点进入**独立进程组**；
  * 停止时对**整个进程组**发 SIGINT，再 wait；超时升级 TERM，再超时 KILL；
  * 绝不能用 subprocess.run(timeout=) 了事：它只杀 ros2 包装进程，会留下
    lio_input_audit 孤儿进程混进 Domain 88 的离线回放。
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time

import pytest

PKG = "hei_robot_localization"
EXE = "lio_input_audit"
LIVE_SECONDS = 4.0
STOP_GRACE_S = 8.0


def _ros_ready() -> bool:
    return os.environ.get("ROS_DISTRO") is not None and shutil.which("ros2") is not None


def _env() -> dict:
    return dict(os.environ, ROS_DOMAIN_ID="88", ROS_LOCALHOST_ONLY="1")


def _stop_group(proc: subprocess.Popen) -> None:
    """保证整个进程组退出：SIGINT -> TERM -> KILL，每一步都有界等待。"""
    if proc.poll() is not None:
        return
    for sig, grace in ((signal.SIGINT, STOP_GRACE_S), (signal.SIGTERM, 4.0), (signal.SIGKILL, 4.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            continue


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_node_starts_keeps_running_and_stops_cleanly():
    cmd = ["ros2", "run", PKG, EXE, "--ros-args", "-p", "window_seconds:=1.0"]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=_env(),
    )
    try:
        time.sleep(LIVE_SECONDS)
        alive = proc.poll() is None
        output = ""
        if not alive:
            output = proc.stdout.read() if proc.stdout else ""
    finally:
        _stop_group(proc)
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:  # pragma: no cover - 兜底
            proc.kill()
            proc.wait(timeout=2.0)

    if not alive:
        raise AssertionError(
            "节点应在无数据时持续运行，实际提前退出（exit="
            + str(proc.returncode)
            + "）：\n"
            + output
        )
    assert proc.returncode is not None, "停止后必须能 wait 到退出码，不能留孤儿"
    # SIGINT 正常关停应当不是被 SIGKILL 打死的
    assert proc.returncode >= 0, (
        "节点被信号强杀（returncode="
        + str(proc.returncode)
        + "），说明关闭路径不响应 SIGINT"
    )


@pytest.mark.skipif(not _ros_ready(), reason="需要已 source 的 ROS 2 环境")
def test_no_residual_audit_nodes_after_stop():
    """回归：停止后不得留下 lio_input_audit 进程。"""
    time.sleep(1.0)
    out = subprocess.run(
        ["pgrep", "-af", "lio_input_audit"],
        capture_output=True,
        text=True,
        check=False,
    )
    # pgrep 的 -a 会匹配到本测试自身的命令行文本，这里只接受空或仅自身
    lines = [ln for ln in out.stdout.splitlines() if "pytest" not in ln and "pgrep" not in ln]
    assert lines == [], "停止后仍有 lio_input_audit 残留：\n" + "\n".join(lines)
