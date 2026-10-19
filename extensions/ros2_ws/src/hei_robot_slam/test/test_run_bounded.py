"""run_bounded 回归：SIGINT 转发、有界升级、残留检测、子节点死亡检测。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from hei_robot_slam import run_bounded


def _script(tmp_path: Path, name: str, body: str) -> str:
    path = tmp_path / name
    path.write_text(body, encoding="utf-8")
    return str(path)


def test_clean_child_exits_on_single_sigint(tmp_path: Path):
    child = _script(
        tmp_path,
        "clean.py",
        "import signal, sys, time\n"
        "signal.signal(signal.SIGINT, lambda *args: sys.exit(0))\n"
        "time.sleep(30)\n",
    )
    result = run_bounded.run_bounded(
        [sys.executable, child],
        log_path=tmp_path / "clean.log",
        result_path=tmp_path / "clean.json",
        duration_s=0.4,
        grace_s=2.0,
        poll_interval_s=0.05,
    )
    assert result["exit_code"] == 0
    assert result["sigint_sent"] is True
    assert result["escalations"] == []
    assert result["residual_after_exit"] == []
    assert result["child_failures"] == []
    assert result["clean"] is True
    assert json.loads((tmp_path / "clean.json").read_text(encoding="utf-8"))["clean"] is True


def test_stubborn_child_gets_bounded_escalation(tmp_path: Path):
    child = _script(
        tmp_path,
        "stubborn.py",
        "import signal, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "time.sleep(60)\n",
    )
    result = run_bounded.run_bounded(
        [sys.executable, child],
        log_path=tmp_path / "stubborn.log",
        duration_s=0.3,
        grace_s=0.6,
        term_grace_s=0.6,
        poll_interval_s=0.05,
    )
    assert result["escalations"], "无视 SIGINT 的子进程必须触发有界升级记录"
    assert result["escalations"][0]["signal"] == "SIGTERM"
    assert result["clean"] is False
    assert result["residual_after_cleanup"] == []


def test_orphan_child_after_parent_exit_is_not_clean(tmp_path: Path):
    sleeper = _script(tmp_path, "sleeper.py", "import time\ntime.sleep(60)\n")
    parent = _script(
        tmp_path,
        "orphan_parent.py",
        "import subprocess, sys\n"
        "subprocess.Popen([sys.executable, sys.argv[1]])\n"
        "sys.exit(0)\n",
    )
    result = run_bounded.run_bounded(
        [sys.executable, parent, sleeper],
        log_path=tmp_path / "orphan.log",
        duration_s=None,
        grace_s=2.0,
        poll_interval_s=0.05,
    )
    assert result["exit_code"] == 0
    assert result["residual_after_exit"], "父进程 exit 0 但遗留子进程必须被检测到"
    assert result["residual_after_cleanup"] == []
    assert result["clean"] is False


def test_child_death_in_log_marks_run_unclean(tmp_path: Path):
    result = run_bounded.run_bounded(
        ["bash", "-c", "echo process has died [pid 123, exit code 1]; exit 0"],
        log_path=tmp_path / "died.log",
        duration_s=None,
        grace_s=2.0,
        poll_interval_s=0.05,
    )
    assert result["exit_code"] == 0
    assert result["child_failures"], "日志出现 process has died 时不得报告 clean"
    assert result["clean"] is False


def test_result_contains_log_fingerprint(tmp_path: Path):
    result = run_bounded.run_bounded(
        ["bash", "-c", "echo hello; exit 0"],
        log_path=tmp_path / "fingerprint.log",
        duration_s=None,
        grace_s=2.0,
        poll_interval_s=0.05,
    )
    assert result["log_size_bytes"] > 0
    assert len(result["log_sha256"]) == 64
    assert result["clean"] is True
