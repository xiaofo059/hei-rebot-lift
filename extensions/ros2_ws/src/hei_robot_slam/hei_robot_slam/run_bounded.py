"""有界启动/回收 harness（用于 ROS 2 launch 与离线回放取证）。

为什么需要它（本轮已踩过的坑）：

* 后台 shell（``&`` / ``setsid``）会把 SIGINT/SIGQUIT 继承为 **SIG_IGN**；
  此时对 launch 父进程发 SIGINT 是空操作，子节点永远收不到正常关停信号。
  修复：``Popen(..., preexec_fn=...)`` 中把 SIGINT/SIGQUIT 显式恢复为 SIG_DFL。
* 正确停止方式是**只对 launch 父进程发一次 SIGINT**，由 launch 自己转发给子节点；
  不要对进程组重复发 INT（会造成二次信号/竞争）。
* 有界升级：等待 ``grace_s``（默认 12s）仍未退出 → 只对父进程 SIGTERM；
  再等 ``term_grace_s`` → SIGKILL；每步都记录，不假装 clean exit。
* 退出后按**进程组**核对残留（start_new_session 保证 pgid 可追踪），
  有残留则逐个 SIGINT→SIGKILL 并在 JSON 中如实记录。

用法（作为 console script）：

    slam_run_bounded --log LOG --result RESULT.json [--duration-s N] \
        -- ros2 launch hei_robot_slam slam_offline_mapping.launch.py uncalibrated:=true ...
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

STOP_SIGNAL_NAME = "SIGINT"

#: launch 日志中表示子节点异常终止的模式：父 launch exit 0 不代表子节点都正常，
#: 必须显式扫描日志（本轮已遇到 probe 崩溃但 launch 父进程仍退出 0 的案例）。
_CHILD_FAILURE_PATTERNS = (
    re.compile(r"process has died"),
    re.compile(r"process has exited with code [-1-9]"),
    re.compile(r"exit code [1-9]\d*"),
);


def _reset_ignored_signals() -> None:  # pragma: no cover - 子进程侧执行
    """Popen preexec：把被后台 shell 继承成 SIG_IGN 的 INT/QUIT 恢复为默认。"""
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGQUIT, signal.SIG_DFL)


def _pgid_members(pgid: int) -> list[dict[str, Any]]:
    """列出当前进程组成员（pid/cmdline），用于退出后残留核对。"""
    members: list[dict[str, Any]] = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == os.getpid():
            continue
        try:
            if os.getpgid(pid) != pgid:
                continue
            raw = Path(f"/proc/{pid}/cmdline").read_bytes()
            cmd = raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()
            members.append({"pid": pid, "cmd": cmd})
        except (ProcessLookupError, PermissionError, FileNotFoundError, OSError):
            continue
    return sorted(members, key=lambda item: item["pid"])


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_kill(pid: int, sig: int) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass
    except PermissionError:
        pass


def run_bounded(
    command: list[str],
    log_path: str | os.PathLike[str],
    result_path: str | os.PathLike[str] | None = None,
    duration_s: float | None = None,
    grace_s: float = 12.0,
    term_grace_s: float = 5.0,
    kill_grace_s: float = 2.0,
    stop_file: str | os.PathLike[str] | None = None,
    tag: str = "",
    poll_interval_s: float = 0.1,
) -> dict[str, Any]:
    """启动命令，等待 duration_s 后只对父进程发 SIGINT，有界升级并记录一切。"""
    if not command:
        raise ValueError("command 不能为空")
    log = Path(log_path).expanduser()
    log.parent.mkdir(parents=True, exist_ok=True)

    # 墙钟只用于 ISO 记录；所有 deadline/超时判定用单调时钟，避免 NTP 跳变破坏有界退出。
    started_wall = time.time()
    started_mono = time.monotonic()
    started_iso = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(started_wall))
    log_handle = open(log, "wb")
    try:
        process = subprocess.Popen(
            command,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=_reset_ignored_signals,
            start_new_session=True,
        )
    finally:
        log_handle.close()

    pgid = process.pid
    result: dict[str, Any] = {
        "kind": "bounded_launch_run",
        "tag": tag,
        "command": command,
        "log": str(log),
        "started_at": started_iso,
        "pid": process.pid,
        "pgid": pgid,
        "duration_limit_s": duration_s,
        "grace_s": grace_s,
        "term_grace_s": term_grace_s,
        "sigint_sent": False,
        "sigint_at": None,
        "escalations": [],
        "stop_reason": None,
        "residual_after_exit": [],
        "residual_cleanup": [],
        "residual_after_cleanup": [],
    }

    stop_requested_at: float | None = None
    stop_stage = "running"
    external_stop = {"requested": False}

    def _on_sigint(_signum: int, _frame: Any) -> None:
        external_stop["requested"] = True

    previous_handler = signal.signal(signal.SIGINT, _on_sigint)
    try:
        while True:
            returncode = process.poll()
            if returncode is not None:
                break
            now = time.monotonic()
            if stop_requested_at is None:
                triggered = False
                if duration_s is not None and (now - started_mono) >= duration_s:
                    triggered = True
                    result["stop_reason"] = "duration_elapsed"
                elif external_stop["requested"]:
                    triggered = True
                    result["stop_reason"] = "operator_sigint"
                elif stop_file is not None and Path(stop_file).exists():
                    triggered = True
                    result["stop_reason"] = "stop_file"
                if triggered:
                    _safe_kill(process.pid, signal.SIGINT)
                    stop_requested_at = now
                    stop_stage = "wait_sigint"
                    result["sigint_sent"] = True
                    result["sigint_at"] = round(now - started_mono, 3)
            else:
                elapsed = now - stop_requested_at
                if stop_stage == "wait_sigint" and elapsed > grace_s:
                    _safe_kill(process.pid, signal.SIGTERM)
                    stop_stage = "wait_term"
                    result["escalations"].append(
                        {"signal": "SIGTERM", "after_s": round(elapsed, 3), "reason": "SIGINT grace timeout"}
                    )
                elif stop_stage == "wait_term" and elapsed > (grace_s + term_grace_s):
                    _safe_kill(process.pid, signal.SIGKILL)
                    stop_stage = "wait_kill"
                    result["escalations"].append(
                        {"signal": "SIGKILL", "after_s": round(elapsed, 3), "reason": "SIGTERM grace timeout"}
                    )
                elif stop_stage == "wait_kill" and elapsed > (grace_s + term_grace_s + kill_grace_s):
                    result["stop_reason"] = (result["stop_reason"] or "") + " (kill grace exceeded)"
                    break
            time.sleep(poll_interval_s)

        try:
            returncode = process.wait(timeout=max(1.0, kill_grace_s))
        except subprocess.TimeoutExpired:  # pragma: no cover - 兜底
            _safe_kill(process.pid, signal.SIGKILL)
            returncode = process.wait(timeout=5.0)
    finally:
        signal.signal(signal.SIGINT, previous_handler)

    finished_wall = time.time()
    finished_mono = time.monotonic()
    result.update(
        {
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(finished_wall)),
            "wall_duration_s": round(finished_mono - started_mono, 3),
            "exit_code": returncode,
            "exited_by_signal": returncode is not None and returncode < 0,
            "term_stage": stop_stage,
        }
    )

    residual = _pgid_members(pgid)
    result["residual_after_exit"] = residual
    cleanup: list[dict[str, Any]] = []
    if residual:
        for item in residual:
            _safe_kill(item["pid"], signal.SIGINT)
            cleanup.append({"pid": item["pid"], "signal": "SIGINT"})
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and _pgid_members(pgid):
            time.sleep(0.1)
        for item in _pgid_members(pgid):
            _safe_kill(item["pid"], signal.SIGKILL)
            cleanup.append({"pid": item["pid"], "signal": "SIGKILL"})
        time.sleep(0.2)
    result["residual_cleanup"] = cleanup
    result["residual_after_cleanup"] = _pgid_members(pgid)
    result.setdefault("child_failures", [])

    child_failures: list[str] = []
    if log.is_file():
        result["log_size_bytes"] = log.stat().st_size
        result["log_sha256"] = _sha256_file(log)
        try:
            lines = log.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            lines = []
        for line in lines:
            if any(pattern.search(line) for pattern in _CHILD_FAILURE_PATTERNS):
                child_failures.append(line)
        result["log_tail"] = lines[-30:]
    result["child_failures"] = child_failures
    # clean 判定必须同时满足：父进程自然退出 0、无升级、且退出时没有任何进程组残留；
    # 事后杀掉孤儿不算 clean（否则会掩盖 launch 退出路径的真实缺陷）。
    result["clean"] = (
        result["exit_code"] == 0
        and not result["escalations"]
        and not result["residual_after_exit"]
        and not result["residual_cleanup"]
        and not result["residual_after_cleanup"]
        and not child_failures
    )

    if result_path is not None:
        target = Path(result_path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="有界启动/停止 ROS 2 launch 并留存取证 JSON")
    parser.add_argument("--log", required=True, help="子进程 stdout/stderr 日志路径")
    parser.add_argument("--result", required=True, help="取证结果 JSON 路径")
    parser.add_argument("--duration-s", type=float, default=None, help="运行 N 秒后发 SIGINT")
    parser.add_argument("--grace-s", type=float, default=12.0, help="SIGINT 有界等待秒数（默认 12）")
    parser.add_argument("--term-grace-s", type=float, default=5.0, help="SIGTERM 有界等待秒数")
    parser.add_argument("--kill-grace-s", type=float, default=2.0, help="SIGKILL 后等待秒数")
    parser.add_argument("--stop-file", default=None, help="该文件出现时触发一次 SIGINT")
    parser.add_argument("--tag", default="", help="取证标签")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="-- 之后的完整命令")
    args = parser.parse_args(argv)

    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("缺少要启动的命令（放在 -- 之后）")

    result = run_bounded(
        command,
        log_path=args.log,
        result_path=args.result,
        duration_s=args.duration_s,
        grace_s=args.grace_s,
        term_grace_s=args.term_grace_s,
        kill_grace_s=args.kill_grace_s,
        stop_file=args.stop_file,
        tag=args.tag,
    )
    print(json.dumps({key: result[key] for key in (
        "tag", "exit_code", "sigint_sent", "escalations", "clean", "wall_duration_s", "result",
    ) if key in result}, ensure_ascii=False, indent=1))
    for line in result.get("log_tail", []):
        print(line)
    return 0 if result["clean"] else 1


if __name__ == "__main__":
    sys.exit(main())