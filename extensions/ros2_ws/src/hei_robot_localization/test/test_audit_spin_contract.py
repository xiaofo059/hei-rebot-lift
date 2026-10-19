"""spin 循环的异常契约（纯离线，不需要 ROS context）。

要求：运行期异常必须传播，不能被吞成“正常退出”；只有 context 已经关停
（SIGINT 关停窗口）时允许降级，且必须留下警告。
"""

from __future__ import annotations

import pytest

from hei_robot_localization.lio_input_audit import spin_loop


class _WarnCollector:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)


class _FakeNode:
    def __init__(self) -> None:
        self.logger = _WarnCollector()

    def get_logger(self) -> _WarnCollector:
        return self.logger


def test_runtime_exception_propagates_while_context_alive():
    node = _FakeNode()

    def spin_once(_node):
        raise RuntimeError("take_message failed")

    with pytest.raises(RuntimeError, match="take_message failed"):
        spin_loop(node, spin_once, lambda: True)
    assert node.logger.warnings == [], "运行期异常不得被降级成警告"


def test_exception_during_shutdown_degrades_with_warning():
    node = _FakeNode()
    state = {"ok": True}

    def spin_once(_node):
        state["ok"] = False  # 模拟 context 在取消息前被关停
        raise RuntimeError("Unable to convert call argument to Python object")

    spin_loop(node, spin_once, lambda: state["ok"])
    assert len(node.logger.warnings) == 1
    assert "Unable to convert" in node.logger.warnings[0]


def test_keyboard_interrupt_is_not_swallowed():
    node = _FakeNode()

    def spin_once(_node):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        spin_loop(node, spin_once, lambda: True)


def test_loop_exits_when_context_not_ok():
    node = _FakeNode()
    calls = {"n": 0}

    def spin_once(_node):
        calls["n"] += 1

    spin_loop(node, spin_once, lambda: False)
    assert calls["n"] == 0
