"""构建脚本的回归护栏（静态检查）。

脚本是 bash，无法在这里整体单测；这里固定住 RF-LIDAR-002 要求的三条不变量：
  1. 必须拒绝已跟踪源码的本地修改（否则构建结果不对应冻结 SHA）；
  2. 不得无条件删除预存的 driver `launch/` 目录；
  3. `ldd` 自身失败必须报错，不能吞掉。
"""

from __future__ import annotations

import re
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_REL = Path("extensions") / "tools" / "build_mid360s_underlay.sh"


def _find_repo_root(start: Path) -> Path:
    """向上找到含构建脚本的仓库根目录（不写死层级，避免目录调整后失效）。"""
    for candidate in (start, *start.parents):
        if (candidate / SCRIPT_REL).is_file():
            return candidate
    raise AssertionError(f"从 {start} 向上找不到 {SCRIPT_REL}")


REPO_ROOT = _find_repo_root(PKG_ROOT)
SCRIPT = REPO_ROOT / SCRIPT_REL


def _source() -> str:
    assert SCRIPT.is_file(), f"缺少构建脚本 {SCRIPT}"
    return SCRIPT.read_text(encoding="utf-8")


def test_script_checks_tracked_dirty_state():
    src = _source()
    assert "--untracked-files=no" in src, "必须只按“已跟踪改动”判定脏工作区"
    assert "存在已跟踪文件的本地修改" in src, "必须有明确的拒绝信息"
    # 允许被忽略的生成物（package.xml / launch/ / build/）存在而不误判
    assert "ignored" in src or "忽略" in src


def test_script_does_not_unconditionally_delete_launch_dir():
    src = _source()
    # 出现 rm -rf "<driver>/launch" 是允许的，但必须受 CREATED_LAUNCH 保护
    assert "CREATED_LAUNCH" in src, "必须记录 launch/ 是否由本脚本创建"
    cleanup = src[src.index("cleanup_driver()") :]
    assert 'if [ "${CREATED_LAUNCH}" = "1" ]' in cleanup, "清理必须仅针对本脚本创建的目录"
    # cp 之前不得有无条件的 rm -rf
    prepare = src[: src.index("cp -rf") if "cp -rf" in src else len(src)]
    assert not re.search(r'^\s*rm -rf\s+"\$\{DRIVER_DIR\}/launch"', prepare, re.M), (
        "准备阶段不得无条件删除已有 launch/"
    )


def test_script_refuses_preexisting_launch_or_conflicting_manifest():
    src = _source()
    assert "拒绝删除或覆盖" in src
    assert "cmp -s" in src, "package.xml 已存在时必须先比对内容再决定是否覆盖"


def test_ldd_failure_is_not_swallowed():
    src = _source()
    assert "ldd 执行失败" in src, "ldd 自身失败必须报错"
    assert not re.search(r'ldd\s+"\$\{f\}"[^\n]*\|\|\s*true', src), "不得用 || true 吞掉 ldd 失败"
