"""构建脚本 patch 状态机的行为回归。

为什么需要它：extensions/tools/build_fast_lio_underlay.sh 的“工作区状态机”决定
第三方源码能否被构建。静态检查无法证明它真的会拒绝未知 dirty、也证明不了拒绝时
不修改工作区。本测试在**临时 git 仓库**里驱动脚本的库函数（HEI_FASTLIO_LIB_ONLY=1
钩子），完全不触碰真实的第三方源码。

固定住四条不变量：
  1. 干净 SHA 能被识别，patch 能顺序应用；
  2. patch 之外任何已跟踪改动都被拒绝，且拒绝前后工作区逐字节不变；
  3. patch 之外任何未跟踪文件都被拒绝；
  4. 已应用状态可逆：撤销后回到干净 SHA。
"""

from __future__ import annotations

import subprocess
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_REL = Path("extensions") / "tools" / "build_fast_lio_underlay.sh"


def _find_repo_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / SCRIPT_REL).is_file():
            return candidate
    raise AssertionError(f"从 {start} 向上找不到 {SCRIPT_REL}")


REPO_ROOT = _find_repo_root(PKG_ROOT)
SCRIPT = REPO_ROOT / SCRIPT_REL

DRIVER = r"""#!/usr/bin/env bash
# 测试驱动：source 构建脚本的库函数，在临时仓库上验证状态机。
set -u
REPO=$1
set --                      # 清空位置参数，避免被脚本的参数解析吃掉
export HEI_FASTLIO_LIB_ONLY=1
# shellcheck disable=SC1090
source "$SCRIPT_PATH"
set +e
# 脚本加载时会把 PATCH_ROOT 重置为真实仓库路径；测试必须改回临时 patch 目录
PATCH_ROOT=$TEST_PATCH_ROOT
PATCHES=$(patch_list fast_lio)
[ -n "$PATCHES" ] || { echo "FAIL 找不到 patch"; exit 1; }

# 复用库里 fail()/step()：不要在这里覆盖 fail，否则会掩盖脚本自身的拒绝路径
ok() { echo "OK $*"; }

worktree_hash() {
  ( cd "$1" && find . -type f -not -path './.git/*' -print0 | sort -z | xargs -0 sha256sum ) | sha256sum | cut -d' ' -f1
}

is_pristine "$REPO" || fail "干净仓库应被识别为 pristine"
ok pristine-detected

apply_patches "$REPO" "$PATCHES" || fail "patch 应用失败"
is_pristine "$REPO" && fail "应用 patch 后不应仍是 pristine"
ok patches-applied

verify_only_patch_delta "$REPO" "$PATCHES" "T" || fail "已应用状态应通过 delta 校验"
verify_expected_content "$REPO" "$PATCHES" || fail "已应用状态应通过内容校验"
ok patched-state-verified

# --- 未知已跟踪改动必须被拒绝，且校验本身不得修改工作区
echo tamper >> "$REPO/other.txt"
before=$(worktree_hash "$REPO")
# 库里失败会 exit（构建脚本的语义），因此用子 shell 捕获，避免直接结束驱动
( verify_only_patch_delta "$REPO" "$PATCHES" "T" ) && fail "未知已跟踪改动必须被拒绝"
after=$(worktree_hash "$REPO")
[ "$before" = "$after" ] || fail "拒绝校验不应修改工作区"
ok unknown-tracked-change-rejected-and-worktree-unchanged
git -C "$REPO" checkout -- other.txt || fail "恢复 other.txt 失败"

# --- 未知未跟踪文件必须被拒绝
echo new > "$REPO/untracked.txt"
( verify_only_patch_delta "$REPO" "$PATCHES" "T" ) && fail "未知未跟踪文件必须被拒绝"
rm -f "$REPO/untracked.txt"
ok unknown-untracked-rejected

# --- 篡改 patch 触碰文件必须被内容校验发现
echo tamper >> "$REPO/f1.txt"
verify_expected_content "$REPO" "$PATCHES" && fail "篡改 patch 文件后内容校验必须失败"
ok tampered-patched-file-detected

# --- 可逆：恢复干净 SHA 后重新应用再撤销
git -C "$REPO" checkout -- . || fail "恢复工作区失败"
is_pristine "$REPO" || fail "恢复后应为 pristine"
apply_patches "$REPO" "$PATCHES" || fail "重新应用 patch 失败"
revert_repo_patches_sparse "$REPO" "$PATCHES" "T" || fail "撤销 patch 失败"
is_pristine "$REPO" || fail "撤销后应回到干净 SHA"
ok revert-restores-pristine

echo DONE
"""


def _run_bash(script: str, *args: str, env: dict | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["/usr/bin/env", "bash", "--noprofile", "--norc", "-c", script, "driver", *args],
        capture_output=True,
        text=True,
        env=env,
        timeout=120,
    )


def _git(repo: Path, *args: str) -> None:
    subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
    )


def test_build_script_patch_state_machine(tmp_path: Path):
    repo = tmp_path / "repo"
    repo.mkdir()
    _git(repo, "init", "-q", ".")
    _git(repo, "config", "user.email", "test@example.invalid")
    _git(repo, "config", "user.name", "test")
    (repo / "f1.txt").write_text("alpha\n", encoding="utf-8")
    (repo / "other.txt").write_text("beta\n", encoding="utf-8")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "init")

    # 生成一个真实可用的 patch：把 f1.txt 从 alpha 改成 ALPHA
    (repo / "f1.txt").write_text("ALPHA\n", encoding="utf-8")
    patch_dir = tmp_path / "patches" / "fast_lio"
    patch_dir.mkdir(parents=True)
    diff = subprocess.run(
        ["git", "-C", str(repo), "diff", "--", "f1.txt"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    (patch_dir / "0001-test.patch").write_text(diff, encoding="utf-8")
    _git(repo, "checkout", "--", "f1.txt")

    driver = tmp_path / "driver.sh"
    driver.write_text(DRIVER, encoding="utf-8")

    env = {
        "PATH": "/usr/bin:/bin",
        "HOME": str(tmp_path),
        "SCRIPT_PATH": str(SCRIPT),
        "TEST_PATCH_ROOT": str(patch_dir.parent),
        "GIT_AUTHOR_NAME": "test",
        "GIT_AUTHOR_EMAIL": "test@example.invalid",
        "GIT_COMMITTER_NAME": "test",
        "GIT_COMMITTER_EMAIL": "test@example.invalid",
    }
    result = _run_bash(driver.read_text(encoding="utf-8"), str(repo), env=env)
    assert result.returncode == 0, (
        "驱动脚本失败：\nSTDOUT:\n"
        + result.stdout
        + "\nSTDERR:\n"
        + result.stderr
    )
    assert "DONE" in result.stdout
    assert "FAIL" not in result.stdout
    for marker in (
        "pristine-detected",
        "patches-applied",
        "patched-state-verified",
        "unknown-tracked-change-rejected-and-worktree-unchanged",
        "unknown-untracked-rejected",
        "tampered-patched-file-detected",
        "revert-restores-pristine",
    ):
        assert marker in result.stdout, f"缺少步骤标记 {marker}"


def test_script_is_sparse_safe_and_bounded():
    """静态护栏：不得为校验/回退下载整树资料，且编译并发受限。"""
    src = SCRIPT.read_text(encoding="utf-8")
    # 注释里可以提到 archive；真正禁用是"整树导出再解包"的调用形式
    assert "git archive HEAD" not in src, "不得用整树 archive 做校验（会触发 doc/ 大文件下载）"
    assert "| tar -x" not in src, "不得把整树导出解包到临时目录"
    assert "revert_repo_patches(" not in src, "回退不得使用整树 archive 的旧实现"
    assert "make_expected_tree" in src
    assert "--filter=blob:none" not in src or "git fetch" in src  # 只在提示里出现获取命令
    assert 'export CMAKE_BUILD_PARALLEL_LEVEL=' in src, "必须限制单包内部编译并发"
    assert 'export MAKEFLAGS="-j' in src
    assert "--parallel-workers 1" in src
    assert "rev-parse --show-toplevel" in src, "子模块 .git 是文件，不能用 -d .git 判断"
    assert "HEI_FASTLIO_LIB_ONLY" in src, "必须保留可测的库模式钩子"
    assert "verify_only_patch_delta" in src
    assert "revert_repo_patches_sparse" in src
