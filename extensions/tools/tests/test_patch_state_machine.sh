#!/usr/bin/env bash
# HEI-LIDAR-20261017-02-FIX: regression test for extensions/tools/lib/patch_state.sh
#
# Uses throw-away git repositories under mktemp -d only; it never touches the
# real third-party worktrees or user files. Cases:
#   clean / apply / re-apply(idempotent) / unknown tracked / same-file unknown
#   change / unknown untracked source / ignored untracked allowed / revert /
#   revert refused on unknown dirty / reverse-order revert of stacked patches.
# Every rejection case asserts the worktree hash is unchanged.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
LIB="${REPO_ROOT}/extensions/tools/lib/patch_state.sh"
[ -f "${LIB}" ] || { echo "缺少 ${LIB}" >&2; exit 1; }

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "PASS: $*"; }
# shellcheck source=../lib/patch_state.sh
. "${LIB}"

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

# ---------------------------------------------------------------- fixture
SRC="${TMP}/src"
mkdir -p "${SRC}"
git -C "${SRC}" init -q
git -C "${SRC}" config user.email test@example.com
git -C "${SRC}" config user.name test
printf 'base\n' > "${SRC}/a.txt"
printf 'other\n' > "${SRC}/c.txt"
printf 'build/\n' > "${SRC}/.gitignore"
git -C "${SRC}" add -A
git -C "${SRC}" commit -qm base
base_commit="$(git -C "${SRC}" rev-parse HEAD)"

printf 'base\none\n' > "${SRC}/a.txt"
printf 'created\n' > "${SRC}/new.txt"
git -C "${SRC}" add -A
git -C "${SRC}" commit -qm p1
printf 'base\none\ntwo\n' > "${SRC}/a.txt"
git -C "${SRC}" add -A
git -C "${SRC}" commit -qm p2

PATCH_ROOT="${TMP}/patches"
mkdir -p "${PATCH_ROOT}/livox_test" "${PATCH_ROOT}/empty"
git -C "${SRC}" diff "${base_commit}" HEAD~1 > "${PATCH_ROOT}/livox_test/0001-one.patch"
git -C "${SRC}" diff HEAD~1 HEAD > "${PATCH_ROOT}/livox_test/0002-two.patch"
git -C "${SRC}" reset -q --hard "${base_commit}"

WORK="${TMP}/work"
git clone -q "${SRC}" "${WORK}"
h0="$(patch_worktree_hash "${WORK}")"

expect_reject() {  # $1=description, rest=command
  local desc="$1"; shift
  local before; before="$(patch_worktree_hash "${WORK}")"
  if ( "$@" >/dev/null 2>&1 ); then fail "${desc}: 本应拒绝却成功"; fi
  [ "$(patch_worktree_hash "${WORK}")" = "${before}" ] || fail "${desc}: 拒绝时改动了工作区"
  pass "${desc}（拒绝且工作区未变）"
}

# 1. 没有 patch 的干净仓库
( prepare_repo "${WORK}" "empty" "t" >/dev/null ) || fail "空 patch 集应通过"
[ "$(patch_worktree_hash "${WORK}")" = "${h0}" ] || fail "空 patch 集不应改动工作区"
pass "干净仓库 + 空 patch 集"

# 2. 干净 -> 按序应用两个叠放 patch
( prepare_repo "${WORK}" "livox_test" "t" >/dev/null ) || fail "应用 patch 失败"
[ -f "${WORK}/new.txt" ] || fail "patch 新增文件缺失"
printf 'base\none\ntwo\n' > "${TMP}/expected_a.txt"
cmp -s "${TMP}/expected_a.txt" "${WORK}/a.txt" || fail "叠放 patch 内容不正确"
h1="$(patch_worktree_hash "${WORK}")"
pass "干净 -> 按序应用 2 个 patch"

# 3. 已应用状态重复执行必须幂等
( prepare_repo "${WORK}" "livox_test" "t" >/dev/null ) || fail "已应用状态应通过"
[ "$(patch_worktree_hash "${WORK}")" = "${h1}" ] || fail "重复执行改动了工作区"
pass "已应用状态幂等"

# 4. 撤销必须按逆序（否则同文件叠放 patch 会失败）
( revert_repo_patches "${WORK}" "livox_test" "t" >/dev/null ) || fail "撤销 patch 失败"
[ "$(patch_worktree_hash "${WORK}")" = "${h0}" ] || fail "撤销后未恢复冻结基线"
pass "逆序撤销 2 个叠放 patch"

# 5. 干净态 + 未知已跟踪改动：必须在 apply 之前拒绝
printf 'junk\n' >> "${WORK}/c.txt"
expect_reject "干净态未知已跟踪改动" prepare_repo "${WORK}" "livox_test" "t"
[ ! -f "${WORK}/new.txt" ] || fail "被拒绝后仍然应用了 patch"
git -C "${WORK}" checkout -q -- c.txt

# 6. 干净态 + 同文件未知改动：patch 状态不确定，拒绝
printf 'evil\n' >> "${WORK}/a.txt"
expect_reject "同文件未知改动" prepare_repo "${WORK}" "livox_test" "t"
git -C "${WORK}" checkout -q -- a.txt

# 7. 干净态 + 未知未跟踪源码文件：拒绝（可能被 CMake glob 纳入构建）
printf 'int main(){}\n' > "${WORK}/unknown.cpp"
expect_reject "未知未跟踪源码文件" prepare_repo "${WORK}" "livox_test" "t"
rm -f "${WORK}/unknown.cpp"

# 8. 被 .gitignore 忽略的未跟踪产物允许存在
mkdir -p "${WORK}/build"
printf 'obj\n' > "${WORK}/build/x.o"
( prepare_repo "${WORK}" "livox_test" "t" >/dev/null ) || fail "被忽略的未跟踪产物不应导致拒绝"
pass "被忽略的未跟踪产物允许"
rm -rf "${WORK}/build"

# 9. 已应用态 + 未知已跟踪改动：撤销必须在修改前拒绝
printf 'dirty\n' >> "${WORK}/c.txt"
expect_reject "已应用态未知脏改动时撤销" revert_repo_patches "${WORK}" "livox_test" "t"
git -C "${WORK}" checkout -q -- c.txt

# 10. 收尾撤销并核对
( revert_repo_patches "${WORK}" "livox_test" "t" >/dev/null ) || fail "收尾撤销失败"
[ "$(patch_worktree_hash "${WORK}")" = "${h0}" ] || fail "收尾撤销后未恢复冻结基线"
[ ! -f "${WORK}/new.txt" ] || fail "撤销后 patch 新增文件仍存在"
pass "收尾撤销恢复冻结基线"

echo
echo "ALL PATCH STATE TESTS PASSED"
