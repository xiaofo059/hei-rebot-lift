#!/usr/bin/env bash
# HEI-LIDAR-20261017-02-FIX: tracked-patch state machine for third-party worktrees.
#
# The frozen baseline SHA stays verifiable: the only allowed delta from HEAD is
# exactly the tracked patch set (extensions/dependencies/patches/).  A worktree
# is accepted only in one of two states, and both are decided BEFORE any
# mutation, so a rejected run never changes the worktree:
#   * pristine: git diff --quiet HEAD and no non-ignored untracked files;
#   * fully patched: byte-identical to a temporary tree built from
#     "git archive HEAD + patches" (stacked patches included).
# Anything else is unknown dirty and rejected.
#
# Caller must provide: PATCH_ROOT, fail().
# shellcheck shell=bash

patch_list() {  # $1 = patch subdir under PATCH_ROOT
  local d="${PATCH_ROOT}/$1"
  [ -d "${d}" ] || return 0
  find "${d}" -maxdepth 1 -name '*.patch' -print | sort
}

patch_touched_files() {  # $1=repo, $2=patch list -> paths modified or created
  local repo="$1" patches="$2" p
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply --numstat "${p}" | awk '{print $3}'
  done <<< "${patches}" | sort -u
}

patch_created_files() {  # $1=repo, $2=patch list -> paths created by patches
  local repo="$1" patches="$2" p
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply --summary "${p}" | awk '/^ create mode/ {print $4}'
  done <<< "${patches}" | sort -u
}

# Hash of every worktree file except .git (used by the regression test and by
# "rejection left the worktree unchanged" assertions).
patch_worktree_hash() {  # $1=repo
  local repo="$1"
  ( cd "${repo}" && find . -type f -not -path './.git/*' -print0 | sort -z | xargs -0 sha256sum ) | sha256sum | cut -d' ' -f1
}

verify_pristine_tree() {  # $1=repo : exactly HEAD, no non-ignored untracked
  local repo="$1"
  git -C "${repo}" diff --quiet HEAD -- || return 1
  [ -z "$(git -C "${repo}" ls-files --others --exclude-standard)" ] || return 1
  return 0
}

# Compare the worktree with a temporary tree built from git archive HEAD + all
# patches (applied in order, so stacked patches work).  Rejects wrong content,
# unknown tracked modifications, and non-ignored untracked files that the patch
# set does not create (such files could be picked up by CMake globs).
verify_patched_tree() {  # $1=repo, $2=patch list
  local repo="$1" patches="$2" tmp p f touched created dirty rc=0
  tmp="$(mktemp -d)"
  if ! ( cd "${repo}" && git archive HEAD ) | tar -x -C "${tmp}"; then
    rm -rf "${tmp}"; return 1
  fi
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    if ! ( cd "${tmp}" && git apply "${p}" ); then
      rm -rf "${tmp}"; return 1
    fi
  done <<< "${patches}"
  touched="$(patch_touched_files "${repo}" "${patches}")"
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    if ! cmp -s "${repo}/${f}" "${tmp}/${f}"; then
      echo "错误: ${repo}/${f} 与「冻结 SHA + patch」内容不一致" >&2
      rc=1
    fi
  done <<< "${touched}"
  dirty="$(git -C "${repo}" status --porcelain --untracked-files=no | sed 's/^...//')"
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    if ! grep -qxF "${f}" <<< "${touched}"; then
      echo "错误: ${repo} 存在未知的已跟踪改动: ${f}" >&2
      rc=1
    fi
  done <<< "${dirty}"
  created="$(patch_created_files "${repo}" "${patches}")"
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    if ! grep -qxF "${f}" <<< "${created}"; then
      echo "错误: ${repo} 存在未知的未跟踪文件（可能被 CMake glob 纳入构建）: ${f}" >&2
      rc=1
    fi
  done <<< "$(git -C "${repo}" ls-files --others --exclude-standard)"
  rm -rf "${tmp}"
  return "${rc}"
}

# Accept a repo only as pristine (then apply the patch set) or as fully patched
# (then just verify).  Both decisions happen before any mutation.
prepare_repo() {  # $1=repo, $2=patch subdir, $3=label
  local repo="$1" pdir="$2" label="$3" patches p count
  patches="$(patch_list "${pdir}")"
  if [ -z "${patches}" ]; then
    if ! verify_pristine_tree "${repo}"; then
      git -C "${repo}" status --porcelain >&2
      fail "${label}: 存在已跟踪文件的本地修改或未知未跟踪文件，且没有对应 patch；拒绝构建"
    fi
    echo "  ${label}: 无 patch，工作区干净"
    return 0
  fi
  if verify_pristine_tree "${repo}"; then
    echo "  ${label}: 冻结 SHA 干净工作区，按顺序应用 tracked patch"
    while IFS= read -r p; do
      [ -n "${p}" ] || continue
      git -C "${repo}" apply "${p}" || fail "${label}: 应用 patch 失败：${p}"
      echo "    applied $(basename "${p}")"
    done <<< "${patches}"
  elif verify_patched_tree "${repo}" "${patches}"; then
    echo "  ${label}: 检测到 patch 已完整应用，逐字节校验「冻结 SHA + patch」"
  else
    fail "${label}: 工作区既不是冻结 SHA，也不等于「冻结 SHA + patch」；拒绝构建（工作区未修改）"
  fi
  count="$(printf '%s\n' "${patches}" | grep -c . || true)"
  if ! verify_patched_tree "${repo}" "${patches}"; then
    fail "${label}: 工作区与「冻结 SHA + patch」不一致（未知 dirty），拒绝构建"
  fi
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    echo "    sha256 $(sha256sum "${p}" | cut -d' ' -f1)  $(basename "${p}")"
  done <<< "${patches}"
  echo "  ${label}: patch 校验通过（${count} 个）"
}

# Revert the tracked patch set.  Refuses (without touching anything) unless the
# worktree is exactly "frozen SHA + patch set".  Multiple patches revert in
# reverse application order.
revert_repo_patches() {  # $1=repo, $2=patch subdir, $3=label
  local repo="$1" pdir="$2" label="$3" patches p rev
  patches="$(patch_list "${pdir}")"
  if [ -z "${patches}" ]; then
    echo "  ${label}: 无 patch"
    return 0
  fi
  if ! verify_patched_tree "${repo}" "${patches}"; then
    fail "${label}: 工作区不符合「冻结 SHA + patch」，拒绝撤销（工作区未修改）"
  fi
  mapfile -t rev < <(printf '%s\n' "${patches}" | tac)
  for p in "${rev[@]}"; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply --reverse "${p}" || fail "${label}: 撤销 patch 失败：${p}"
    echo "    reverted $(basename "${p}")"
  done
  if ! verify_pristine_tree "${repo}"; then
    fail "${label}: 撤销 patch 后工作区不干净"
  fi
}
