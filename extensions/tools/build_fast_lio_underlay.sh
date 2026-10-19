#!/usr/bin/env bash
# HEI-LIDAR-20261017-03-LIO
#
# 构建离线 LIO 所需的第三方 underlay：Ericsii/FAST_LIO_ROS2 (package "fast_lio")。
# 全程无 sudo：源码在 extensions/third_party/fast_lio_ws/src，产物在同工作空间
# build/install/log（/extensions/third_party/ 已被 .gitignore 忽略，不入库）。
#
# 必须在“干净 ROS shell”中运行（脚本会自检，不满足直接失败）：
#   env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
#       PATH=/usr/bin:/bin bash --noprofile --norc
#   source /opt/ros/humble/setup.bash
#   source extensions/third_party/livox_ws/install/local_setup.bash   # 冻结的 driver underlay
#   extensions/tools/build_fast_lio_underlay.sh
#
# 依赖来源与冻结：
#   * extensions/dependencies/fast_lio.repos 给出主仓库与子模块的 40 位 SHA；
#     本脚本**不下载**任何东西（离线可复现），缺失时只打印精确获取命令后退出 2。
#   * 必需改动一律以 tracked patch 放在 extensions/dependencies/patches/fast_lio/：
#       0001-deps-pcl_ros-to-tf2_ros.patch
#           上游 CMakeLists.txt 声明了从未被引用的 pcl_ros（本机未装、禁止 apt 全局安装），
#           却漏掉源码实际 include 的 tf2_ros（原先靠 pcl_ros 传递依赖才编过）-> 最小纠正。
#       0002-frame-end-from-max-point-offset.patch
#           laserMapping.cpp sync_packages() 原用 points.back().curvature 当帧尾；
#           livox_ros_driver2 逐点只按 UDP 到达序追加（pub_handler.cpp:404/432/467，
#           全链路无排序），末点不保证最大。改为对**实际保留点**求 max curvature，
#           不排序消息、不改逐点 offset_time、不改下采样集合。
#
# 工作区状态机（所有判定都在修改之前完成）：
#   * 干净 SHA + 子模块干净 -> 按顺序 apply patch；
#   * 已应用 -> 除 patch 触碰文件外不得有任何 tracked 改动，untracked 只允许
#     patch 新增路径；再逐字节校验「冻结 SHA + patch」；
#   * 其它任何 dirty -> 拒绝构建，且工作区保持原样。
#   校验只比较 patch 触碰文件（不用 git archive 整树）：本任务为低带宽采用 blobless
#   部分克隆 + 稀疏检出跳过 doc/ 大文件，整树 archive 会触发下载。
#   --revert-patches 使用同一套稀疏安全校验，撤销前先验证，不下载任何资料。
#
# 并发：colcon 的 --parallel-workers 只限制“包”级并发；单包内部的 CMake/make 由
#   CMAKE_BUILD_PARALLEL_LEVEL 与 MAKEFLAGS 固定为 PARALLEL（默认 4），避免吃满核心。
#
# 测试钩子：HEI_FASTLIO_LIB_ONLY=1 source 本文件只加载函数，不做环境检查、不改文件。
#
# 用法：extensions/tools/build_fast_lio_underlay.sh [--clean] [--revert-patches]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

TP_DIR="${REPO_ROOT}/extensions/third_party"
FASTLIO_WS="${TP_DIR}/fast_lio_ws"
FASTLIO_SRC="${FASTLIO_WS}/src/FAST_LIO_ROS2"
IKD_SRC="${FASTLIO_SRC}/include/ikd-Tree"
IKD_REL="include/ikd-Tree"
LIVOX_WS="${TP_DIR}/livox_ws"
REPOS_FILE="${REPO_ROOT}/extensions/dependencies/fast_lio.repos"
PATCH_ROOT="${REPO_ROOT}/extensions/dependencies/patches"
PATCH_SUBDIR="fast_lio"
PATCH_LABEL="FAST_LIO_ROS2"

PARALLEL="${PARALLEL:-4}"
CLEAN=0
REVERT=0
for arg in "$@"; do
  case "${arg}" in
    --clean) CLEAN=1 ;;
    --revert-patches) REVERT=1 ;;
    -h|--help) sed -n '2,50p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "未知参数: ${arg}" >&2; exit 2 ;;
  esac
done
case "${PARALLEL}" in
  ''|*[!0-9]*) echo "PARALLEL 必须是正整数，当前='${PARALLEL}'" >&2; exit 2 ;;
esac
if [ "${PARALLEL}" -lt 1 ] || [ "${PARALLEL}" -gt 64 ]; then
  echo "PARALLEL 必须在 1..64，当前=${PARALLEL}" >&2; exit 2
fi

fail() { echo "错误: $*" >&2; exit 1; }
step() { echo; echo "===> $*"; }

# ============================================================================
# 库函数（HEI_FASTLIO_LIB_ONLY=1 source 本文件时只定义这些，不执行构建）
# ============================================================================

patch_list() {  # $1=patch 子目录名
  local d="${PATCH_ROOT}/$1"
  [ -d "${d}" ] || return 0
  find "${d}" -maxdepth 1 -name '*.patch' -print | sort
}

patch_touched_files() {  # $1=repo $2=patch 列表 -> 改动或新建的路径
  local repo="$1" patches="$2" p
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply --numstat "${p}" | awk '{print $3}'
  done <<< "${patches}" | sort -u
}

patch_created_files() {  # $1=repo $2=patch 列表 -> patch 新建的路径
  local repo="$1" patches="$2" p
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply --summary "${p}" | awk '/^ create mode/ {print $4}'
  done <<< "${patches}" | sort -u
}

tracked_dirty_paths() {  # $1=repo -> 已跟踪改动路径（每行一个）
  git -C "$1" status --porcelain --untracked-files=no | sed 's/^...//' | sed 's/^"//; s/"$//'
}

untracked_paths() {  # $1=repo -> 未跟踪且未被忽略的路径
  git -C "$1" ls-files --others --exclude-standard
}

verify_submodule() {  # $1=path $2=want_sha $3=label
  local path="$1" want="$2" label="$3" got dirty untracked
  [ -d "${path}" ] || fail "${label}: 缺少子模块目录 ${path}"
  if ! got="$(git -C "${path}" rev-parse HEAD 2>/dev/null)"; then
    fail "${label}: ${path} 不是 git 工作区（子模块需单独克隆，见 fast_lio.repos）"
  fi
  [ "${got}" = "${want}" ] || fail "${label}: 子模块 SHA 不符，期望 ${want}，实际 ${got}"
  dirty="$(git -C "${path}" status --porcelain --untracked-files=no | sed 's/^...//')"
  untracked="$(git -C "${path}" ls-files --others --exclude-standard)"
  if [ -n "${dirty}" ] || [ -n "${untracked}" ]; then
    printf '  子模块未提交内容:\n%s\n%s\n' "${dirty}" "${untracked}" >&2
    fail "${label}: 子模块 ${path} 工作区不干净，拒绝构建"
  fi
  echo "  ${label}: 子模块 @ ${got}，工作区干净"
}

# 除 patch 触碰文件外不得有任何 tracked 改动；untracked 只允许 patch 新建路径。
# 父仓库可能把“脏子模块”报成 include/ikd-Tree 一项：该路径已在 verify_submodule
# 按 SHA + 干净度单独核对，这里显式豁免，其余一律拒绝。
verify_only_patch_delta() {  # $1=repo $2=patches $3=label
  local repo="$1" patches="$2" label="$3" touched created f rc=0
  touched="$(patch_touched_files "${repo}" "${patches}")"
  created="$(patch_created_files "${repo}" "${patches}")"
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    [ "${f}" = "${IKD_REL}" ] && continue
    if ! grep -qxF "${f}" <<< "${touched}"; then
      echo "  未知的已跟踪改动: ${f}" >&2
      rc=1
    fi
  done <<< "$(tracked_dirty_paths "${repo}")"
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    if ! grep -qxF "${f}" <<< "${created}"; then
      echo "  未知的未跟踪文件（可能被 CMake 纳入构建）: ${f}" >&2
      rc=1
    fi
  done <<< "$(untracked_paths "${repo}")"
  [ "${rc}" = "0" ] || fail "${label}: 存在 patch 之外的改动，拒绝构建（工作区未修改）"
  return 0
}

# HEAD 中 patch 触碰文件的原始内容 + 全部 patch = 期望内容。只用 git show，
# 不做整树 archive，因此不会触发 doc/ 大文件下载。
make_expected_tree() {  # $1=repo $2=patches -> 打印临时目录
  local repo="$1" patches="$2" tmp f p
  tmp="$(mktemp -d)"
  git -C "${tmp}" init -q .
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    mkdir -p "${tmp}/$(dirname "${f}")"
    if ! git -C "${repo}" show "HEAD:${f}" > "${tmp}/${f}" 2>/dev/null; then
      rm -rf "${tmp}"
      fail "${repo}: 取 HEAD:${f} 失败（部分克隆是否缺对象？）"
    fi
  done <<< "$(patch_touched_files "${repo}" "${patches}")"
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    if ! ( cd "${tmp}" && git apply "${p}" ); then
      rm -rf "${tmp}"
      fail "临时树应用 patch 失败: ${p}"
    fi
  done <<< "${patches}"
  echo "${tmp}"
}

verify_expected_content() {  # $1=repo $2=patches -> 0 表示全部逐字节一致
  local repo="$1" patches="$2" tmp f rc=0
  tmp="$(make_expected_tree "${repo}" "${patches}")"
  while IFS= read -r f; do
    [ -n "${f}" ] || continue
    if ! cmp -s "${repo}/${f}" "${tmp}/${f}"; then
      echo "  与「冻结 SHA + patch」不一致: ${f}" >&2
      rc=1
    fi
  done <<< "$(patch_touched_files "${repo}" "${patches}")"
  rm -rf "${tmp}"
  return "${rc}"
}

is_pristine() {  # $1=repo
  local repo="$1"
  if [ -n "$(tracked_dirty_paths "${repo}" | grep -vxF "${IKD_REL}" || true)" ]; then
    return 1
  fi
  [ -z "$(untracked_paths "${repo}")" ] || return 1
  return 0
}

apply_patches() {  # $1=repo $2=patches
  local repo="$1" patches="$2" p
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply "${p}" \
      || fail "应用 patch 失败：${p}（工作区可能已被部分修改，请人工检查）"
    echo "    applied $(basename "${p}")"
  done <<< "${patches}"
}

revert_patches() {  # $1=repo $2=patches（逆序）
  local repo="$1" patches="$2" p
  local -a rev=()
  mapfile -t rev < <(printf '%s\n' "${patches}" | tac)
  for p in "${rev[@]}"; do
    [ -n "${p}" ] || continue
    git -C "${repo}" apply --reverse "${p}" || fail "撤销 patch 失败：${p}"
    echo "    reverted $(basename "${p}")"
  done
}

prepare_repo_patches() {  # $1=repo $2=patches $3=label
  local repo="$1" patches="$2" label="$3" p
  if [ -z "${patches}" ]; then
    is_pristine "${repo}" || fail "${label}: 无 patch 但工作区不干净，拒绝构建"
    echo "  ${label}: 无 patch，工作区干净"
    return 0
  fi
  if is_pristine "${repo}"; then
    echo "  ${label}: 冻结 SHA 干净工作区，按顺序应用 tracked patch"
    apply_patches "${repo}" "${patches}"
  else
    echo "  ${label}: 工作区已有改动，按 patch 集合严格校验（不修改任何文件）"
    verify_only_patch_delta "${repo}" "${patches}" "${label}"
    verify_expected_content "${repo}" "${patches}" \
      || fail "${label}: 工作区与「冻结 SHA + patch」不一致，拒绝构建（工作区未修改）"
    echo "  ${label}: 检测到 patch 已完整应用，逐字节校验通过"
  fi
  while IFS= read -r p; do
    [ -n "${p}" ] || continue
    echo "    sha256 $(sha256sum "${p}" | cut -d' ' -f1)  $(basename "${p}")"
  done <<< "${patches}"
}

revert_repo_patches_sparse() {  # $1=repo $2=patches $3=label
  local repo="$1" patches="$2" label="$3"
  if [ -z "${patches}" ]; then
    echo "  ${label}: 无 patch"
    return 0
  fi
  if is_pristine "${repo}"; then
    echo "  ${label}: 已是冻结 SHA，无需撤销"
    return 0
  fi
  verify_only_patch_delta "${repo}" "${patches}" "${label}"
  verify_expected_content "${repo}" "${patches}" \
    || fail "${label}: 工作区不符合「冻结 SHA + patch」，拒绝撤销（工作区未修改）"
  revert_patches "${repo}" "${patches}"
  is_pristine "${repo}" || fail "${label}: 撤销 patch 后工作区仍不干净，请人工检查"
  echo "  ${label}: 已恢复冻结 SHA，工作区干净"
}

expected_sha() {  # $1=repos key
  awk -v key="  $1:" '
    $0 == key { inblock = 1; next }
    inblock && /^  [^ ]/ { inblock = 0 }
    inblock && $1 == "version:" { print $2; exit }
  ' "${REPOS_FILE}"
}

check_repo_sha() {  # $1=path $2=repos key $3=label
  local path="$1" key="$2" label="$3" want got toplevel
  [ -d "${path}" ] || fail "${label}: 缺少源码目录 ${path}"
  # 子模块的 .git 是文件而不是目录，不能用 -d .git 判断
  if ! toplevel="$(git -C "${path}" rev-parse --show-toplevel 2>/dev/null)"; then
    fail "${label}: ${path} 不是 git 工作区"
  fi
  [ "$(cd "${toplevel}" && pwd -P)" = "$(cd "${path}" && pwd -P)" ] \
    || fail "${label}: ${path} 的 git 顶层是 ${toplevel}，与实际目录不一致（拒绝误判嵌套仓库）"
  want="$(expected_sha "${key}")"
  [ -n "${want}" ] || fail "无法从 fast_lio.repos 解析 ${key} 的 version"
  got="$(git -C "${path}" rev-parse HEAD)"
  [ "${got}" = "${want}" ] || fail "${label}: SHA 不符，期望 ${want}，实际 ${got}（拒绝用未冻结提交构建）"
  echo "  OK ${key} @ ${got}"
}

if [ "${HEI_FASTLIO_LIB_ONLY:-0}" = "1" ]; then
  return 0 2>/dev/null || exit 0
fi

# ============================================================================
# 主流程
# ============================================================================
step "环境自检"
[ "${ROS_DISTRO:-}" = "humble" ] || fail "未在 Humble 环境中：ROS_DISTRO='${ROS_DISTRO:-未设置}'；请先 source /opt/ros/humble/setup.bash"
command -v ros2 >/dev/null 2>&1 || fail "PATH 中找不到 ros2；请先 source /opt/ros/humble/setup.bash"
command -v colcon >/dev/null 2>&1 || fail "PATH 中找不到 colcon"

CMAKE_BIN="$(command -v cmake || true)"
[ -n "${CMAKE_BIN}" ] || fail "PATH 中找不到 cmake"
case "${CMAKE_BIN}" in
  /usr/bin/cmake) ;;
  *) fail "cmake 解析为 ${CMAKE_BIN}，不是 /usr/bin/cmake（多为 conda base 污染 PATH）。
       请用干净 shell（PATH=/usr/bin:/bin）后 source /opt/ros/humble/setup.bash。" ;;
esac
CXX_BIN="$(command -v g++ || true)"
case "${CXX_BIN}" in
  /usr/bin/g++) ;;
  *) fail "g++ 解析为 ${CXX_BIN:-未找到}，不是 /usr/bin/g++；请使用干净 shell" ;;
esac
if [ ! -f "${LIVOX_WS}/install/local_setup.bash" ]; then
  fail "缺少已冻结的 Livox underlay：${LIVOX_WS}/install
       请先运行 extensions/tools/build_mid360s_underlay.sh（本脚本不重建、不修改 driver/SDK）"
fi
ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1 \
  || fail "当前环境解析不到 livox_ros_driver2。请先 source ${LIVOX_WS}/install/local_setup.bash"

echo "  cmake          : ${CMAKE_BIN} ($("${CMAKE_BIN}" --version | head -1))"
echo "  g++            : ${CXX_BIN} ($("${CXX_BIN}" --version | head -1))"
echo "  ROS_DISTRO     : ${ROS_DISTRO}"
echo "  livox underlay : ${LIVOX_WS}/install"
echo "  并行度         : PARALLEL=${PARALLEL}（colcon 包并发 1；CMake/make 由环境变量限制）"
echo "  repo root      : ${REPO_ROOT}"

step "检查第三方源码是否存在（本脚本不下载）"
if [ ! -e "${FASTLIO_SRC}/.git" ]; then
  cat >&2 <<CLONE_EOF
错误: 缺少 ${FASTLIO_SRC}

按 extensions/dependencies/fast_lio.repos 获取（二选一，均可核对 SHA）：

  # A) vcstool（会同时取子模块）
  mkdir -p ${FASTLIO_WS}
  vcs import ${FASTLIO_WS} < ${REPOS_FILE}

  # B) 手工低带宽（本任务实际采用：blobless + 稀疏检出，跳过 doc/ 大文件）
  mkdir -p ${FASTLIO_WS}/src && cd ${FASTLIO_WS}/src
  git init FAST_LIO_ROS2 && cd FAST_LIO_ROS2
  git remote add origin https://github.com/Ericsii/FAST_LIO_ROS2.git
  git config --local remote.origin.promisor true
  git config --local remote.origin.partialclonefilter blob:none
  git fetch --depth 1 --filter=blob:none --no-tags origin ros2
  git checkout -B ros2 FETCH_HEAD
  git sparse-checkout init --cone
  git sparse-checkout set src include config launch msg rviz rviz_cfg Log
  # 子模块 ikd-Tree（精确 SHA 见 fast_lio.repos）
  cd include && git init ikd-Tree && cd ikd-Tree
  git remote add origin https://github.com/hku-mars/ikd-Tree.git
  git config --local remote.origin.promisor true
  git config --local remote.origin.partialclonefilter blob:none
  git fetch --depth 1 --filter=blob:none --no-tags origin \
      e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
  git checkout -B fast_lio FETCH_HEAD

CLONE_EOF
  exit 2
fi

step "校验依赖源码 SHA 与子模块状态（对照 extensions/dependencies/fast_lio.repos）"
[ -f "${REPOS_FILE}" ] || fail "缺少 ${REPOS_FILE}"
[ -f "${FASTLIO_SRC}/.gitmodules" ] || fail "缺少 ${FASTLIO_SRC}/.gitmodules"
grep -q "${IKD_REL}" "${FASTLIO_SRC}/.gitmodules" \
  || fail "上游 .gitmodules 未包含 ${IKD_REL}：与审计基线不符，拒绝构建"
check_repo_sha "${FASTLIO_SRC}" "src/FAST_LIO_ROS2" "${PATCH_LABEL}"
verify_submodule "${IKD_SRC}" "$(expected_sha "src/FAST_LIO_ROS2/include/ikd-Tree")" "${PATCH_LABEL}"

PATCHES="$(patch_list "${PATCH_SUBDIR}")"
[ -n "${PATCHES}" ] || fail "缺少 patch：${PATCH_ROOT}/${PATCH_SUBDIR}/*.patch（依赖纠正与帧尾修复是必要改动）"

if [ "${REVERT}" = "1" ]; then
  step "撤销 tracked patch（恢复冻结 SHA 工作区，不下载任何资料）"
  revert_repo_patches_sparse "${FASTLIO_SRC}" "${PATCHES}" "${PATCH_LABEL}"
  echo "  OK：第三方工作区已恢复冻结 SHA（未构建）"
  exit 0
fi

step "应用/校验 tracked patch（冻结 SHA + patch）"
prepare_repo_patches "${FASTLIO_SRC}" "${PATCHES}" "${PATCH_LABEL}"

step "colcon 构建 fast_lio（依赖已冻结的 livox_ws underlay）"
if [ "${CLEAN}" = "1" ]; then
  rm -rf "${FASTLIO_WS}/build" "${FASTLIO_WS}/install" "${FASTLIO_WS}/log"
  echo "  已清理 ${FASTLIO_WS}/{build,install,log}"
fi

set +u
# shellcheck disable=SC1091
source "${LIVOX_WS}/install/local_setup.bash"
set -u

# 真正限制单包内部编译作业数（--parallel-workers 只管包级并发）
export CMAKE_BUILD_PARALLEL_LEVEL="${PARALLEL}"
export MAKEFLAGS="-j${PARALLEL}"

pushd "${FASTLIO_WS}" >/dev/null
set +u
colcon build \
  --packages-select fast_lio \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tee "${FASTLIO_WS}/fast_lio_colcon_build.log"
rc="${PIPESTATUS[0]}"
set -u
popd >/dev/null
[ "${rc}" = "0" ] || fail "colcon build 失败（exit=${rc}，日志 ${FASTLIO_WS}/fast_lio_colcon_build.log）"

FASTLIO_BIN="${FASTLIO_WS}/install/fast_lio/lib/fast_lio/fastlio_mapping"
[ -x "${FASTLIO_BIN}" ] || fail "构建后找不到可执行文件 ${FASTLIO_BIN}"

step "校验 patch 已生效（源码事实，不是运行结论）"
grep -q "frame_end_curvature" "${FASTLIO_SRC}/src/laserMapping.cpp" \
  || fail "0002 patch 未生效：laserMapping.cpp 缺少 frame_end_curvature"
if grep -q "points.back().curvature" "${FASTLIO_SRC}/src/laserMapping.cpp"; then
  fail "0002 patch 未生效：laserMapping.cpp 仍以 points.back().curvature 作帧尾"
fi
if grep -q "pcl_ros" "${FASTLIO_SRC}/CMakeLists.txt" || grep -q "pcl_ros" "${FASTLIO_SRC}/package.xml"; then
  fail "0001 patch 未生效：仍声明未安装的 pcl_ros"
fi
grep -q "find_package(tf2_ros REQUIRED)" "${FASTLIO_SRC}/CMakeLists.txt" \
  || fail "0001 patch 未生效：CMakeLists.txt 未显式声明实际使用的 tf2_ros"
grep -q "<depend>tf2_ros</depend>" "${FASTLIO_SRC}/package.xml" \
  || fail "0001 patch 未生效：package.xml 未声明 tf2_ros"
echo "  OK: 帧尾取 max curvature；pcl_ros 已移除、tf2_ros 已显式声明"

step "校验逐点时间语义未被改写（0002 只改帧尾参考）"
grep -q "use curvature as time of each laser points" "${FASTLIO_SRC}/src/preprocess.cpp" \
  || fail "preprocess.cpp 的逐点时间语义被意外改动"
grep -q "offset_time" "${FASTLIO_SRC}/src/preprocess.cpp" \
  || fail "preprocess.cpp 不再从 offset_time 取逐点时间"
echo "  OK: offset_time -> curvature(ms) 仍是唯一逐点时间来源"

step "ldd 依赖检查（source underlay 后不得有 not found）"
set +u
# shellcheck disable=SC1091
source "${FASTLIO_WS}/install/local_setup.bash"
set -u
LDD_OUT="${FASTLIO_WS}/ldd_fastlio_mapping.txt"
if ! ldd "${FASTLIO_BIN}" > "${LDD_OUT}" 2>&1; then
  rc=$?
  cat "${LDD_OUT}"
  fail "ldd 执行失败（exit=${rc}）：${FASTLIO_BIN}"
fi
if grep -q 'not found' "${LDD_OUT}"; then
  grep 'not found' "${LDD_OUT}"
  fail "ldd 报告 not found；见 ${LDD_OUT}"
fi
echo "  no not found（记录: ${LDD_OUT}）"

step "完成"
cat <<SUMMARY_EOF
  FAST_LIO 源码 : ${FASTLIO_SRC}
  ikd-Tree      : ${IKD_SRC}
  FAST_LIO 安装 : ${FASTLIO_WS}/install
  可执行文件    : ${FASTLIO_BIN}
  colcon 日志   : ${FASTLIO_WS}/fast_lio_colcon_build.log
  tracked patch : ${PATCH_ROOT}/${PATCH_SUBDIR}/
  撤销 patch    : extensions/tools/build_fast_lio_underlay.sh --revert-patches

  运行时（新终端，仓库根目录；本 underlay 只用于**离线**回放）：
    source /opt/ros/humble/setup.bash
    source extensions/third_party/livox_ws/install/local_setup.bash
    source extensions/third_party/fast_lio_ws/install/local_setup.bash
    source extensions/ros2_ws/install/local_setup.bash
    ros2 launch hei_robot_localization fast_lio_mid360s_offline.launch.py \\
        bag_path:=/home/robo/hei-data/mid360s_live_20261017_retest01 uncalibrated:=true

  注意：外参为厂商名义值、**未经现场标定**；launch 要求显式 uncalibrated:=true。
SUMMARY_EOF
