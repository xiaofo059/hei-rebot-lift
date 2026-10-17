#!/usr/bin/env bash
# HEI-LIDAR-20261016-02-PREP
#
# 构建 MID-360S 所需的 Livox SDK2 + livox_ros_driver2 独立 underlay。
# 全程无 sudo：SDK 安装到仓库内 extensions/third_party/livox_sdk2_install，
# driver 在独立工作空间 extensions/third_party/livox_ws 内 colcon 构建。
#
# 必须在“干净 ROS shell”中运行（本脚本会自检，不满足直接失败）：
#   env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
#       PATH=/usr/bin:/bin bash --noprofile --norc
#   source /opt/ros/humble/setup.bash
#   extensions/tools/build_mid360s_underlay.sh
#
# 为什么必须干净 shell：本机 PATH 里 conda base 的 cmake 4.1.3 会覆盖系统
# cmake 3.22.1，而 Livox-SDK2 的 cmake_minimum_required(VERSION 3.0) 在 CMake>=4
# 被拒绝（Compatibility with CMake < 3.5 has been removed）。系统 cmake 3.22.1 正常。
#
# 与官方 build.sh 的差异（本脚本等价但更可追溯）：
#   * 官方 build.sh 会 `rm -rf ../../build/ ../../devel/ ../../install/`，因此只能在
#     独立 livox_ws 里执行，不能放进自研工作空间；本脚本只清理自己的 livox_ws 构建目录。
#   * 官方 build.sh 用 `-DROS_EDITION=ROS2 -DDISTRO_ROS=${ROS_DISTRO}`；本脚本保持相同
#     CMake 参数，并额外显式指定本项目内的 SDK 路径，避免依赖 /usr/local 全局安装。
#
# 冻结 SHA + tracked patch（HEI-LIDAR-20261017-02-FIX）：
#   两个第三方仓库仍按 extensions/dependencies/livox.repos 的 40 位 SHA 冻结；
#   所有必要改动以 patch 形式放在 extensions/dependencies/patches/ 下并由本脚本
#   自动应用/校验，不手改第三方源码：
#     * livox_sdk2:        0001-export-only-public-c-api.patch
#         shared 库只导出 livox_lidar_api.h 的公开 C API，隐藏 bundled
#         spdlog 1.3.1/fmt 内部符号，避免被 ROS 2 载入的系统 libspdlog.so.1
#         (1.9.2) 抢占后退出 SIGBUS/SIGABRT（见 patches/README.md）。
#     * livox_ros_driver2: 0001-monotonic-host-clock.patch
#         NoSync(time_type==0) 路径改用 steady_clock 派生的单调 epoch 时间戳，
#         非同步发布调度改用 steady_clock；PTP/GPS 路径不变。
#   工作区状态机：干净 SHA -> 自动 apply；已应用 patch -> 反向校验 + 逐字节
#   比对「冻结 SHA + patch」；其它任何 dirty -> 拒绝构建。
#   --revert-patches 只撤销 patch 并退出（不构建）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

TP_DIR="${REPO_ROOT}/extensions/third_party"
SDK_SRC="${TP_DIR}/livox_sdk2_src"
SDK_BUILD="${TP_DIR}/livox_sdk2_build"
SDK_INSTALL="${TP_DIR}/livox_sdk2_install"
LIVOX_WS="${TP_DIR}/livox_ws"
DRIVER_DIR="${LIVOX_WS}/src/livox_ros_driver2"
REPOS_FILE="${REPO_ROOT}/extensions/dependencies/livox.repos"
PATCH_ROOT="${REPO_ROOT}/extensions/dependencies/patches"

PARALLEL="${PARALLEL:-4}"
CLEAN=0
REVERT=0
for arg in "$@"; do
  case "${arg}" in
    --clean) CLEAN=1 ;;
    --revert-patches) REVERT=1 ;;
    -h|--help) sed -n '2,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "未知参数: ${arg}" >&2; exit 2 ;;
  esac
done

fail() { echo "错误: $*" >&2; exit 1; }
step() { echo; echo "===> $*"; }

# ---------------------------------------------------------------- 环境自检
step "环境自检"

[ "${ROS_DISTRO:-}" = "humble" ] || fail "未在 Humble 环境中：ROS_DISTRO='${ROS_DISTRO:-未设置}'；请先 source /opt/ros/humble/setup.bash"

command -v ros2 >/dev/null 2>&1 || fail "PATH 中找不到 ros2；请先 source /opt/ros/humble/setup.bash"

CMAKE_BIN="$(command -v cmake || true)"
[ -n "${CMAKE_BIN}" ] || fail "PATH 中找不到 cmake"
case "${CMAKE_BIN}" in
  /usr/bin/cmake) ;;
  *) fail "cmake 解析为 ${CMAKE_BIN}，不是 /usr/bin/cmake。多半是 conda base 污染 PATH；
       请用任务规定的干净 shell（PATH=/usr/bin:/bin）后 source /opt/ros/humble/setup.bash。" ;;
esac
CMAKE_VER="$("${CMAKE_BIN}" --version | head -1)"
echo "  cmake      : ${CMAKE_BIN} (${CMAKE_VER})"
echo "  ROS_DISTRO : ${ROS_DISTRO}"
echo "  repo root  : ${REPO_ROOT}"

# ---------------------------------------------------------------- 依赖 SHA 校验
step "校验依赖源码 SHA（对照 extensions/dependencies/livox.repos）"
[ -f "${REPOS_FILE}" ] || fail "缺少 ${REPOS_FILE}"

expected_sha() {  # $1 = repo key（key 内含 '/'，故不用 sed 的 '/' 定界地址）
  awk -v key="  $1:" '
    $0 == key { inblock = 1; next }
    inblock && /^  [^ ]/ { inblock = 0 }
    inblock && $1 == "version:" { print $2; exit }
  ' "${REPOS_FILE}"
}

check_repo() {  # $1 = path, $2 = repos key
  local path="$1" key="$2" want got dirty
  [ -d "${path}/.git" ] || fail "缺少源码仓库 ${path}；请按 livox.repos 用 vcs import 或 git clone 补齐"
  want="$(expected_sha "${key}")"
  [ -n "${want}" ] || fail "无法从 livox.repos 解析 ${key} 的 version"
  got="$(git -C "${path}" rev-parse HEAD)"
  if [ "${got}" != "${want}" ]; then
    fail "${path} SHA 不符：期望 ${want}，实际 ${got}（拒绝用未冻结的提交构建）"
  fi

  echo "  OK ${key} @ ${got}"
}

# ---------------------------------------------------------------- tracked patch
# 冻结 SHA 必须可追踪：所有第三方改动只允许来自 extensions/dependencies/patches/。
# 实现集中在 extensions/tools/lib/patch_state.sh（可用临时 git 仓库单测）：
#   * 已跟踪改动：git status --porcelain --untracked-files=no；
#     被测仓库若存在未经 tracked patch 覆盖的改动 -> 明确报
#     "存在已跟踪文件的本地修改，拒绝构建"（构建前，工作区不变）。
#   * 未跟踪源码：git ls-files --others --exclude-standard；只允许 patch 新增路径。
#     被 .gitignore 忽略 的生成物（build/、driver package.xml/launch/）不误判。
#   * 干净 SHA -> 自动 apply；已应用 -> 逐字节比对「冻结 SHA + patch」；
#     其它 dirty -> 在**任何修改之前**拒绝。
if [ ! -f "${SCRIPT_DIR}/lib/patch_state.sh" ]; then
  echo "错误: 缺少 ${SCRIPT_DIR}/lib/patch_state.sh" >&2
  exit 1
fi
# shellcheck source=lib/patch_state.sh
. "${SCRIPT_DIR}/lib/patch_state.sh"
check_repo "${SDK_SRC}" "livox_sdk2_src"
check_repo "${DRIVER_DIR}" "livox_ws/src/livox_ros_driver2"

if [ "${REVERT}" = "1" ]; then
  step "撤销 tracked patch（恢复冻结 SHA 工作区）"
  revert_repo_patches "${SDK_SRC}" "livox_sdk2" "Livox-SDK2"
  revert_repo_patches "${DRIVER_DIR}" "livox_ros_driver2" "livox_ros_driver2"
  echo "  OK：第三方工作区已恢复冻结 SHA（未构建）"
  exit 0
fi

step "应用/校验 tracked patch（冻结 SHA + patch）"
prepare_repo "${SDK_SRC}" "livox_sdk2" "Livox-SDK2"
prepare_repo "${DRIVER_DIR}" "livox_ros_driver2" "livox_ros_driver2"

# ---------------------------------------------------------------- SDK2 构建
step "构建并安装 Livox-SDK2（无 sudo，前缀在仓库内）"
if [ "${CLEAN}" = "1" ]; then
  rm -rf "${SDK_BUILD}" "${SDK_INSTALL}"
  echo "  已清理 ${SDK_BUILD} 与 ${SDK_INSTALL}"
fi

"${CMAKE_BIN}" -S "${SDK_SRC}" -B "${SDK_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${SDK_INSTALL}" >"${SDK_BUILD}.configure.log" 2>&1 \
  || { tail -20 "${SDK_BUILD}.configure.log"; fail "Livox-SDK2 configure 失败（完整日志见 ${SDK_BUILD}.configure.log）"; }
"${CMAKE_BIN}" --build "${SDK_BUILD}" -j "${PARALLEL}" >"${SDK_BUILD}.build.log" 2>&1 \
  || { tail -30 "${SDK_BUILD}.build.log"; fail "Livox-SDK2 编译失败（完整日志见 ${SDK_BUILD}.build.log）"; }
"${CMAKE_BIN}" --install "${SDK_BUILD}" >"${SDK_BUILD}.install.log" 2>&1 \
  || { tail -20 "${SDK_BUILD}.install.log"; fail "Livox-SDK2 安装失败（完整日志见 ${SDK_BUILD}.install.log）"; }

SDK_LIB="${SDK_INSTALL}/lib/liblivox_lidar_sdk_shared.so"
SDK_INC="${SDK_INSTALL}/include"
[ -f "${SDK_LIB}" ] || fail "安装后仍找不到 ${SDK_LIB}"
[ -f "${SDK_INC}/livox_lidar_api.h" ] || fail "安装后仍找不到 ${SDK_INC}/livox_lidar_api.h"
echo "  SDK 库    : ${SDK_LIB}"
echo "  SDK 头文件: ${SDK_INC}"

# 符号隔离验收：bundled spdlog/fmt 与内部 C++ 符号不得进入动态符号表，
# 公开 C API 必须仍然导出（否则 driver 链接失败或运行期崩溃）。
step "校验 SDK 动态符号导出（只允许公开 C API）"
leaked="$(nm -D --defined-only "${SDK_LIB}" | grep -E 'spdlog|_ZN3fmt|_ZN5livox5lidar' || true)"
if [ -n "${leaked}" ]; then
  printf '%s\n' "${leaked}" | head -20 >&2
  fail "SDK 仍导出 spdlog/fmt/livox:: 内部符号：符号隔离 patch 未生效"
fi
API_LIST="${PATCH_ROOT}/livox_sdk2/public_c_api.txt"
[ -f "${API_LIST}" ] || fail "缺少公开 C API 清单 ${API_LIST}"
missing=0
exported="$(nm -D --defined-only "${SDK_LIB}" | awk '{print $3}')"
while IFS= read -r sym; do
  [ -n "${sym}" ] || continue
  if ! grep -qx "${sym}" <<< "${exported}"; then
    echo "  缺少导出 C API: ${sym}" >&2
    missing=1
  fi
done < "${API_LIST}"
[ "${missing}" = "0" ] || fail "SDK 公开 C API 导出不完整（符号隔离 patch 过度隐藏）"
echo "  OK: 未导出 spdlog/fmt/livox:: 内部符号"
echo "  OK: 公开 C API 导出完整（$(grep -c . "${API_LIST}") 个）"

# ---------------------------------------------------------------- driver 准备
step "按官方等价方式准备 driver 清单与 launch（不修改第三方源码、不覆盖预存文件）"
[ -f "${DRIVER_DIR}/package_ROS2.xml" ] || fail "缺少 ${DRIVER_DIR}/package_ROS2.xml"
[ -d "${DRIVER_DIR}/launch_ROS2" ] || fail "缺少 ${DRIVER_DIR}/launch_ROS2"

# 只清理“本脚本确实创建”的临时目录（RF-LIDAR-002）。
# 预存的 launch/ 可能是用户自己的内容，绝不无条件 rm -rf。
CREATED_LAUNCH=0
if [ -e "${DRIVER_DIR}/launch" ]; then
  fail "${DRIVER_DIR}/launch 已存在，拒绝删除或覆盖。
       若确认它是本脚本上次异常退出留下的临时目录，请人工删除后重跑：
         rm -rf ${DRIVER_DIR}/launch
       若是你自己的内容，请先移走（脚本不会碰它）。"
fi

# package.xml 同理：忽略文件可能已存在；内容一致则复用，不一致则拒绝覆盖。
if [ -e "${DRIVER_DIR}/package.xml" ]; then
  if cmp -s "${DRIVER_DIR}/package_ROS2.xml" "${DRIVER_DIR}/package.xml"; then
    echo "  package.xml 已存在且与 package_ROS2.xml 一致，复用"
  else
    fail "${DRIVER_DIR}/package.xml 已存在且与 package_ROS2.xml 不同，拒绝覆盖。
       请人工确认后删除或改名，再重跑本脚本。"
  fi
else
  cp -f "${DRIVER_DIR}/package_ROS2.xml" "${DRIVER_DIR}/package.xml"
  echo "  package.xml <- package_ROS2.xml"
fi

cp -rf "${DRIVER_DIR}/launch_ROS2" "${DRIVER_DIR}/launch"
CREATED_LAUNCH=1
echo "  launch/ <- launch_ROS2/（等价于官方 build.sh 的 ROS2 分支）"

# 仅当 launch/ 是本脚本创建时才在退出时清理；预存目录已在上面拒绝执行
cleanup_driver() {
  if [ "${CREATED_LAUNCH}" = "1" ]; then
    rm -rf "${DRIVER_DIR}/launch"
  fi
}
trap cleanup_driver EXIT

# ---------------------------------------------------------------- driver 构建
step "colcon 构建 livox_ros_driver2（显式绑定本项目 SDK，不依赖 /usr/local）"
if [ "${CLEAN}" = "1" ]; then
  rm -rf "${LIVOX_WS}/build" "${LIVOX_WS}/install" "${LIVOX_WS}/log"
  echo "  已清理 ${LIVOX_WS}/{build,install,log}"
fi

pushd "${LIVOX_WS}" >/dev/null
set +u  # colcon 的 setup.bash 会引用未定义变量
colcon build \
  --parallel-workers "${PARALLEL}" \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${SDK_INSTALL}" \
    -DLIVOX_LIDAR_SDK_LIBRARY="${SDK_LIB}" \
    -DLIVOX_LIDAR_SDK_INCLUDE_DIR="${SDK_INC}" \
    -DCMAKE_INSTALL_RPATH="${SDK_INSTALL}/lib" \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
  2>&1 | tee "${LIVOX_WS}/colcon_build.log"
rc="${PIPESTATUS[0]}"
set -u
popd >/dev/null
[ "${rc}" = "0" ] || fail "colcon build 失败（exit=${rc}，日志 ${LIVOX_WS}/colcon_build.log）"

DRIVER_NODE="${LIVOX_WS}/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
DRIVER_LIB="${LIVOX_WS}/install/livox_ros_driver2/lib/liblivox_ros_driver2.so"
[ -x "${DRIVER_NODE}" ] || fail "构建后找不到可执行文件 ${DRIVER_NODE}"
[ -f "${DRIVER_LIB}" ] || fail "构建后找不到组件库 ${DRIVER_LIB}"

# 先 source 本工作空间，否则 ldd 会因缺少 ROS 运行库而满屏 not found（假阳性）。
# 注意：node 只是 rclcpp_components 的瘦加载器，真正链接 Livox SDK 的是组件库
# liblivox_ros_driver2.so，两者都必须检查。
step "ldd 依赖检查（source underlay 后，node 与组件库都不得有 not found）"
set +u
# shellcheck disable=SC1091
source "${LIVOX_WS}/install/local_setup.bash"
set -u
ldd_rc=0
for f in "${DRIVER_NODE}" "${DRIVER_LIB}"; do
  base="$(basename "${f}")"
  out="${LIVOX_WS}/ldd_${base}.txt"
  # ldd 自身失败（文件不存在/不是 ELF/权限问题）必须报错，
  # 不能 `|| true` 之后仍然宣称“检查通过”（RF-LIDAR-002）。
  if ! ldd "${f}" >"${out}" 2>&1; then
    rc=$?
    cat "${out}"
    fail "ldd 执行失败（exit=${rc}）：${f}"
  fi
  echo "  --- ${base} ---"
  if grep -q 'not found' "${out}"; then
    grep 'not found' "${out}"
    ldd_rc=1
  else
    grep -E 'livox_lidar_sdk_shared' "${out}" || echo "  (未直接链接 liblivox_lidar_sdk_shared.so)"
    echo "  no not found  (记录: ${out})"
  fi
done
[ "${ldd_rc}" = "0" ] || fail "ldd 报告 not found；RPATH/依赖解析异常，见 ${LIVOX_WS}/ldd_*.txt"

step "校验 driver 时间修复已生效（tracked patch）"
grep -q "HostNowNs" "${DRIVER_DIR}/src/comm/pub_handler.cpp" \
  || fail "driver 未应用单调时钟 patch：pub_handler.cpp 缺少 HostNowNs"
grep -q "steady_clock::time_point" "${DRIVER_DIR}/src/comm/pub_handler.h" \
  || fail "driver 未应用单调时钟 patch：pub_handler.h 的 TimePoint 不是 steady_clock"
echo "  OK: NoSync 路径使用 HostNowNs；非同步发布调度使用 steady_clock"

step "完成"
cat <<EOF
  SDK 安装前缀 : ${SDK_INSTALL}
  driver 安装  : ${LIVOX_WS}/install
  driver 可执行: ${DRIVER_NODE}
  ldd 记录     : ${LIVOX_WS}/ldd_livox_ros_driver2_node.txt, ${LIVOX_WS}/ldd_liblivox_ros_driver2.so.txt
  colcon 日志  : ${LIVOX_WS}/colcon_build.log
  tracked patch: ${PATCH_ROOT}
    - livox_sdk2/0001-export-only-public-c-api.patch
    - livox_ros_driver2/0001-monotonic-host-clock.patch
  撤销 patch   : extensions/tools/build_mid360s_underlay.sh --revert-patches

  运行时（新终端，仓库根目录）：
    source /opt/ros/humble/setup.bash
    source extensions/third_party/livox_ws/install/local_setup.bash
    ros2 launch hei_robot_bringup mid360s_bringup.launch.py config_path:=<现场配置.json>

  说明：驱动组件库已写入 RPATH=${SDK_INSTALL}/lib，正常无需再设 LD_LIBRARY_PATH；
        若把构建产物搬到别的机器/路径，RPATH 失效，此时才需要
        export LD_LIBRARY_PATH=${SDK_INSTALL}/lib:\${LD_LIBRARY_PATH}
EOF
