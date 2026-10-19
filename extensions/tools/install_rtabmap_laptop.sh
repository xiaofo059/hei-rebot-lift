#!/usr/bin/env bash
# HEI MID-360S 三维 SLAM：RTAB-Map 0.23.13（ROS 2 Humble 官方 apt 包）依赖安装/核验脚本
#
# 为什么有这个脚本：
#   本机 ROS 2 Humble 已存在，但 ros-humble-rtabmap* 未安装；同时本机 sudo 需要密码
#   （sudo -n 不可用），所以提供两条路径：
#     1) --apt          有密码/可交互 sudo 时，走官方 ROS apt 常规安装（先 dry-run 记录）；
#     2) --local-prefix 默认：无需 root，用 apt-get download 下载**固定版本**的官方 deb，
#        校验 SHA256 后解包到 gitignored 的 extensions/third_party/rtabmap_local/ 前缀，
#        生成 env.sh 并做导入/动态链接核验。不改 /opt、不改 apt 源、不改 bashrc。
#
# 固定版本与 SHA256 来自本机实际下载的官方包（见下 MANIFEST）；版本由 2026-10-19
# ROS 2 Humble apt 源快照提供，rtabmap 上游版本 0.23.13。
#
# 用法：
#   extensions/tools/install_rtabmap_laptop.sh                 # 本地前缀安装（默认）
#   extensions/tools/install_rtabmap_laptop.sh --verify        # 只核验已安装前缀
#   extensions/tools/install_rtabmap_laptop.sh --apt           # 官方 apt 安装（需要 sudo）
#
# 安装后，运行 SLAM 前在终端 source：
#   source extensions/third_party/rtabmap_local/env.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEFAULT_PREFIX="${REPO_ROOT}/extensions/third_party/rtabmap_local"
PREFIX="${RTABMAP_LOCAL_PREFIX:-${DEFAULT_PREFIX}}"
RTABMAP_VERSION="0.23.13"
CACHE_DIR="${PREFIX}/debs"
UNPACK_DIR="${PREFIX}/prefix"

# 官方 deb 固定清单：package/version/architecture/sha256/file（本机 2026-10-19 实际下载并校验）
read -r -d "" MANIFEST_JSON <<'RTABMAP_DEB_MANIFEST' || true
[
 {
  "package": "libamd2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "fb1304a59465049d4c20e886acb316ba54ae1a6e41e287e8f7a0dd351a899bae",
  "file": "libamd2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libbtf1",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "f6201a3428f3b8e731c21beb48b786d456fe601936f4666244242426637f265f",
  "file": "libbtf1_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libcamd2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "b771ae509a48132b2c00e050589abcc379c212c3777d0621e989898de4f144bf",
  "file": "libcamd2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libccolamd2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "d8f24c71ee5e8da8fd633f1e6c78a1a6f70c5752b50f816dcae548a929a5cc0f",
  "file": "libccolamd2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libcholmod3",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "440d5d208f1eeead716166203de85a973d102bd06d9dd8f46a11ce6f357a5d1d",
  "file": "libcholmod3_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libcolamd2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "6893bdfcd37c2e4110f3a8de4b536b9408613d8d01b99f9a468b5fd0cf245f89",
  "file": "libcolamd2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libcxsparse3",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "d78cb3f8599fc1352bc9450b432c5c7c6d42100d129e39f84566c812cf4b24f1",
  "file": "libcxsparse3_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libklu1",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "f16a0091e5cb3c4023730ef0b98157d64b742ccb335f67b4503083e6beecaf22",
  "file": "libklu1_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libldl2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "275f3821a9c5be97e5bf8831b09cdfa918eb2ab9d949f52b21476b6e80b75dc5",
  "file": "libldl2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libmetis5",
  "version": "5.1.0.dfsg-7build2",
  "architecture": "amd64",
  "sha256": "357ce72c2d3698feeb54212edfadd334e06789458cece8d61636e2dadb2fb080",
  "file": "libmetis5_5.1.0.dfsg-7build2_amd64.deb"
 },
 {
  "package": "libmongoose2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "da0508dabcb9c0cdfda8821c4240ab7eb208bce6f62561179ac482af5a5f9f27",
  "file": "libmongoose2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "librbio2",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "911c987489b4ae86f80a735aba59f513f768cb33695c9eca0ed20b0efc3d69ed",
  "file": "librbio2_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libsliplu1",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "cb590a9e927efc96690944bbdf7cb02849ba6adf1cf2c6b1537170f082b95ed5",
  "file": "libsliplu1_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libsuitesparse-dev",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "46834c804b105bdb627d5b1830e8edc0bf61dcc3c3cc29da848fcde08b1b3e23",
  "file": "libsuitesparse-dev_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libsuitesparseconfig5",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "5e0433af3c3a5fa0c20073804fc1f73834b34dd98633ad9ff0a33a28c350830b",
  "file": "libsuitesparseconfig5_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "libumfpack5",
  "version": "1:5.10.1+dfsg-4build1",
  "architecture": "amd64",
  "sha256": "3cf270a12d0cf6f168c5866aaa440059421dcc210cfcaf15dc1cd450a08d3d8f",
  "file": "libumfpack5_1%3a5.10.1+dfsg-4build1_amd64.deb"
 },
 {
  "package": "ros-humble-apriltag-msgs",
  "version": "2.0.2-1jammy.20260907.213637",
  "architecture": "amd64",
  "sha256": "d310e0b272af0bcbede6b900d20a6b35b9ed2cf71b21213dd44cbad24cfe4878",
  "file": "ros-humble-apriltag-msgs_2.0.2-1jammy.20260907.213637_amd64.deb"
 },
 {
  "package": "ros-humble-aruco-msgs",
  "version": "5.0.6-1jammy.20261011.054947",
  "architecture": "amd64",
  "sha256": "45c9e8c26e65a861a67b84cf0bda6181c210e5a2d2acceb422ba9dbba52caab2",
  "file": "ros-humble-aruco-msgs_5.0.6-1jammy.20261011.054947_amd64.deb"
 },
 {
  "package": "ros-humble-aruco-opencv-msgs",
  "version": "2.4.2-1jammy.20260907.211203",
  "architecture": "amd64",
  "sha256": "357f3b61266b43557d2eb8344c6e431a8c744da4f6d4f0c33f792c5ea5a84c1d",
  "file": "ros-humble-aruco-opencv-msgs_2.4.2-1jammy.20260907.211203_amd64.deb"
 },
 {
  "package": "ros-humble-diagnostic-updater",
  "version": "4.0.7-1jammy.20260907.223805",
  "architecture": "amd64",
  "sha256": "8687b9e56d001d02e622a8c62100e06ae277fe4209f0c32919cc9d3af3ea9ae0",
  "file": "ros-humble-diagnostic-updater_4.0.7-1jammy.20260907.223805_amd64.deb"
 },
 {
  "package": "ros-humble-grid-map-core",
  "version": "2.0.1-1jammy.20260304.202024",
  "architecture": "amd64",
  "sha256": "eb9cd0c764e4f428eb92c5162b8d9ae724191073baca22237449359ad6f18759",
  "file": "ros-humble-grid-map-core_2.0.1-1jammy.20260304.202024_amd64.deb"
 },
 {
  "package": "ros-humble-grid-map-cv",
  "version": "2.0.1-1jammy.20260907.224219",
  "architecture": "amd64",
  "sha256": "fb876a17873b5bca44376cd6021096554388a02eb3e51706897ae8446a989cc0",
  "file": "ros-humble-grid-map-cv_2.0.1-1jammy.20260907.224219_amd64.deb"
 },
 {
  "package": "ros-humble-grid-map-msgs",
  "version": "2.0.1-1jammy.20260907.223300",
  "architecture": "amd64",
  "sha256": "101e3da1f6af37fbdabe087a6a44f5907765f78ae783695a85d72f8bcfdaf8c9",
  "file": "ros-humble-grid-map-msgs_2.0.1-1jammy.20260907.223300_amd64.deb"
 },
 {
  "package": "ros-humble-grid-map-ros",
  "version": "2.0.1-1jammy.20260915.211053",
  "architecture": "amd64",
  "sha256": "6e17270015b810e3510be82d0f66df6f90baa72dd460837086f87460ed62b44b",
  "file": "ros-humble-grid-map-ros_2.0.1-1jammy.20260915.211053_amd64.deb"
 },
 {
  "package": "ros-humble-gtsam",
  "version": "4.2.0-3jammy.20260226.005743",
  "architecture": "amd64",
  "sha256": "e075d667c1030a3ca03e102d86ddada723486d0eae69aaf37f84cac027951533",
  "file": "ros-humble-gtsam_4.2.0-3jammy.20260226.005743_amd64.deb"
 },
 {
  "package": "ros-humble-libg2o",
  "version": "2020.5.29-4jammy.20260304.090502",
  "architecture": "amd64",
  "sha256": "5c42c64cfb88677360540cf7845dc7dc8b6ec0edbab925985d6847aad05a003a",
  "file": "ros-humble-libg2o_2020.5.29-4jammy.20260304.090502_amd64.deb"
 },
 {
  "package": "ros-humble-libnabo",
  "version": "1.0.7-3jammy.20260226.011207",
  "architecture": "amd64",
  "sha256": "64c397c7510adade40ec676413554b68eecc0b5554264003ed62df5c5da3deb4",
  "file": "ros-humble-libnabo_1.0.7-3jammy.20260226.011207_amd64.deb"
 },
 {
  "package": "ros-humble-libpointmatcher",
  "version": "1.3.1-3jammy.20260226.012136",
  "architecture": "amd64",
  "sha256": "a4169ad708bfc7319aae606ca585c5e132c51de01e2e09eb0f0911d26d3c1367",
  "file": "ros-humble-libpointmatcher_1.3.1-3jammy.20260226.012136_amd64.deb"
 },
 {
  "package": "ros-humble-nav2-msgs",
  "version": "1.1.20-1jammy.20260907.223327",
  "architecture": "amd64",
  "sha256": "7ea288dd8e6718cb46658616eceb667bfda53c0d2b08596e3bc8a70061dc3487",
  "file": "ros-humble-nav2-msgs_1.1.20-1jammy.20260907.223327_amd64.deb"
 },
 {
  "package": "ros-humble-pcl-ros",
  "version": "2.4.5-2jammy.20260915.212316",
  "architecture": "amd64",
  "sha256": "70ccedd81684ccdb42d6b517dc3c06e6c3a23dac1c1a74ec4ffcdd7636c09b1d",
  "file": "ros-humble-pcl-ros_2.4.5-2jammy.20260915.212316_amd64.deb"
 },
 {
  "package": "ros-humble-rtabmap-conversions",
  "version": "0.23.13-1jammy.20261002.122822",
  "architecture": "amd64",
  "sha256": "8f52946f6992e885c5c1ece830fb34634e3926e6a25d987743d3b69a387d6277",
  "file": "ros-humble-rtabmap-conversions_0.23.13-1jammy.20261002.122822_amd64.deb"
 },
 {
  "package": "ros-humble-rtabmap-msgs",
  "version": "0.23.13-1jammy.20261002.103800",
  "architecture": "amd64",
  "sha256": "1b2f0311d816d4d8e18af3073359066b24a2785d12b585b685cf950b186dd48d",
  "file": "ros-humble-rtabmap-msgs_0.23.13-1jammy.20261002.103800_amd64.deb"
 },
 {
  "package": "ros-humble-rtabmap-slam",
  "version": "0.23.13-1jammy.20261011.111917",
  "architecture": "amd64",
  "sha256": "4a2f04592d993360080875a4dbcb963baae68fc22b7625fd5ac4ee0c8e4816ac",
  "file": "ros-humble-rtabmap-slam_0.23.13-1jammy.20261011.111917_amd64.deb"
 },
 {
  "package": "ros-humble-rtabmap-sync",
  "version": "0.23.13-1jammy.20261002.123912",
  "architecture": "amd64",
  "sha256": "d386eb409470356ec7c4a80d00d13b1c74ad2ebdc78f69a45baf6fcd525fd0fe",
  "file": "ros-humble-rtabmap-sync_0.23.13-1jammy.20261002.123912_amd64.deb"
 },
 {
  "package": "ros-humble-rtabmap-util",
  "version": "0.23.13-1jammy.20261002.133003",
  "architecture": "amd64",
  "sha256": "140955aaccb2c3238d681ab752266e67823b957a8e85116f051c015380a91d84",
  "file": "ros-humble-rtabmap-util_0.23.13-1jammy.20261002.133003_amd64.deb"
 },
 {
  "package": "ros-humble-rtabmap",
  "version": "0.23.13-1jammy.20261002.103658",
  "architecture": "amd64",
  "sha256": "efdf8843d6780e73612e131cdab1e6db16446893b1652e4afb6dbca93a37ae5b",
  "file": "ros-humble-rtabmap_0.23.13-1jammy.20261002.103658_amd64.deb"
 }
]
RTABMAP_DEB_MANIFEST

die() { echo "ERROR: $*" >&2; exit 1; }
info() { echo "[install_rtabmap] $*"; }

# 固定 deb 清单与库路径均为 amd64（x86_64 Ubuntu 22.04 笔记本）。
# AGX（aarch64）不支持本地前缀模式，显式拒绝，避免用 x86_64 deb 破坏 ARM 环境。
HOST_ARCH="$(dpkg --print-architecture)"
[ "$HOST_ARCH" = "amd64" ] || die "本脚本的固定 deb/库路径只支持 amd64，本机 dpkg 架构为 $HOST_ARCH（aarch64/AGX 请另建 ARM64 构建方案）"

manifest_tsv() {
  printf "%s" "${MANIFEST_JSON}" | /usr/bin/python3 -c 'import json,sys; [print(item["package"]+"|"+item["version"]+"|"+item["sha256"]+"|"+item["file"]) for item in json.load(sys.stdin)]'
}

apt_package_specs() {
  printf "%s" "${MANIFEST_JSON}" | /usr/bin/python3 -c 'import json,sys; [print(item["package"]+"="+item["version"]) for item in json.load(sys.stdin)]'
}

write_env_file() {
  cat > "${PREFIX}/env.sh" <<'ENVEOF'
# Local (non-root) RTAB-Map Humble 0.23.13 overlay for the HEI MID-360S SLAM task.
# Generated by extensions/tools/install_rtabmap_laptop.sh
# Source AFTER /opt/ros/humble/setup.bash and the FAST-LIO underlay local_setup.bash.
_rtabmap_local_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RTABMAP_LOCAL_PREFIX="$_rtabmap_local_prefix"
export AMENT_PREFIX_PATH="$_rtabmap_local_prefix/prefix/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$_rtabmap_local_prefix/prefix/opt/ros/humble${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$_rtabmap_local_prefix/prefix/opt/ros/humble/lib/x86_64-linux-gnu:$_rtabmap_local_prefix/prefix/opt/ros/humble/lib:$_rtabmap_local_prefix/prefix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# ROS 2 Humble deb 的 Python 消息布局有两个目录：普通包在 site-packages，
# rtabmap_msgs 等 deb 实际装在 local/lib/python3.10/dist-packages。两个都必须有，
# 否则 from rtabmap_msgs.msg import MapGraph 会 ModuleNotFoundError（本轮已实测）。
export PYTHONPATH="$_rtabmap_local_prefix/prefix/opt/ros/humble/local/lib/python3.10/dist-packages:$_rtabmap_local_prefix/prefix/opt/ros/humble/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$_rtabmap_local_prefix/prefix/opt/ros/humble/bin:${PATH:-}"
export PKG_CONFIG_PATH="$_rtabmap_local_prefix/prefix/usr/lib/x86_64-linux-gnu/pkgconfig:$_rtabmap_local_prefix/prefix/usr/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export RTABMAP_VERSION="0.23.13"
ENVEOF
  chmod 644 "${PREFIX}/env.sh"
}

write_install_manifest() {
  local mode="$1"
  printf "%s" "${MANIFEST_JSON}" > "${PREFIX}/deb_manifest.json"
  /usr/bin/python3 - "${PREFIX}" "$mode" <<'PYEOF'
import json, sys, time
from pathlib import Path
prefix = Path(sys.argv[1])
mode = sys.argv[2]
manifest = json.loads((prefix / "deb_manifest.json").read_text(encoding="utf-8"))
payload = {
    "kind": "rtabmap_local_prefix_install",
    "rtabmap_version": "0.23.13",
    "ros_distro": "humble",
    "installed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    "mode": mode,
    "package_count": len(manifest),
    "packages": [{"package": item["package"], "version": item["version"],
                  "architecture": item["architecture"], "sha256": item["sha256"]} for item in manifest],
}
(prefix / "install_manifest.json").write_text(json.dumps(payload, ensure_ascii=False, indent=1) + chr(10), encoding="utf-8")
print("manifest:", prefix / "install_manifest.json")
PYEOF
}

download_and_verify_debs() {
  mkdir -p "${CACHE_DIR}"
  local package version sha256 filename deb actual
  while IFS="|" read -r package version sha256 filename; do
    deb="${CACHE_DIR}/$filename"
    if [ -f "$deb" ]; then
      actual="$(sha256sum "$deb" | cut -d" " -f1)"
      if [ "$actual" = "$sha256" ]; then
        info "cached OK: $filename"
        continue
      fi
      info "cache SHA256 不匹配，重新下载: $filename"
      rm -f "$deb"
    fi
    info "download: $package=$version"
    ( cd "${CACHE_DIR}" && LC_ALL=C apt-get download "$package=$version" ) || die "apt-get download 失败：$package=$version（检查网络/apt 源；本脚本不修改源配置）"
    [ -f "$deb" ] || die "下载后找不到 $deb"
    actual="$(sha256sum "$deb" | cut -d" " -f1)"
    [ "$actual" = "$sha256" ] || die "SHA256 不匹配：$deb"
  done < <(manifest_tsv)
}

unpack_debs() {
  # 安全：不覆盖已存在的前缀（可能正被运行中的 SLAM 依赖）。需要重装时先人工归档。
  if [ -e "${UNPACK_DIR}" ] && [ -n "$(ls -A "${UNPACK_DIR}" 2>/dev/null)" ]; then
    die "解包目录已存在且非空：${UNPACK_DIR}。如只想核验请用 --verify；如要重装请先手工归档/移走该目录（脚本不会 rm -rf 现有运行依赖）。"
  fi
  mkdir -p "${UNPACK_DIR}"
  local package version sha256 filename deb actual
  # 只解包 manifest 指定且逐个 SHA256 校验过的 deb，不遍历缓存目录（避免旧包/多余包覆盖）。
  while IFS="|" read -r package version sha256 filename; do
    deb="${CACHE_DIR}/$filename"
    [ -f "$deb" ] || die "缺少 deb 文件：$deb"
    actual="$(sha256sum "$deb" | cut -d" " -f1)"
    [ "$actual" = "$sha256" ] || die "SHA256 不匹配：$deb"
    dpkg-deb -x "$deb" "${UNPACK_DIR}/"
  done < <(manifest_tsv)
  info "已解包 ${UNPACK_DIR}（仅 manifest 指定的 36 个已校验 deb）"
}

verify_prefix() {
  [ -d "${UNPACK_DIR}/opt/ros/humble" ] || die "本地前缀不存在：${UNPACK_DIR}（先运行安装）"
  [ -f "${PREFIX}/env.sh" ] || die "缺少 env.sh：${PREFIX}/env.sh"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${PREFIX}/env.sh"
  set -u
  local pkg_prefix missing_node missing_core export_help
  pkg_prefix="$(ros2 pkg prefix rtabmap_slam 2>/dev/null)" || die "ros2 pkg prefix rtabmap_slam 失败"
  info "rtabmap_slam prefix: $pkg_prefix"
  /usr/bin/python3 -c 'from rtabmap_msgs.msg import MapGraph, Info; from rtabmap_msgs.srv import GetMap; print("rtabmap_msgs python imports OK (MapGraph/Info/GetMap)")' || die "rtabmap_msgs Python 导入失败：检查 env.sh 的 PYTHONPATH（必须包含 local/lib/python3.10/dist-packages）"
  export_help="$(rtabmap-export --help 2>&1 || true)"
  echo "$export_help" | grep -q -- "--cloud" || die "rtabmap-export 不可执行（--help 无 --cloud）"
  echo "$export_help" | grep -q -- "--poses_format" || die "rtabmap-export 缺 --poses_format"
  info "rtabmap-export OK（--cloud/--scan/--poses/--poses_format/--opt 接口）"
  missing_node="$(ldd "${UNPACK_DIR}/opt/ros/humble/lib/rtabmap_slam/rtabmap" | grep -c "not found" || true)"
  missing_core="$(ldd "${UNPACK_DIR}/opt/ros/humble/lib/x86_64-linux-gnu/librtabmap_core.so" | grep -c "not found" || true)"
  [ "$missing_node" = "0" ] || die "rtabmap 节点存在 $missing_node 个未解析动态库"
  [ "$missing_core" = "0" ] || die "librtabmap_core 存在 $missing_core 个未解析动态库"
  info "ldd OK: 无未解析动态库"
  info "verify PASS（rtabmap ${RTABMAP_VERSION}，36 个官方 deb 固定版本）"
}

apt_mode() {
  [ -f /opt/ros/humble/setup.bash ] || die "缺少 ROS 2 Humble（/opt/ros/humble/setup.bash）"
  if ! sudo -n true 2>/dev/null; then
    echo "当前会话 sudo -n 不可用（需要密码）。可选：" >&2
    echo "  1) 在可交互终端手动执行官方 apt 安装（先 dry-run，再安装）：" >&2
    echo "" >&2
    echo "     sudo apt-get install -s --no-install-recommends \\" >&2
    apt_package_specs | sed "s/^/         /" >&2
    echo "" >&2
    echo "  2) 或使用无需 root 的本地前缀模式：" >&2
    echo "     ${BASH_SOURCE[0]}" >&2
    die "sudo -n 不可用，未修改系统"
  fi
  mkdir -p "${PREFIX}"
  mapfile -t PKGS < <(apt_package_specs)
  info "apt dry-run 记录到 ${PREFIX}/apt_dry_run.log"
  sudo -n apt-get install -s --no-install-recommends "${PKGS[@]}" | tee "${PREFIX}/apt_dry_run.log" >/dev/null
  info "开始 apt 安装（仅这些固定版本包，不升级无关包）"
  sudo -n apt-get install -y --no-install-recommends "${PKGS[@]}" | tee "${PREFIX}/apt_install.log" >/dev/null
  info "apt 安装完成：ROS 2 系统前缀 /opt/ros/humble 已含 rtabmap 0.23.13"
  info "系统级安装无需 env.sh；直接 source /opt/ros/humble/setup.bash 即可"
}

case "${1:---local-prefix}" in
  --apt) apt_mode ;;
  --verify) verify_prefix ;;
  --local-prefix)
    download_and_verify_debs
    unpack_debs
    write_env_file
    write_install_manifest "local-prefix"
    verify_prefix
    info "完成：source ${PREFIX}/env.sh"
    ;;
  -h|--help) sed -n "2,30p" "${BASH_SOURCE[0]}"; exit 0 ;;
  *) die "未知参数：${1:-<none>}（支持 --local-prefix | --verify | --apt）" ;;
esac