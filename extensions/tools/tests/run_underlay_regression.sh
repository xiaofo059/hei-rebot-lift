#!/usr/bin/env bash
# HEI-LIDAR-20261017-02-FIX: underlay regression suite (no hardware needed).
#
#  1. bash syntax of build script / patch-state lib / tests
#  2. tracked-patch state machine regression (temporary git repos only)
#  3. host_time.h synthetic mapping test against the patched driver header
#  3b. process-local CLOCK_REALTIME step injection (SYNTHETIC; OS clock untouched)
#  4. built SDK symbol isolation (no spdlog/fmt/livox:: exports, C API complete)
#  5. no-hardware exit-crash A/B probe:
#       original SDK + system libspdlog preloaded -> SIGBUS (reproduce)
#       patched  SDK + system libspdlog preloaded -> exit 0  (fix)
#     The original SDK path is optional: set LIVOX_ORIGINAL_SDK to compare.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
TP="${REPO_ROOT}/extensions/third_party"
BUILD_SCRIPT="${REPO_ROOT}/extensions/tools/build_mid360s_underlay.sh"
PATCH_LIB="${REPO_ROOT}/extensions/tools/lib/patch_state.sh"
DRIVER_DIR="${TP}/livox_ws/src/livox_ros_driver2"
SDK_LIB="${TP}/livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so"
API_LIST="${REPO_ROOT}/extensions/dependencies/patches/livox_sdk2/public_c_api.txt"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT
FAILED=0

check() {  # $1=desc, rest=command
  local desc="$1"; shift
  if "$@"; then echo "PASS: ${desc}"; else echo "FAIL: ${desc}" >&2; FAILED=1; fi
}

echo "== 1. bash -n =="
for f in "${BUILD_SCRIPT}" "${PATCH_LIB}" "${SCRIPT_DIR}/test_patch_state_machine.sh" "${BASH_SOURCE[0]}"; do
  check "bash -n $(basename "${f}")" bash -n "${f}"
done

echo
echo "== 2. tracked-patch state machine =="
check "patch state machine regression" "${SCRIPT_DIR}/test_patch_state_machine.sh"

echo
echo "== 3. host_time.h mapping test =="
if [ -f "${DRIVER_DIR}/src/comm/host_time.h" ]; then
  if g++ -std=c++11 -O2 -Wall -Wextra -I "${DRIVER_DIR}/src/comm" \
      "${SCRIPT_DIR}/test_host_time_mapping.cpp" -o "${TMP}/test_host_time_mapping"; then
    check "host time mapping (first call >= anchor, monotonic)" "${TMP}/test_host_time_mapping"
  else
    echo "FAIL: host_time.h 编译失败" >&2; FAILED=1
  fi
else
  echo "FAIL: 缺少 ${DRIVER_DIR}/src/comm/host_time.h（driver patch 未应用）" >&2; FAILED=1
fi

echo
echo "== 3b. process-local CLOCK_REALTIME step injection (SYNTHETIC) =="
if [ -f "${DRIVER_DIR}/src/comm/host_time.h" ]; then
  if g++ -std=c++11 -O2 -Wall -Wextra -I "${DRIVER_DIR}/src/comm" \
      "${SCRIPT_DIR}/test_process_clock_steps.cpp" -o "${TMP}/test_process_clock_steps" -pthread; then
    check "process-local clock step injection (fault observed, mapping stable)" \
      "${TMP}/test_process_clock_steps"
  else
    echo "FAIL: test_process_clock_steps.cpp 编译失败" >&2; FAILED=1
  fi
else
  echo "FAIL: 缺少 ${DRIVER_DIR}/src/comm/host_time.h（driver patch 未应用）" >&2; FAILED=1
fi

echo
echo "== 4. SDK symbol isolation =="
if [ -f "${SDK_LIB}" ]; then
  leaked="$(nm -D --defined-only "${SDK_LIB}" | grep -E 'spdlog|_ZN3fmt|_ZN5livox5lidar' || true)"
  if [ -n "${leaked}" ]; then
    echo "FAIL: SDK 仍导出内部符号:" >&2; printf '%s\n' "${leaked}" | head -5 >&2; FAILED=1
  else
    echo "PASS: SDK 未导出 spdlog/fmt/livox:: 内部符号"
  fi
  exported="$(nm -D --defined-only "${SDK_LIB}" | awk '{print $3}')"
  missing=0
  while IFS= read -r sym; do
    [ -n "${sym}" ] || continue
    grep -qx "${sym}" <<< "${exported}" || { echo "FAIL: 缺少 C API ${sym}" >&2; missing=1; }
  done < "${API_LIST}"
  if [ "${missing}" = "0" ]; then echo "PASS: 公开 C API 导出完整"; else FAILED=1; fi
else
  echo "FAIL: 缺少 ${SDK_LIB}（先运行 build_mid360s_underlay.sh）" >&2; FAILED=1
fi

echo
echo "== 5. exit-crash A/B probe (no hardware) =="
if [ -f "${SDK_LIB}" ] && g++ -O2 -o "${TMP}/probe_sdk_loader" "${SCRIPT_DIR}/probe_sdk_loader.cpp" -ldl; then
  probe() { ( ulimit -c 0; "${TMP}/probe_sdk_loader" "$@" >/dev/null 2>&1 ); }
  probe "${SDK_LIB}"; rc=$?
  if [ "${rc}" = "0" ]; then echo "PASS: patched SDK alone -> exit 0"; else echo "FAIL: patched SDK alone -> ${rc}" >&2; FAILED=1; fi
  probe "${SDK_LIB}" --system-first; rc=$?
  if [ "${rc}" = "0" ]; then
    echo "PASS: patched SDK + system libspdlog -> exit 0（符号隔离生效）"
  else
    echo "FAIL: patched SDK + system libspdlog -> ${rc}（期望 0）" >&2; FAILED=1
  fi
  if [ -n "${LIVOX_ORIGINAL_SDK:-}" ] && [ -f "${LIVOX_ORIGINAL_SDK}" ]; then
    probe "${LIVOX_ORIGINAL_SDK}"; rc=$?
    if [ "${rc}" = "0" ]; then echo "PASS: original SDK alone -> exit 0"; else echo "FAIL: original SDK alone -> ${rc}" >&2; FAILED=1; fi
    probe "${LIVOX_ORIGINAL_SDK}" --system-first; rc=$?
    if [ "${rc}" = "135" ]; then
      echo "PASS: original SDK + system libspdlog -> SIGBUS(135)（复现缺陷）"
    else
      echo "FAIL: original SDK + system libspdlog -> ${rc}（期望 135=SIGBUS）" >&2; FAILED=1
    fi
  else
    echo "SKIP: 未提供 LIVOX_ORIGINAL_SDK，跳过缺陷复现对照（不影响修复验收）"
  fi
else
  echo "FAIL: 无法编译/定位 probe" >&2; FAILED=1
fi

echo
if [ "${FAILED}" = "0" ]; then
  echo "ALL UNDERLAY REGRESSION CHECKS PASSED"
  exit 0
fi
echo "UNDERLAY REGRESSION FAILURES PRESENT" >&2
exit 1
