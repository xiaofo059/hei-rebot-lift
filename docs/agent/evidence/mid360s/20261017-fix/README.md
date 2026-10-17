# 2026-10-17 MID-360S 源时间 / 驱动退出崩溃诊断与最小修复（证据包）

任务：HEI-LIDAR-20261017-02-FIX。本目录只放证据与复现说明；原始 bag、
第三方源码、用户文件均不在仓库内复制。

## 0. 结论（一句话）

- **driver 退出 SIGBUS/SIGABRT 已定位因果**：SDK 内置 spdlog 1.3.1 的默认可见性弱符号
  被 ROS 2 日志后端载入的系统 `libspdlog.so.1`(1.9.2) 抢占，SDK 全局
  `shared_ptr<spdlog::logger>` 在 exit handler 用 1.9.2 的 `_M_dispose` 析构 1.3.1 布局对象。
  已用「只导出公开 C API」的符号版本脚本修复，并用无硬件 A/B 探针验证（修复前 SIGBUS=135，修复后 0）。
- **LiDAR 源时间异常不是网络/传感器时间基准问题**：2026-10-16 18:33 附近主机
  `CLOCK_REALTIME` 回拨约 598 ms；MID-360S 两端口本次为 NoSync(`time_type=0`)，driver 用
  `high_resolution_clock`(=CLOCK_REALTIME) 生成 LiDAR/IMU header 并用同钟做非同步帧调度，
  于是 header 回退、发布调度停摆约 0.70 s，堆积出 140,736 点巨帧。
- **原 QC FAIL 保留**：真实记录顺序下 LiDAR 仍有 -504.288 ms 的帧基准回退与 +713.432 ms 的
  真实前跳；但按 bag 时间戳排序读取会额外制造 1 次回退/1 个 gap，并把 IMU 的 -598.307 ms
  真实回拨**掩盖**成 0 次——排序读取不可当作原始顺序。
- 修复只作用于 NoSync 路径：一次 `system_clock` epoch 锚点 + `steady_clock` elapsed 生成单调
  epoch ns（LiDAR/IMU 共用），帧调度改用 `steady_clock`；PTP/GPS 路径保持原样。
  **该锚点不是硬件时间同步**。

## 1. 目录内容

| 文件 | 内容 |
| --- | --- |
| `timestamp_forensic.md` | 正式 bag 的 insertion-id vs timestamp-sorted 取证（命令、SQL、原始数值） |
| `coredump_344190_sigbus.txt` | 正式录包后 SIGBUS core 的 gdb 栈（系统 libspdlog frame） |
| `symbol_isolation.md` | 符号/ODR 证据、A/B 探针结果、修复后符号表 |
| `underlay_regression.log` | `extensions/tools/tests/run_underlay_regression.sh` 完整输出 |
| `build_after_revert.log` | 先 `--revert-patches` 再全量重建（干净 SHA -> apply patch -> build）日志 |
| `build_repeat.log` | 已应用 patch 状态下重复构建（幂等）日志 |
| `patch_sha256.txt` | tracked patch 的 sha256 |

## 2. 复现命令（无硬件）

```bash
# 构建（自动 apply / 校验 tracked patch，构建后校验符号隔离与 driver patch）
extensions/tools/build_mid360s_underlay.sh
# 撤销 patch，恢复冻结 SHA 工作区（不构建）
extensions/tools/build_mid360s_underlay.sh --revert-patches
# 无硬件回归：patch 状态机 + host_time 映射 + SDK 符号隔离 + 退出崩溃 A/B
LIVOX_ORIGINAL_SDK=<original_sdk.so> extensions/tools/tests/run_underlay_regression.sh
```

## 3. 修复范围与未验证边界

修复文件（均未 commit）：

- `extensions/dependencies/patches/livox_sdk2/0001-export-only-public-c-api.patch`
- `extensions/dependencies/patches/livox_ros_driver2/0001-monotonic-host-clock.patch`
- `extensions/tools/build_mid360s_underlay.sh`（patch 状态机 + 构建后验收）
- `extensions/tools/lib/patch_state.sh`、`extensions/tools/tests/*`（可重放测试）

**未验证 / 交给 Codex 硬件实验**：

- 真实驱动正常 SIGINT 退出多次（需要雷达与唯一 driver 占用）；本机未启动 driver。
- 真实 31 分钟包在修复后驱动的重新录制与完整 QC（本任务不启动采集）。
- host 时钟回拨的根因（timesyncd 昨 18:30-18:35 日志已不可用）；本修复不依赖根因，
  也不再受 CLOCK_REALTIME 步进影响。
- time_type 运行期切换（NoSync↔PTP/GPS）边界：timestamp 域会一次性跳变，QC 应如实报出；
  本次被动抓包确认两端口均为 NoSync。
- 设备原始计时（raw_timestamp）映射未引入：无重启/回绕证据前不做新映射。

## 4. 引用类外部证据（不复制）

- Codex 无硬件探针与原始库：`/home/robo/hei-data/mid360s_retest_20261017_evidence/`
  （`probe_sdk_loader.cpp`、`original_sdk.so`、`original_sdk_loader_probe.json`）。
- Codex 被动 UDP 采样：同目录 `passive_udp_both.json`（56301/56401 均 `time_type=0`）。
- 正式只读 bag：`/home/robo/hei-data/mid360s_live_20261016_1827/`（未复制、未改动）。
