# 证据索引：MID-360S 三维 SLAM（FAST-LIO2 + RTAB-Map 0.23.13）电脑端接入

任务：HEI-LIDAR-04-SLAM-LAPTOP-BASE-20261019；分支 `feat/mid360s-lio`，HEAD `1aeb8a1`。
本目录只放小型清单/日志/JSON/测试结果，**不放大地图、PLY、原始 bag**；大产物在
`/home/robo/hei-data/mid360s_slam_20261019/`。TASK_BOARD/EXECUTION_LOG 由 Codex 单写，本目录不含其改动。

## 文件说明

| 路径 | 内容 |
| --- | --- |
| `00-prestate/` | 本轮开始前 git 状态/HEAD/分支与 75 个 dirty+untracked 文件 SHA256 快照（保护旧 LIO 工作） |
| `01-freeze/` | 首次冻结：源码/安装文件逐项 SHA256、entry points、`env.sh` PYTHONPATH 缺陷修复与导入证据、纯单测日志 |
| `02-codex-full-run/` | Codex 旧参数完整回放证据的**只读引用**（路径+SHA256，不复制/不修改）；本机对 Codex DB 的只读 SQLite 检查；DB 只读备份 SHA256 |
| `02b-new-config-run/` | **新参数（ProximityOdomGuess=true）完整 2x 回放**：本机只读 SQLite 复核（55 节点/55 扫描/type0=108/type2=50=25 独立）、probe 摘要、launch_info、Codex 接管监督与 DB 审查的只读引用（路径+SHA256） |
| `03-p2-build-test/` | 最终构建日志、pytest（纯单测 + 真实节点启动，87 passed）、运行文件哈希、防覆盖/隔离拒绝负向证据、取消-接管的操作例外 |
| `04-export/` | 严格导出结果 JSON、命令与日志、PLY/poses SHA256、pose 文本；`new_final_config/` 为新最终库（48,855 顶点 + 55 poses），`export_result.json/export_final.json` 等为旧库历史对照 |
| `05-localization/` | 只读定位 harness/log/launch_info/probe、DB SHA256 前后、`/initialpose` 采集、`/info` collector 原始 JSON、修复前/后实验日志、预创建目录拒绝证据 |
| `06-installer/` | `install_rtabmap_laptop.sh --verify` 原始输出（Codex 另有独立 verify 与外部前缀安装 PASS） |
| `07-git-scope/` | 最终 git status/HEAD/分支；与 `00-prestate` 的 dirty 文件哈希对比；原 dirty/untracked LIO 文件保护报告（69 项未变） |
| `evidence_sha256_manifest.txt` | 本目录全部证据文件 SHA256 清单（不含本清单自身） |

## 数据目录（大产物，位于 /home/robo/hei-data/mid360s_slam_20261019/）

| 路径 | 说明 |
| --- | --- |
| `codex_full_observer/`、`codex_full_map/` | Codex 旧参数完整 2x 回放结果与 DB（Codex 持有，本任务只读引用） |
| `mapping_proxguess_20261019_121802/` | **新最终建图**：DB `b30fbaec…`（55 节点/55 扫描/25 独立 type2）、probe、launch_info；收尾由 Codex 接管（见下） |
| `export_optimized_final_proxguess_20261019_122137/` | **新最终导出**：PLY 48,855 顶点（`60f7e5e6…`）+ 55 行轨迹（`663f3274…`） |
| `export_optimized_20261019_120641.final/` | 旧库历史导出：48,986 顶点 + 52 行轨迹（仅旧配置对照） |
| `export_optimized_20261019_120436/` | 首次导出（未加 `--voxel`，cloud 为空）的失败证据，保留说明 0.23.13 CLI 约束 |
| `localization_readonly_20261019_120903/` | 旧配置只读定位运行目录（probe/launch_info） |
| `loc_experiment_proxguess_*` | `ProximityOdomGuess=true` 对照实验运行目录 |
| `_db_backups/` | Codex 建图 DB 的只读备份副本（SHA256 与原件核对） |

## 关键数字（与 docs/MID360S_SLAM.md 一致）

* 旧参数完整回放：2335 lidar / 46708 IMU 全收；2331 odom；2331 帧 body 云 8,680,103 点（XYZI 哈希 2331/2331 一致）；
  52 节点 / 52 扫描；102 条有向 neighbor + 28 条有向 type2（=14 条独立自动几何约束）；DB SHA256 `afab12b8…`；
* 新参数完整建图：55 节点 / 55 扫描；108 有向 neighbor + 50 有向 type2（=25 条独立自动几何约束）；视觉 WARN 0；DB SHA256 `b30fbaec…`；
* 新最终库导出：PLY 48,855 顶点（binary LE，body 完整、有限 XYZ 0）、poses 55 行 9 列全有限；DB SHA256 前后一致；
* 旧库历史导出：PLY 48,986 顶点 + 52 poses（仅旧配置对照，不作为新库结果）；
* 定位修复前：15 条 `Missing visual features` WARN；修复后（`RGBD/ProximityOdomGuess=true`）：0 条；
  `/info` 36 条全部 ref_id>0、9 条 proximity_detection_id>0（map 节点 1,29,31,33,36,38,40,44,47）；
  初始位姿发布前 0 次接受、发布后从 map 节点 1 开始连续接受；
* 全部离线运行 Domain 89 + localhost only；退出清理无残留；DB 只读模式 SHA256 不变。

## 诚实性/操作例外

1. 04-SLAM 仍 PARTIAL：无 30-50 m 闭环、无 3 位置重定位精度、无现场外参标定、无实机/AGX 验收；
2. 新参数完整建图的 harness 曾被外部 job 取消，launch 及子进程由 Codex 接管监督并按单次 SIGINT + 有界回收收尾；
   本轮不把该次运行陈述为原 harness clean，结果以 Codex 回显与原始 log/DB 为准；
3. 早期临时探针在 `/tmp` 一次性临时库用过 `delete_db_on_start=true`，随后移除；最终交付零自动删库参数；
4. 预创建 run 目录/未设置隔离域等负向案例都作为**护栏通过**证据，而不是功能失败；
5. 一次性 6 s 事后 `/info` 旁观 0 条不作为定位证据；定位结论只采用运行中 collector 原始数据 + 源码核对。

## 复现与核验入口

```bash
cd /home/robo/robotopia/hei-rebot-lift
extensions/tools/install_rtabmap_laptop.sh --verify
cd extensions/ros2_ws/src/hei_robot_slam && python3 -m pytest test/ -q
ros2 run hei_robot_slam slam_db_tools inspect /home/robo/hei-data/mid360s_slam_20261019/codex_full_map/rtabmap_mapping.db
```

完整命令、契约与限制见 `docs/MID360S_SLAM.md` 与本目录各子目录说明。