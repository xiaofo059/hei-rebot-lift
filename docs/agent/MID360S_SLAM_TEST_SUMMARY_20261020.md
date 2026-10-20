# MID-360S 三维 SLAM 电脑端功能测试总结（2026-10-20）

> 文档性质：Codex 验收前冻结的交接总结；本文件是本次任务唯一新增文件。生成时间：**2026-10-20T16:34:37+08:00（实际本机时间，CST）**。
> 修订记录：2026-10-20T16:36:25+08:00 按文档 Review 修正 §1 授权表述、§10.4 监测时序、§11.2 后续建议；仅文档修订，不改实现/证据。
> 分支 `codex/slam-mapping-relocalization-test`，HEAD `fbe160bb5a4a9cdb36bc1ab069ed72d9cc9a2e42`（未提交、未 push、未切分支）。
> 当前环境：笔记本 x86_64 / Ubuntu 22.04.5 LTS / ROS 2 Humble；测试已停，Codex 已关闭现场节点并核验无残留；本总结未启动任何节点/驱动/RViz，未 build，未跑算法。
> 地图数据不进 Git：`map/.gitignore` 以 `*` 排除地图/DB/JSONL 等本地产物；大产物在 `/home/robo/hei-data/`。

## 0. 一页结论

- **本轮通过（功能范围）**：电脑端小范围三维建图、地图保存/加载、有粗初值重定位、运动时配准。用户现场目视认可；Codex 独立日志/JSON 复核了关键接受事件、时间、地图 SHA 与干净关闭。
- **明确失败边界**：180 度偏航、未发 `/initialpose` 的自动起始尝试 82 帧候选、0 次接受，用户同时报告可见扫描-地图错位；该场景不通过。
- **未验收**：30–50 m 闭环度量、3 位置重复率、外参现场标定、真实 AGX/底盘/base_link/导航；`accepted` 只是后端事件证据，不是绝对精度。
- **里程碑不变**：04-SLAM 总体仍 **PARTIAL**；`HEI-LIDAR-04` 仅“电脑端软件范围” DONE；`HEI-SLAM-OBS-20261020` 仅“连续取证软件范围” DONE。本文不把总里程碑写成全部 DONE。
- **下一门槛**：见 §11；接手先读 §1 顺序，不要直接启动节点或占用 Domain 89。

## 1. 新会话接手顺序（按序读，不复制命令真相）

1. [README_FOR_AGENTS.md](../../README_FOR_AGENTS.md)：项目索引、证据等级与当前交接状态。
2. 本总结：本轮状态、数字、失败边界与未结事项。
3. [docs/agent/TASK_BOARD.md](TASK_BOARD.md)、[docs/agent/EXECUTION_LOG.md](EXECUTION_LOG.md)：台账/执行记录（**Codex 单写**，本任务只读）。
4. [docs/MID360S_SLAM.md](../MID360S_SLAM.md)：架构、参数、模式与数据库保护、§8 已知限制、§10 复现入口；**复现命令以该文件和包 README 为准**。
5. [扩展包 README](../../extensions/ros2_ws/src/hei_robot_slam/README.md)：两种运行模式、frame/topic/时间契约、工具、测试、§定位连续取证。
6. 证据索引：[本轮取证证据](evidence/mid360s/20261020-localization-observability/README.md)、[上一轮 SLAM 证据](evidence/mid360s/20261019-slam/README.md)。

**接手后先做**：核对 `git status` / `git rev-parse HEAD` 与 §9 一致；确认无 ROS 算法/驱动进程。后续只读检查或运行按**用户当次任务范围**执行并保留证据，不默认扩大范围；**涉及硬件运动、急停/现场安全的动作必须专项授权**。未经当次任务明确授权，不得擅自启停现场算法/驱动/launch/RViz，也不得向 Domain 89 发布 `/initialpose`。

## 2. 背景、环境与架构

- 场景：笔记本现场实时建图与保存图重载定位；不是 AGX/实机导航。无机器人 `base_link`，无 AGX 安装，无底盘/导航/运动控制。
- 前端：FAST-LIO2 发布 `camera_init -> body` 局部 LIO；后端：RTAB-Map 0.23.13 纯激光 ICP（`RGBD/ProximityOdomGuess=true` 等既有参数），发布 `map -> camera_init`；适配输入为单帧 `body` 系 XYZI 云。
- 运行：Domain 89 + localhost 为现场算法链路；DSH 自动测试固定 Domain 91（拒绝 89），Codex 独立测试用 Domain 92。**测试隔离不等于现场验证**。
- 运行文件：本轮继续使用既有 launch/包；OBS 任务只读增加监测，未改算法、参数、TF/点云内容、驱动、地图或 launch 行为。

## 3. 工作时间线（时间为原始文件/JSON 记录，非事后估算）

| 时间（2026-10-20） | 事件 | 证据/说明 |
| --- | --- | --- |
| 10-19 全天 | 上一轮 HEI-LIDAR-04：电脑端 SLAM 软件部署/终审（历史基线：55 节点/55 扫描、25 独立 type2、87 tests、DB `b30fbaec…`） | 见 [20261019 证据](evidence/mid360s/20261019-slam/README.md)；04-SLAM 总体仍 PARTIAL |
| 14:19:03–14:21 | 第一次实景建图 `mapping_test_20261020_141902`：60 持久节点/60 扫描，46 条有向 type2（=23 独立） | `launch_info.started_at=14:19:03`；DB SHA `7b6b143b…`；inspection integrity ok |
| 14:21:15 / 14:44:01 / 14:48:24 | 以首图做三次只读定位尝试（launch_info 均指向首图） | 仅有旧 probe 报告与 `/info` 快照，未形成完整接受时间线；**只作过程记录，不作验收** |
| 14:53:16–14:55 | 第二张建图 `mapping_retest_20261020_145316`：40 持久节点/40 扫描，18 条有向 type2（=9 独立） | DB SHA `97df…`；inspection integrity ok |
| 14:56:14 起 | 后续定位/监测 run 全部以只读模式加载第二张图 | `20261020_145614`、`retest_…154950`、`yaw180_…155411` 的 `database_path`/`database_sha256_before` 均为第二张图 `97df…` |
| 15:22 | 诊断：18 个静止采样帧全 0、仅见候选；旧 probe 只数 frame，不订阅 `/info` | `map/20261020_145614/localization_observability_diagnosis.json`；该诊断触发连续取证修复 |
| 15:42:43–15:44:35 | Codex 只读 monitor 冒烟 `live_20261020_154242`：108 Info 帧、0 accepted、0 initialpose、325 事件、errors 0、干净关闭 | 当时尚未修正最终 metric；作冒烟记录 |
| 15:44:44–15:49:31 | 只读 monitor `live_20261020_154443`：初值偏差恢复证据所在 run（270 Info/6 accepted/833 事件，errors 0） | `live_bias_recovery_20261020_154708.json` |
| 15:47:08.967 | 收到纠正后的 `/initialpose`（seq 415） | 同一 run 事件时间线 |
| 15:47:11.048 | 后端接受：ref 3049 / 地图节点 75，初始位姿后 **2.0813 s**；map correction 转角 **36.902°**；TF 变化平移 0.807 m / 转角 0.644 rad | `live_bias_recovery_20261020_154708.json`；仅后端事件证据，精度未测 |
| 15:49:50（run 起）/15:49:54.805 | `retest_…154950` 自动起始：未发 `/initialpose` 首次接受 ref 152 / 地图 125；该 run 结束累计 12 accepted/231 Info、0 initialpose | `automatic_start_matching_review.json` 为首次接受快照；只证明该起始条件，不是 IMU 单独全局定位 |
| 15:54:12（launch）/15:55:41 | `yaw180_…155411` 无初值段：82 Info、82 candidate、**0 accepted**、0 initialpose；用户报告 180 度偏航与可见扫描-地图错位 | `yaw180_no_initialpose_review.json`；失败边界 |
| 15:56:17.531 / 15:56:18.806 | 人工粗初值（方向约 176°，由四元数换算）后 **1.2757 s** 接受 ref 268 / 地图 125；接受帧 review 记录 yaw≈173.0° | `yaw180_with_initialpose_review.json`、`motion_followup_review.json` accepted_frames[0] |
| 16:13:05 | 运动跟踪中途快照：76 accepted/1078 Info、tf_updates 22640；odom 11316 帧无时间戳回退/非有限、四元数范数≈1 | `motion_followup_review.json`（76 是中途快照） |
| 16:30:41.377 | 最终关停 review：final/closed=true、**81 accepted / 2076 Info / 2 initialpose / 2 rejected**、write_errors 0、数据库 SHA 未变、无算法残留 | `yaw180_…155411/shutdown_review.json`；**最终数量以该文件为准** |
| 16:34 | 仅本文档整理（只读），未启动任何节点、未跑测试 | 本文件 |

## 4. 分工

| 角色 | 本轮职责 | 边界/证据 |
| --- | --- | --- |
| 用户 | 现场操作与目视观察；报告 180° 偏航/扫描-地图错位、初值恢复与运动配准现象；认可本轮功能测试；授权并确认关闭节点 | 未提供数值 ground truth；目视反馈不等于精度验收 |
| Codex | 现场运行编排与只读 monitor；生成 review JSON/launch_info/备份与 DB SHA；独立完整测试与最终针对性测试；关闭节点并核验无残留 | `codex_acceptance.json`、各 run 的 review/shutdown JSON；独立测试 Domain 92 |
| DSH-LAPTOP（本执行端） | 上一轮交付 SLAM 软件/launch/包；本轮交付连续取证代码（probe/`localization_events.py`）、测试、README、证据目录；本次只读整理本总结 | 未在 Domain 89 启停/发布初值/占用；未改算法；未 commit/push |

## 5. 改动与交付（未提交）

| 类别 | 文件/目录 | 状态/说明 |
| --- | --- | --- |
| 实现 | `extensions/ros2_ws/src/hei_robot_slam/hei_robot_slam/localization_events.py` | 新增纯逻辑：事件分类、累计 summary、JSONL 独占 writer、后端日志只读 tail |
| 实现 | `.../hei_robot_slam/cloud_probe.py` | 最小接线：`/info`、`/initialpose`、`/rosout`、`/localization_pose`、`map->camera_init` TF 变化、backend log tail；保留旧 report 契约 |
| 实现 | `.../hei_robot_slam/package.xml` | 新增 `rcl_interfaces` 依赖 |
| 测试 | `.../test/test_localization_events.py`、`test_localization_observability_integration.py` | 新增纯逻辑与 Domain 91 synthetic 端到端；`test_node_startup.py` 测试域改默认 91（拒绝 89） |
| 文档 | `.../hei_robot_slam/README.md` | 新增“定位连续取证”说明 |
| 证据 | `docs/agent/evidence/mid360s/20261020-localization-observability/` | 未跟踪新目录：README、raw 测试日志、synthetic 样例、Codex acceptance、freeze 哈希等 |
| Codex 单写 | `docs/agent/TASK_BOARD.md`、`docs/agent/EXECUTION_LOG.md` | 本轮显示 modified，但由 Codex 维护；DSH 未写 |
| 本文件 | `docs/agent/MID360S_SLAM_TEST_SUMMARY_20261020.md` | 本次唯一新增文件；未 commit |

- 无算法/参数/TF/点云/驱动/地图/launch 内容修改；无 build（本轮文档任务）；无 commit/push/切分支。
- runtime 已按 OBS 任务冻结：22 项 source↔installed 哈希全一致，见 [runtime_frozen_hashes.json](evidence/mid360s/20261020-localization-observability/freeze/runtime_frozen_hashes.json)；`localization_events.py` 最终 SHA256 `46c8c13e…`（完整值见该 JSON）。

## 6. 已验证结果

### 6.1 建图、保存与加载

| 项 | 首图 | 第二轮图（实际加载图） |
| --- | --- | --- |
| 路径 | `extensions/ros2_ws/src/hei_robot_slam/map/mapping_test_20261020_141902/` | `extensions/ros2_ws/src/hei_robot_slam/map/mapping_retest_20261020_145316/` |
| 建图启动 | 2026-10-20T14:19:03+08:00 | 2026-10-20T14:53:16+08:00 |
| 持久节点 / 扫描 | 60 / 60 | 40 / 40 |
| 自动空间约束 | 46 条有向 type2 = **23 独立**（按正反向成对口径） | 18 条有向 type2 = **9 独立** |
| 其它链接 | neighbor 118；global loop 0；user manual closure 0 | neighbor 78；global loop 0；user manual closure 0 |
| DB SHA256 | `7b6b143b87e9bb4e96df551f230f01e545d09ac24a1b58b5f8669eab3379c444` | `97dfdf59973031152daf86742a031eb49d656cc62096d82dc72282b075f2facf` |
| 检查结果 | integrity ok；Data.scan 非空；保存退出正常 | integrity ok；Data.scan 非空；保存退出正常；后续定位前后 SHA 未变 |

- 两张图均成功保存；第二张图在 14:56 之后所有定位/监测 run 中作为只读加载图（见 §3）。
- **注意**：type2 是局部几何空间闭环，不等于全局任意位置重定位能力，也不等于地图/定位精度合格。

### 6.2 定位、初值恢复与运动跟踪

| 场景 | 结果 | 证据 | 边界 |
| --- | --- | --- | --- |
| 初值偏差后纠正 | 2.0813 s 接受 ref 3049 / 地图 75；map correction 转角 36.902°；TF 平移 0.807 m / 转角 0.644 rad | `live_bias_recovery_20261020_154708.json` | 仅后端接受事件；无精度 ground truth |
| 自动起始（未发 initialpose） | 15:49:54.805 首次接受 ref 152 / 地图 125；该 run 累计 12 accepted/231 Info/0 initialpose | `automatic_start_matching_review.json` 与 run summary | 只证明该起始条件自动定位成功；不是 IMU 单独全局定位，不能外推任意位置 |
| 180° 偏航、无初值 | 82 Info / 82 candidate / **0 accepted**；用户现场看到明显扫描-地图错位 | `yaw180_no_initialpose_review.json` | **失败边界明确**；不等于物理角度精确测量 |
| 180° 偏航、人工粗初值 | 1.2757 s 接受 ref 268 / 地图 125；接受帧 review yaw≈173.0°（人工初值方向约 176°） | `yaw180_with_initialpose_review.json`、`motion_followup_review.json` | 仍只是后端事件；精度/重复率未测 |
| 运动时配准/跟踪 | 最终 81 accepted / 2076 Info / 2 initialpose / 2 rejected；tf_updates 43726、tf_change 85；关停干净 | `shutdown_review.json`（16:30:41） | 不是 30–50 m 闭环度量；accepted 不等于精度 |

- 保存图重载链路可用：14:56/15:49/15:54 之后的 run 均加载第二张图且数据库 SHA 在定位前后不变。
- 中途快照 76 accepted 与最终 81 accepted 不矛盾：76 是 16:13 中途快照，最终以 shutdown review 为准。

### 6.3 监测/取证修复（由用户现场目视反馈触发）

用户看到实际配准时，Codex 此前短时静止采样 18 帧全 0。诊断确认：旧 probe 只数 frame，RTAB-Map 的 proximity 计数逐 process 迭代重置，延迟确认会清空 closure id；因此逐帧零值不能证明“没有历史接受”。修复后交付：

- 连续 JSONL 时间线（`info`/`initialpose`/`rosout`/`backend_log`/`tf_map_to_camera_init`/`localization_pose`）+ 累计 summary；源时间戳与接收 wall/mono 时间分开记录。
- accepted **只认** `/info` 非零 `proximity_detection_id`/`loop_closure_id`；pending（“Localization was good, but waiting…”）不是 accepted；后端 link 加入日志只是 candidate；TF/pose 发布不代表 accepted。
- 新 `/initialpose` 后历史累计保留，同时用 `accepted_since_latest_initialpose` 标记“这次初值之后是否已有接受”，避免把历史接受误算到新初值上。
- rtabmap core UWARN 不走 `/rosout`：通过 `backend_log_path` 只读增量 tail 补齐；未配置时 summary 明确 `rosout_only_partial`/`complete=false`。
- JSONL/summary 独占新文件、逐条 flush、关停 final summary 反映 `closed=true`；日志截断/轮转/缺失/超长行有显式事件。
- 测试：DSH 最终全量 **123 passed**（Domain 91 synthetic）；Codex 独立全量 **122 passed**（最终 metric 修正前，Domain 92）+ 最终针对性 **32 passed**。**不是 155 个独立测试**，也不能与现场验收相加。
- 历史真实数据重分析：codex_archived_info_review.json 显示归档数据中 9 次接受在追加 18 帧静止零计数后仍保留；这是**历史数据复核，不是本轮现场定位验证**。

### 6.4 用户目视反馈 vs Codex 独立日志验证

| 来源 | 内容 | 证据强度 |
| --- | --- | --- |
| 用户现场目视 | 看到实时配准/运动跟踪；报告 180° 场景明显扫描-地图错位；认可本轮功能测试；确认关闭现场节点 | 功能现象与可用性反馈；无坐标/角度测量，不是精度结论 |
| Codex 独立日志/JSON | 2.0813 s/ref3049/map75/36.902°、自动接受 ref152/map125、82 帧 0 接受、1.2757 s/ref268/map125、最终 81 accepted/2076 Info/errors0、DB SHA 未变、无算法残留、独立 pytest | 可审计后端事件与文件级证据；`accepted` 仍是事件证据，不是绝对精度 |

- 两者互补：用户看到“实际有效”的现象，Codex 时间线解释“为何短时静止采样全 0”；不能拿任一单方替代指标化验收。

## 7. 失败边界与未验收项

1. **180° 偏航无初值自动重定位失败**：82 帧候选、0 接受、用户目视错位；已有明确失败记录，不能写成通过。
2. **未做** 30–50 m 实采闭环度量、3 位置重复率、现场外参标定、真实 AGX/底盘/base_link/导航验收。
3. **不把 accepted 当绝对精度**：accepted 是 RTAB 后端事件证据；本轮无外部测量/ground truth，不能给出位置/角度误差或成功率。
4. **不是任意位置全局重定位**：局部 type2 几何闭环不等于全局重定位；自动起始成功只对应该起始条件，**不是 IMU 单独全局定位**。
5. **旧图/新图差异**：首图仅 23 独立约束，最终定位统一用第二图（9 独立约束）；首图早期三次定位尝试只有过程记录，未形成完整接受证据。
6. **观测完整性有限**：`/rosout` 不包含 core UWARN，backend log tail 也不是“完整日志”；summary 的 `complete=false` 必须保留。
7. **环境边界**：笔记本 ROS Humble 电脑端功能测试，不是机器人/AGX 部署，不涉及运动控制/急停/现场导航安全验收。

## 8. 证据路径（区分仓库内与本地大产物）

| 类别 | 路径 |
| --- | --- |
| 本轮取证主索引 | [docs/agent/evidence/mid360s/20261020-localization-observability/README.md](evidence/mid360s/20261020-localization-observability/README.md) |
| Codex 验收/关停/基线 | [codex_acceptance.json](evidence/mid360s/20261020-localization-observability/codex_acceptance.json)、[codex_live_shutdown.json](evidence/mid360s/20261020-localization-observability/codex_live_shutdown.json)、`codex_baseline.json`、`codex_archived_info_review.json` |
| 初值恢复原始事件 | [live_bias_recovery_20261020_154708.json](evidence/mid360s/20261020-localization-observability/live_bias_recovery_20261020_154708.json) |
| DSH 测试原始日志 | `.../raw/full_pytest_domain91.log`（123）、`.../raw/pure_pytest.log`（119）、`.../raw/ros_domain91_pytest.log`（4） |
| Codex 独立测试日志 | `.../codex_pytest.log`（122）、`.../codex_final_targeted_pytest.log`（32） |
| synthetic 样例 | `.../examples/synthetic_events.jsonl`、`synthetic_summary.json`、`synthetic_probe_report.json`（**Domain 91 synthetic，不是现场**） |
| runtime freeze | [freeze/runtime_frozen_hashes.json](evidence/mid360s/20261020-localization-observability/freeze/runtime_frozen_hashes.json)、`freeze/evidence_sha256_manifest.txt` |
| 上一轮 SLAM 证据 | [docs/agent/evidence/mid360s/20261019-slam/README.md](evidence/mid360s/20261019-slam/README.md) |
| 首图（本地，不进 Git） | `extensions/ros2_ws/src/hei_robot_slam/map/mapping_test_20261020_141902/inspection.json` 与 `rtabmap_mapping.db` |
| 第二图/实际加载图（本地，不进 Git） | `extensions/ros2_ws/src/hei_robot_slam/map/mapping_retest_20261020_145316/inspection.json` 与 `rtabmap_mapping.db` |
| 定位监测 run（本地） | `/home/robo/hei-data/localization_observability/live_20261020_154242/`、`live_20261020_154443/`、`retest_20261020_154950/`、`yaw180_20261020_155411/`（**互相并列的 run 目录**） |
| 最终关停证据（本地） | `/home/robo/hei-data/localization_observability/yaw180_20261020_155411/shutdown_review.json` 及同目录 JSONL/summary/monitor.log |
| 采样诊断/旧 probe 快照 | `map/20261020_145614/localization_observability_diagnosis.json`、`map/localization_test_20261020_1421/info_initial.yaml`、`info_followup.yaml` |

- 地图/DB/JSONL/monitor 日志在 `/home/robo/hei-data/` 与包的 `map/` 下，均**本地大产物、不进 Git**；仓库内只保留小型 review/acceptance/freeze 证据。

## 9. Git 状态与保护（2026-10-20 16:31 快照，未提交）

```text
branch: codex/slam-mapping-relocalization-test
HEAD:   fbe160bb5a4a9cdb36bc1ab069ed72d9cc9a2e42

 M docs/agent/EXECUTION_LOG.md                  # Codex 单写
 M docs/agent/TASK_BOARD.md                     # Codex 单写
 M extensions/ros2_ws/src/hei_robot_slam/README.md
 M extensions/ros2_ws/src/hei_robot_slam/hei_robot_slam/cloud_probe.py
 M extensions/ros2_ws/src/hei_robot_slam/package.xml
 M extensions/ros2_ws/src/hei_robot_slam/test/test_node_startup.py
?? docs/agent/evidence/mid360s/20261020-localization-observability/
?? extensions/ros2_ws/src/hei_robot_slam/hei_robot_slam/localization_events.py
?? extensions/ros2_ws/src/hei_robot_slam/test/test_localization_events.py
?? extensions/ros2_ws/src/hei_robot_slam/test/test_localization_observability_integration.py
```

- 本文件新增后 `git status` 会再多一项未跟踪的 `docs/agent/MID360S_SLAM_TEST_SUMMARY_20261020.md`；这是本次唯一新增文件。
- 保护范围：现有未提交监测改动与 Codex 台账改动不得被覆盖；未执行 `git add`/`commit`/`push`/`checkout`/`stash`。

## 10. 异常、缺失与已知小缺陷（不扩大本轮范围）

1. **路径表述澄清**：任务文字把各 run 写成 `live_20261020_154443/` 的子目录；实测它们是 `/home/robo/hei-data/localization_observability/` 下**并列目录**（`retest_20261020_154950/`、`yaw180_20261020_155411/` 等），本总结按实际路径记录。
2. **监测 summary 元数据小缺陷（未修代码）**：`localization_events.py` 的 `_record_accepted_event` 从 `event["data"].get("accepted_ids")` 取值，实际 ID 在 `event["data"]["derived"]["accepted_ids"]`；导致 `summary.last.accepted_event.accepted_ids` 为空字典（例如 yaw run seq 3989/ref 1450、retest seq 420/ref 286、live_154443 seq 666/ref 3128）。**原始 JSONL 的 raw/derived ID、累计计数、review/acceptance 与算法配准均不受影响**；建议后续小任务修复并加回归。
3. **快照 vs 最终数**：retest 自动起始 review 是首次接受快照（accepted_frames=1），该 run 结束累计 12 accepted/231 Info；运动 review 的 76 accepted 是 16:13 中途快照，最终以 shutdown review 的 81 accepted/2076 Info 为准。
4. **证据时序与验收快照**：`/live_20261020_154242` 是修复前的 108 帧只读 smoke（其关停 JSON 自述先于 `optimization_frames` 计数修正）；`/live_20261020_154443` 是**修复后重新启动的最终只读 monitor**。随后用户才进行 15:47 偏差恢复、15:49 自动起始，以及 15:54/15:56 的 180° 旋转测试；这些是本轮**修复后的现场真实测试证据**。`codex_acceptance.json` 中的 `not_validated` 是写入当时的快照，后续真实测试已经发生，不能当作最新结论；本文以实际 JSONL/review 时间为准。
5. **monitor.log 关停噪音**：每个 monitor 在 ctx 关闭后出现 `Failed to publish log message to rosout: publisher context is invalid`；这是关闭阶段 rosout 发布噪音，JSONL/summary write_errors 均为 0，不是落盘失败。
6. **残留进程**：只读 `ps` 未发现 FAST-LIO/RTAB-Map/livox/launch/RViz 进程；仅见一个 `ros2` CLI daemon（Domain 89，非算法节点，本总结未操作）。Codex 已核验现场算法节点全部关闭。
7. **无精度真值**：本轮没有外部测量、标定或 ground truth，故不提供位置/角度误差数字；用户目视反馈也无法替代测量。
8. **旧图早期定位尝试**：14:21/14:44/14:48 只有旧 probe 报告和 `/info` 快照，没有连续 JSONL/完整 review，因此本总结不把它们列为通过或失败，只记录“曾用首图做过定位尝试”。

## 11. 后续建议（按优先级）

1. **先修监测元数据小缺陷**（§10.2）：改为读 `data.derived.accepted_ids` 并加一条回归；这是小任务，不要在本总结提交时顺手修改。
2. **后续重复多组偏差恢复统计成功率**：在修复后 monitor 与已完成的单次偏差恢复基础上，重复多组记录 initialpose→accepted 延迟、correction 角度、TF 变化，统计成功率/分布；不要把本轮单次事件当作重复性结论。
3. **补 30–50 m 实采闭环度量**：闭环误差、重访一致性、地图漂移；记录路线/速度/光照/结构条件。
4. **3 位置重复率**：同一保存图、固定三位置、多次重定位成功率和误差分布；再谈“任意位置全局重定位”。
5. **外参现场标定**与线缆/安装固定；否则运动/偏航场景的可见错位无法归因。
6. **AGX/实机/base_link/底盘导航**为独立里程碑；需重新授权、急停与现场安全检查，不在本轮范围。
7. **保留证据卫生**：每次 run 记录 map SHA、launch_info、monitor JSONL/summary、exit/write_errors；地图/DB/JSONL 不进 Git，只在 `/home/robo/hei-data/` 保留并登记哈希。
8. **提交策略**：待 Codex 终审通过、用户明确授权后再 commit/push；当前所有改动（含本总结）保持未提交、未推送、未切分支。

## 12. 本文档检查与冻结

- 仅新增本文件；未改代码/配置/地图，未 build、未启动节点/驱动/RViz，未运行算法，未操作机器人，未 commit/push/切分支。
- 已检查本地 Markdown 链接可解析、`git diff --check`（针对本文件）无空白错误；建议 Codex 审查后按需再将本文件纳入证据清单。
- 待 Codex 审查；审查前视为冻结版本，不再修改实现或运行环境。
