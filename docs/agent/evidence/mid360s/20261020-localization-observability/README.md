# HEI-SLAM-OBS-20261020 证据：定位连续取证（probe JSONL / summary）

> 任务：只实现定位连续取证，不改算法。
> 分支 codex/slam-mapping-relocalization-test，基线 fbe160bb5a4a9cdb36bc1ab069ed72d9cc9a2e42。
> 本目录只含小型日志/JSON/清单；runtime 与 build 已冻结（哈希见 freeze/runtime_frozen_hashes.json）。
> 本目录内所有 JSONL/summary 示例均来自 **Domain 91 synthetic** 测试，不是现场定位、不是精度验收。

## 1. 实现文件（白名单内）

| 文件 | 说明 |
| --- | --- |
| hei_robot_slam/localization_events.py（新增，纯逻辑） | 事件分类/累计 summary/JSONL 独占 writer/后端日志只读 tail；无 ROS import |
| hei_robot_slam/cloud_probe.py（最小接线） | 新增 /info、/initialpose、/rosout 订阅；map->camera_init TF 变化；/localization_pose；backend_log tail；report 增加 localization_observability（旧字段保留） |
| package.xml | 新增 rcl_interfaces（/rosout Log 类型） |
| test/test_localization_events.py（新增） | 纯逻辑 31 项回归 |
| test/test_localization_observability_integration.py（新增） | Domain 91 synthetic Info/initialpose/TF/rosout/backend log 端到端 |
| test/test_node_startup.py（改） | 测试域从硬编码 89 改为默认 **91**（拒绝 89），新增 JSONL/summary 关停与 disabled 状态断言 |
| README.md（改） | 新增“定位连续取证”说明 |

未改：FAST-LIO/RTAB 算法、参数 yaml、TF/点云内容、launch、驱动/硬件、地图、third_party；未 commit/push。

## 2. 语义（验收重点）

| 项 | 行为 |
| --- | --- |
| accepted | **仅** /info 发布的非零 proximity_detection_id / loop_closure_id（CoreWrapper.cpp 2429 使用这些 ID）；不是精度通过 |
| pending | “Localization was good, but waiting ...” 记为 pending 候选，不计 accepted；/localization_pose 发布不算 accepted |
| candidate_link_added | 后端日志 “Add local loop closure in SPACE” 记为候选 link（可能被图优化拒绝），**不更新** last accepted、不清 pending、不计最终接受 |
| no_current_event | 当前帧 accepted id 为 0 是正常现象；累计 history 不清零、不等于失败 |
| 小运动无候选 | small_movement_frames_without_candidate 计数并明确不是失败 |
| initialpose | 记录收到事件，与 accepted 只做 **时间关联**（association=time_only_not_causal）；新初值后 accepted_since_latest_initialpose=false 表示这次初值之后尚无接受事件，历史累计保留 |
| map->camera_init TF | 只记录首值/变化（平移 m、角度 rad，q/-q 等价）；TF 变化不代表 accepted |
| /rosout vs 后端 stderr | 实查：initialpose received 是 ROS 日志；Localization was good / Update map correction 是 rtabmap core UWARN 直写 stderr（ANSI + Rtabmap.cpp 时间戳），**不发布到 /rosout**。未配置 backend_log_path 时 summary 明确 rosout_only_partial、complete=false |

## 3. 输出文件与读取方式

report_path 存在时自动派生（独占新文件；已存在则加 __02 等后缀，不覆盖）：

* <report_stem>.localization_events.jsonl：逐事件（schema=hei.localization_event.v1），每条 flush，含 seq/source/source_stamp_ns/recv_wall_ns/recv_mono_ns；
* <report_stem>.localization_summary.json：cumulative（历史累计）/current/last/recent/derived/pending_observability/event_status；关停时 final=true 且 writer.closed=true。

事件类型：info、initialpose、rosout、backend_log、tf_map_to_camera_init、localization_pose。
后端日志 tail 从启动时 EOF 开始，显式产生 backend_log_truncation/rotation/missing/reappeared/line_overflow/open_error 事件；
open_error 单独计入 backend_log_error_events（不混入 overflow）。

## 4. 证据清单

| 路径 | 内容 |
| --- | --- |
| raw/full_pytest_domain91.log | 全量测试 **123 passed**（含 ROS 节点与 synthetic 集成；Domain 91） |
| raw/pure_pytest.log | 纯逻辑/静态回归 **119 passed** |
| raw/ros_domain91_pytest.log | ROS 节点启动 + synthetic 集成 **4 passed** |
| examples/synthetic_events.jsonl | synthetic 端到端时间线（14 事件：info×3、initialpose×1、tf×2、rosout×2、backend_log×5 等） |
| examples/synthetic_summary.json | synthetic summary：accepted=2（仅 info id）、pending=2、candidate_link_added=1、tf_change=2、writer.closed=true、events_written=14、errors=0 |
| examples/synthetic_probe_report.json | synthetic probe 最终报告（旧契约字段 + localization_observability） |
| examples/synthetic_backend_log_snapshot.log | synthetic 后端日志快照（含截断/轮转前内容） |
| examples/onsite_log_format_readonly_excerpt.txt | 用户现场运行日志的**只读小片段**，仅用于证明 UWARN 非 rosout 格式；不修改源文件 |
| freeze/runtime_frozen_hashes.json | 22 项 source↔installed 哈希全部一致（含新模块/entry point/package.xml） |
| freeze/evidence_sha256_manifest.txt | 本目录文件哈希清单 |

## 5. 测试覆盖（摘要）

| 覆盖点 | 断言 |
| --- | --- |
| accepted 一帧后连续零帧 | cumulative 仍为 1；current accepted=false；state_hint 为历史接受无当前事件 |
| 小运动无候选 | small_movement_frames_without_candidate 计数，不判失败 |
| pending 延迟确认 | rosout/backend 文本均计 pending，accepted=0、last accepted=None |
| candidate link added | 只计入 candidate 计数，不更新 accepted/pending 状态 |
| 新 initialpose | accepted_since_latest_initialpose=false、pending_current_initialpose_attempt=true，历史累计>0；之后接受则变 true |
| initialpose 时间关联 | association=time_only_not_causal；无初值时 correlation=None |
| TF q/-q 等价与去重 | 同值重复不落盘；首值+变化各 1 次；角度/平移差值正确 |
| 缺字段/NaN | raw id 保持 null、stats 非有限置 null；summary 不允许 NaN |
| optimization_frames | 仅 iterations>0 计入；0/字段缺失不计（最终 metric 小修） |
| JSONL 不覆盖/关停 | 既有 jsonl+summary 换后缀；orphan summary 不被覆盖；close 后 flush、final summary closed=true |
| backend tail | EOF 起步、截断/轮转/缺失/重现/超长行、partial line |
| 节点集成 | synthetic Info/initialpose/TF/rosout/localization_pose + backend log 端到端；旧 report 契约保留 |

## 6. 复现命令

    cd /home/robo/robotopia/hei-rebot-lift/extensions/ros2_ws
    source /opt/ros/humble/setup.bash
    source ../third_party/livox_ws/install/local_setup.bash
    source ../third_party/fast_lio_ws/install/local_setup.bash
    source ../third_party/rtabmap_local/env.sh
    source install/local_setup.bash
    export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 HEI_TEST_ROS_DOMAIN_ID=91
    cd src/hei_robot_slam
    /usr/bin/python3 -m pytest test/ -q

## 7. 边界与未覆盖项

* **synthetic**：本目录所有 JSONL/summary 示例来自 Domain 91 合成发布器；未启动真实 RTAB-Map/FAST-LIO/雷达驱动；
* **现场**：Domain 89 用户会话未被停止/重启/发布初值/占用；DSH 未在该域启动新 observer。现场只读 monitor 由 Codex 追加，待其回显；本目录仅含现场日志只读格式片段；
* **历史**：旧 20261019 数据仅作为历史基线，不在本任务结论中使用；
* **精度**：accepted 是后端事件证据，不是地图/定位精度或现场成功率；30–50 m 闭环、3 位置重定位、外参标定仍未验收；
* **观测完整性**：/rosout 不能完整捕获 rtabmap core UWARN；默认 rosout_only_partial。配置 backend_log_path 可补齐 pending，但不保证所有内部日志都写入该文件（summary 保持 complete=false）；
* **测试域**：所有 ROS 测试固定默认 Domain 91，显式拒绝 89；Codex 独立测试使用 Domain 92。

## 8. Git 范围

仅修改/新增白名单内文件（见 §1）；docs/agent/TASK_BOARD.md、docs/agent/EXECUTION_LOG.md 为 Codex 单写，本次未修改（哈希见 freeze/protected_files_sha256.txt）。未 commit/push/切分支。

## 9. 最终 metric 小修（收尾）

终审发现：on_info 里 optimization_frames 只要 iterations 非 None 就 ++，现场零迭代会被误计为优化帧。
已只改这一处为 **iterations > 0**（字段缺失/0 均不计），并新增纯逻辑回归
test_zero_optimization_iterations_not_counted_as_optimization_frame（零迭代→正迭代→字段缺失）。
修复后重跑：全量 **123 passed**、纯逻辑 **119 passed**、ROS 节点+synthetic 集成 **4 passed**（Domain 91）。
synthetic 样例中所有 info 帧 iterations>0，因此既有样例不受该 metric 修复影响；runtime 哈希已刷新
（freeze/runtime_frozen_hashes.json，frozen_at 为实际写入时间），此后不再改代码/构建。

## 10. 外部状态（Codex 回显，非 DSH 自测）

* Codex 已独立完整 pytest 通过，并独立运行新增针对测试通过；
* Codex 现场只读 monitor 已验证 108 帧并正常 close、零写错误；最终 monitor 已启动且地图 SHA 不变；
* metric 修复后 Codex 重启自己的 monitor，不碰算法；DSH 仍未在 Domain 89 启动 observer、未停止/重启现场算法。
