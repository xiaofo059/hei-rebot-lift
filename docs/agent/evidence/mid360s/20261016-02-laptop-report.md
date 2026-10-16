# HEI-LIDAR-20261016-02-PREP 专项报告（DSH-LAPTOP）

## 0. 身份与范围

| 项 | 值 |
| --- | --- |
| Task ID | `HEI-LIDAR-20261016-02-PREP`（P1，Owner/验收 Codex） |
| 执行端 | DSH-LAPTOP |
| 主机 / 架构 | `fo` / x86_64 / Ubuntu 22.04（`Linux 6.8.0-138-generic`） |
| 仓库根 | `/home/robo/robotopia/hei-rebot-lift` |
| 分支 / base SHA | `feat/mid360s-bringup` @ `f97fc4c14216d202d63269a8de1424ae646ca14d` |
| 硬件 | **雷达未连接**；真实启动/录包/回放一律 NOT_RUN |
| 本报告状态 | 软件实现 + 本机测试完成，**等待 Codex 最终 Review**（非自行验收） |

## 1. 改动文件（限白名单）

| 路径 | 说明 |
| --- | --- |
| `.gitignore` | 追加 `/extensions/third_party/`、`/extensions/config/local/` |
| `extensions/dependencies/livox.repos` | 两个第三方仓库 40 位 SHA 冻结清单 |
| `extensions/tools/build_mid360s_underlay.sh` | 新增：无 sudo 构建 SDK2 + driver underlay |
| `extensions/ros2_ws/src/hei_robot_bringup/**` | 新增：launch、配置生成、bag 验收、测试、README |
| `docs/MID360S_BRINGUP.md` | 新增：阶段路线与操作/验收说明 |
| `docs/agent/evidence/mid360s/**` | 新增：本报告与原始证据 |

未触碰：Codex 的 `docs/agent/TASK_BOARD.md`、`docs/agent/EXECUTION_LOG.md`（保持 dirty 原样）、
其他模块、旧文档、网络配置、`extensions/third_party/` 内第三方源码（`git status` 全程干净）。

## 2. 依赖冻结

| 组件 | tag | SHA |
| --- | --- | --- |
| Livox-SDK2 | `v1.3.1` | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` |
| livox_ros_driver2 | `1.2.6` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |

S 型支持证据：`livox_lidar_def.h:69 kLivoxLidarTypeMid360s = 35`；
`pub_handler.cpp:135` 同时处理 `kLivoxLidarTypeMid360{,.s}`；driver 自带
`config/MID360s_config.json`。**未用 MID360（非 S）配置冒充 S 型**。

## 3. 真实构建与测试结果

构建（干净 ROS shell，无 sudo）：

```bash
extensions/tools/build_mid360s_underlay.sh
```

| 检查 | 结果 |
| --- | --- |
| SDK 安装 | `extensions/third_party/livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so`（1 214 224 B） |
| driver colcon build | `Finished <<< livox_ros_driver2`，exit=0 |
| `ldd` 组件库 | `liblivox_lidar_sdk_shared.so => <sdk_install>/lib/...`，not_found=0 |
| `ldd` node | not_found=0 |
| RUNPATH | `<sdk_install>/lib:/opt/ros/humble/lib` |
| 第三方源码 | `git status --porcelain --untracked-files=no` = 0 行 |

测试：

```bash
colcon test --packages-select hei_robot_bringup && colcon test-result --verbose
```

| 结果 | 值 |
| --- | --- |
| 用例总数 | **126 tests** |
| 失败 / 错误 / 跳过 | **0 / 0 / 0** |

集成检查（`20261016-integration-checks.txt`）：`ros2 pkg prefix hei_robot_bringup` /
`livox_ros_driver2` 均 exit=0；`ros2 pkg executables` 列出两个 CLI；
`ros2 interface list` 含 `livox_ros_driver2/msg/CustomMsg|CustomPoint`；
`ros2 launch ... --show-args` exit=0 且 `config_path` 无默认值。

## 4. Review 纠错逐项关闭证据

### RF-LIDAR-001（P1）首次运行多级目录缺失 → FileNotFoundError

- 修复：`make_config._nearest_existing_dir()` 向上找到第一个真实存在的祖先再交给
  `git check-ignore`；不覆盖已有文件的语义保持不变。
- 回归：`test_first_run_creates_missing_multi_level_dirs`、
  `test_first_run_unignored_missing_dirs_refused_cleanly`、`test_nearest_existing_dir_walks_up`。
- 实测：在 `extensions/config/local/` 整条路径不存在时生成成功（exit=0）。

### RF-LIDAR-002（P2）构建脚本过弱

| 子项 | 修复 | 证据 |
| --- | --- | --- |
| 只校验 HEAD | `git status --porcelain --untracked-files=no` 非空即拒绝 | 实测：改 `CHANGELOG.md` → 脚本 exit=1 并打印该文件；还原后 porcelain=0 |
| 无条件删除 `launch/` | 预存 `launch/` 直接报错退出；仅清理本脚本创建的目录（`CREATED_LAUNCH`） | 实测：预置 `launch/KEEP_ME.txt` → exit=1，用户文件内容原样保留 |
| `ldd` 失败被吞 | 去掉 `|| true`，`ldd` 非零即 `fail` | 静态护栏 `test_ldd_failure_is_not_swallowed`；`ldd` 对非 ELF 返回 1（证明判定有意义） |

### RF-LIDAR-003（P2）上游事实与表述

- 已核对并写入文档：`src/lddc.cpp:481` 硬编码 `imu_msg.header.frame_id = "livox_frame"`，
  点云（`lddc.cpp:263,355,421`）才用 `frame_id`。
- 表述已改正：由「与其发布话题不一致」改为「与**配置的点云 frame**不同」，
  并说明 frame 与 topic 无需同名（`test_imu_frame_comment_does_not_claim_topic_mismatch`）。
- IMU 单位/字段（`20261016-build-evidence.txt`）：
  `pub_handler.cpp:119-124` 原样拷贝、`lddc.cpp:489-494` 原样写入且**不设**
  `orientation`/`covariance`（全 `src/` 命中 0）；`lidar_imu_data_queue.h:37-42`
  driver 自述 **acc = g、gyro = rad/s**。文档与 `bag_qc` 输出/JSON `note` 都声明：
  原始包保留原值、本阶段不做 SI 转换、**不得**当作已符合 SI 的融合输入或已有姿态。
- **证据边界（如实记录）**：DSH 沙箱无法访问 Codex 给出的官方协议 URL，
  `web_fetch` 原始错误为 `URL hostname "livox-wiki-en.readthedocs.io" resolves to a non-public
  IP address`；故单位结论来自**冻结源码自述**，未独立核对 wiki。
- 未发布任何 base/IMU TF（`test_no_tf_publisher_for_unmeasured_extrinsics`）。

### RF-LIDAR-004（P1）质量工具虚报 PASS

Codex 四个独立复现，修复前均错误 `exit=0`；修复后（`20261016-synthetic-qc.txt`）：

| 复现 | 修复后 |
| --- | --- |
| ① 点云 2 s@10 Hz、IMU 仅 0.1 s@100 Hz | exit=1（`imu.duration`、`streams.window_coverage` FAIL） |
| ② 点云 1–3 s、IMU 101–103 s 完全不重叠 | exit=1（`streams.header_overlap` = −98 s FAIL） |
| ③ 点云载荷全为非法 CDR | exit=1（`lidar.decode` 61/61 FAIL） |
| ④ PointCloud2 width=0/data 空/fields 空 | exit=1（`lidar.pc2_structure` 301 条 FAIL） |

实现要点：

- **两条流都**校验覆盖时长、断流、帧头存在/单调、源帧头间隔；
  新增**源帧头时间范围重叠且覆盖要求时长**（同一采集窗口）。
- 类型不可导入、反序列化失败、缺帧头、结构非法 → **FAIL**，不再只 WARN。
- PointCloud2 结构：`width/height>0`、`fields` 含 x/y/z、字段不越界、
  `row_step = point_step×width`、`data` 长度匹配、必须含点级 `timestamp`。
- CustomMsg：`point_num == len(points)`、点非空、逐点 `offset_time` 存在且单调不冻结。
- 帧头检查**默认全量**；`--header-sample` 只能 INCONCLUSIVE、**不能 PASS**；
  `0`/负数直接拒绝（exit=2）。
- 阈值校验：有限正数、上下限次序、IMU 上下限成对。

**残留修复**（Codex 二次复现：第 6–10 帧冻结第 5 帧时间后恢复仍 exit=0）：

- 由「整段唯一值集合」改为**逐相邻帧头对**判定：重复/冻结与倒退分别计数；
- 新增**源帧头最大间隔** `<= max-gap`，捕捉「接收时间连续但传感器时间跳变」；
- 实测（`data/mid360s/midfreeze`，expected-duration=2、imu 95–105）：
  `lidar.header_monotonic` FAIL「重复/冻结 5 次」、`lidar.header_gap` FAIL「0.600s」，
  **exit=1**；回归用例 `test_midstream_header_freeze_then_recovery_fails`、
  `test_source_header_gap_detected_even_when_receive_times_contiguous`、
  `test_imu_source_header_gap_fails`。
- 另按要求：`--lidar-topic` 与 `--imu-topic` 相同 → exit=2
  （防止两条流统计合并；`test_same_topic_for_both_streams_rejected`）。

### RF-LIDAR-005（P2）文档

1. 阶段编号改为与 TASK_BOARD 一致：01-SOFTWARE / 01-HARDWARE / 02-PREP / 02-LIVE /
   03-LIO / 04-SLAM / 05-AGX / 06L·06A-BRIDGE / 07L·07A-NAV；接线/录包/回放归入
   02-LIVE 子步骤，不再另编号。
2. §4 标题改为「操作指令（逐段标注执行状态）」，每段标 **[已执行]** 或 **[NOT_RUN]**；
   并声明 02-PREP 软件仍等待 Codex 最终 Review，非自行验收通过。
3. 命令块统一从仓库根目录开始；§4.3 构建后 `cd ../..` 回根；§4.4 改用
   `local_setup.bash` 且删除 `2>/dev/null || true`。
4. 正式包流程明确：先停 PointCloud2 driver → 用 `pointcloud_format:=custom` 单独启动
   **唯一** driver → 录 CustomMsg + IMU + 存在的 `/tf /tf_static`；短包检查与 30 分钟正式
   验收分开；建议录 **31 分钟**覆盖启停边界、仍要求有效覆盖 ≥1800 s；回放标 NOT_RUN。
5. 网络约束改述为**本轮（雷达未接）**的范围约束，并写明「保持原样直到用户确认真实接线/IP，
   后续按已授权任务与现场证据选择可回滚方案」；补充 §5.4「未锁 PTP ≠ 时间无效，
   全零/冻结才不合格」。
6. §3.1 增加不用 vcstool 的精确 SHA 获取命令（不覆盖已存在目录）；
   §6 阈值表更新为与 004 最终实现一致（含源帧头间隔、逐相邻对、话题区分）。

## 5. 未执行项（NOT_RUN）

- 真实雷达连接、真实点云/IMU、RViz 真实显示、`01-HARDWARE`。
- 真实启动（`ros2 launch` 带 config_path）、短包、正式 31 分钟包、`ros2 bag play`。
- 全部 LIO/SLAM/导航、AGX 部署、网络配置变更。
- 硬件时间同步精度（gPTP/PPS 锁定）与外参标定。
- `rosdep install`/apt：本机 `sudo -n true` 失败（DSH 沙箱 `no new privileges`；
  与本机 sudo 需密码是不同限制），未执行任何提权安装；`vcstool` 未安装，
  以 `git clone` + SHA 校验代替。

## 6. 阻塞与风险

- 无软件阻塞。硬件阻塞：`01-HARDWARE` 待用户接线。
- 首次真实验收**无法直接给 PASS**：IMU 标称频率未知，需先确认再复验。
- 风险提示：官方示例地址落在 `192.168.1.x`，与本机 WiFi `192.168.0.0/23` 路由重叠，
  接线后须先确认网段（详见 `docs/MID360S_BRINGUP.md` §5）。

## 7. 证据文件

| 文件 | 内容 |
| --- | --- |
| `20261016-integration-checks.txt` | pkg prefix / executables / interface / --show-args 原始输出与退出码 |
| `20261016-build-evidence.txt` | 冻结 SHA、ldd、RUNPATH、S 型证据、IMU 单位证据 |
| `20261016-synthetic-qc.txt` | RF-004 四例 + 残留例 + 三结论路径 + 绕过参数拒绝（全部 SYNTHETIC） |

> 所有 QC 数据均为 **SYNTHETIC 合成 bag**，不代表真实雷达/IMU 观测，也不证明硬件时间同步精度。
