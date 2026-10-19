# MID-360S → FAST-LIO2 离线接入：单位/坐标/外参审计与输入契约

> 任务：HEI-LIDAR-20261017-03-LIO-BASE（03-LIO 首批接入）
> 分支 `feat/mid360s-lio` 基线 `1aeb8a1`；本文所有行号对应下面列出的冻结 SHA。
> 本轮**没有**运动数据、**没有**现场标定，因此本文**不**声称 LIO 可靠、**不**声称 SLAM 完成。
> 本文是可核查的静态审计 + 离线回放证据，不是可靠性验收。

## 0. 结论摘要

| 问题 | 结论 | 依据 |
| --- | --- | --- |
| 原始 IMU 加速度单位 | **g**（原始值原样，非 SI） | driver 源码自述 + 正式包实测范数 0.995 |
| 是否需要在适配器里 ×9.81 | **不需要，且不要做** | FAST-LIO 用 `G_m_s2/|mean_acc|` 自行归一化（`IMU_Processing.hpp:260`） |
| 原始 gyro 单位 | **rad/s**（已是 SI 角速度） | driver 源码自述 |
| orientation 是否可用 | **不可用**（driver 不赋值，FAST-LIO 也不读） | 源码 + 实测恒 `[0,0,0,1]` |
| IMU 轴向 | 手册：**与点云坐标同向**（名义 `R_IL = I`） | MID-360S 手册 §3.5 p.22 |
| IMU–LiDAR 外参 | 名义 `t_IL = [-0.011, -0.02329, +0.04412] m`，**未现场标定** | 手册芯片位置 + `p_I = R_IL p_L + t_IL` 推导 |
| CustomMsg 逐点时间 | ns、相对本帧 `timebase`；帧内**存在乱序** | driver 源码 + 正式包 300 帧实测 |
| 帧尾（scan end） | 上游按数组末元素取；本数据 300/300 帧末元素恰好是最大值，但**过滤后**末保留点可比真实最大早 0–0.457 ms | 实测 + `laserMapping.cpp:399/406/407` |
| 与冻结 driver 是否兼容 | **兼容**，需 `lidar_type=1`、`scan_line=4`、`time_sync_en=false`；另需 2 个 tracked patch | 见 §5、§6 |
| TF | 唯一发布者 `fastlio_mapping`，`camera_init`→`body`；**不发布** `base_link`/`map` | `laserMapping.cpp:647-658` |

**一句话**：冻结 driver 的输出可以被 FAST-LIO 直接消费（加速度不需要换算、帧内乱序不破坏逐点去畸变），
真正缺失的是**现场外参标定**与**运动数据**，两者都不在本轮范围内。

## 1. 基线、证据分级与环境

| 项目 | 值 |
| --- | --- |
| 仓库 / 分支 / 基线 | `/home/robo/robotopia/hei-rebot-lift` / `feat/mid360s-lio` / `1aeb8a19952cd9011c428a2fbea063683f0cade5` |
| FAST_LIO_ROS2 | `2fffc570a25d0df172720bac034fbdb6a13d2162`（分支 `ros2`，package `fast_lio`） |
| ikd-Tree（子模块） | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`（分支 `fast_lio`） |
| livox_ros_driver2（冻结） | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`（1.2.6，带 tracked patch） |
| Livox-SDK2（冻结） | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`（v1.3.1） |
| 官方手册 | Livox Mid-360S User Manual（2026-06-01），sha256 `bf064396c2a5254bc3fc98044d65b2dcf3ad3354e942eee1ee46b37e6e0c80d4` |
| 正式数据 | `/home/robo/hei-data/mid360s_live_20261017_retest01`（1864.757 s，18,648 CustomMsg，372,964 IMU，02-LIVE 已 PASS） |
| 宿主 | `fo` / x86_64 Ubuntu 22.04 / ROS 2 Humble / 系统 `/usr/bin/python3`，Conda 隔离 |

证据分级（沿用 `README_FOR_AGENTS.md`）：**[S]** 源码事实、**[T]** 本轮实测、**[D]** 文档声明、
**[I]** 分析推断、**[?]** 待确认。凡未标注者，本文结论均为 [S] 或 [T]。

许可状态（**必须人工确认，本轮不下结论**）：FAST_LIO_ROS2 的 `package.xml` 写 BSD，但其
`LICENSE` 文件实际是 GPL-2.0 全文（sha256 `8177f975…80643`）——元数据与文件矛盾；
ikd-Tree 冻结树里**没有 LICENSE 文件**，README 与源码头也无许可声明，状态为 UNKNOWN。
两者都在 `extensions/third_party/`（已被 `.gitignore` 忽略），不入父仓库。

## 2. 输入语义审计（冻结 driver，[S] + [T]）

| 事实 | 位置 |
| --- | --- |
| CustomMsg 的 `header.stamp` 与 `timebase` 都等于帧内**首个被追加点**的绝对时间 | `lddc.cpp:354-373` |
| `offset_time = 点绝对时间 - pkg.base_time`，单位 **ns** | `lddc.cpp:395` |
| 逐点绝对时间 = `pkt.time_stamp + i * point_interval`，按 **UDP 到达序**追加，全链路不排序 | `pub_handler.cpp:404/432/467` |
| 点云 frame 用配置值（本仓库 `lidar_link`） | `lddc.cpp:355` |
| IMU 的 `frame_id` 硬编码 `livox_frame`，不受参数影响 | `lddc.cpp:481` |
| IMU 字段**原样**写入，无换算、无 orientation/covariance | `lddc.cpp:489-494`、`pub_handler.cpp:119-124` |
| driver 自述 gyro=rad/s、acc=g | `comm/lidar_imu_data_queue.h:37-42` |
| `line = i % pkt.line_num`，MID-360 系列 `line_num = 4` | `pub_handler.cpp:402`、`comm/comm.h:82` |
| 配置里的 `extrinsic_parameter` 会参与点云坐标计算；本仓库配置为 **identity**（无副作用） | `pub_handler.cpp:386-401`、`hei_robot_bringup/mid360s_config.py` |

### 2.1 正式包实测（只读短窗口，[T]）

15 s 窗口（150 帧 / 1,995,264 点）与 30 s 窗口（300 帧 / 5,998,848 点）：

| 量 | 实测 |
| --- | --- |
| `header.stamp == timebase` | 150/150、300/300 帧成立 |
| 帧内首个点的 `offset_time` | 恒为 0（帧基准=首个追加点） |
| 帧内相邻逆序 | **150/150 帧都有**，共 23,238 处；单次最大回退 **467,980 ns** |
| 单帧 `max(offset_time)` | 90.6–110.2 ms（平均 100.42 ms） |
| 数组末元素是否等于最大 offset | **300/300 帧成立**（本数据未出现“末点非最大”） |
| 末端保留点 vs 最大保留点（复刻 `avia_handler` 过滤） | 94/300 帧 >0，p90 = **272,570 ns**，最大 **457,347 ns** |
| 相邻帧：`next_base - (prev_base + prev_max_offset)` | min **-465,626 ns**、mean **-350,594 ns**、max **+1,289,543 ns** |
| `line` 分布 | 0/1/2/3 各 1,499,712 点（完全均分，全部 < 4） |
| `tag & 0x30` 分类 | 0x00: 5,039,949；0x10: 898,205；**0x20: 60,694（1.01%，会被 avia_handler 丢弃）** |

**含义**：帧与帧的时间范围**本来就重叠**约 0.35 ms（前一帧末点晚于下一帧 base），
这是 NoSync 打戳 + 到达序追加的必然结果，不是数据缺陷；FAST-LIO 的 `lidar_end_time`
与 IMU 配对按“帧 base + 帧尾 offset”计算，因此这一点必须如实记录，不能用任意时间平移“修掉”。

## 3. 官方手册：轴向与 IMU 位置（S 型专属，[S]）

已下载并逐字核对的官方手册：<https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf>

* §3.4 Coordinates（印刷页 18）：给出点云坐标 `O-XYZ` 的定义与轴图。
* §3.5 IMU Data（印刷页 22）原文：

  > The LiDAR sensor is integrated with an IMU chip (with a 3-axis accelerometer and
  > a 3-axis gyroscope). By default, the product starts to push IMU data at a
  > frequency of 200 Hz after power-on … The IMU data contains the 3-axis
  > acceleration and 3-axis angular velocity, **the directions of which are the same
  > with that of the point cloud coordinates**. **The position of the IMU chip in the
  > point cloud coordinates is x=11.0 mm, y=23.29 mm, z=-44.12 mm.**

由此得到两个**名义**参数：

1. 轴向同向 ⇒ `R_IL = I`（IMU 系与 LiDAR/点云系无旋转）。
2. 芯片位置是 `r_LI`（IMU 原点在 LiDAR 系中的坐标），**不是** FAST-LIO 要的外参。

### 3.1 外参换算：p_I = R_IL p_L + t_IL

FAST-LIO 对每个点做的正是这个变换（`IMU_Processing.hpp:327` 的
`offset_R_L_I * P_i + offset_T_L_I`；参数装载见 `laserMapping.cpp:895-897`），
所以 `mapping.extrinsic_T` 的语义是 **t_IL：LiDAR 原点在 IMU 系中的位置**。

```text
p_I = R_IL (p_L - r_LI) = R_IL p_L + t_IL
t_IL = -R_IL · r_LI
R_IL = I  ⇒  t_IL = -r_LI = [-0.011, -0.02329, +0.04412] m
```

**把芯片坐标 [+0.011, +0.02329, -0.04412] 直接填进 `extrinsic_T` 是错的（三个符号全反）。**
回归测试 `test_imu_origin_maps_to_zero_in_imu_frame` 固定了这一点：把芯片位置当作 LiDAR 系中的点
送进 `lidar_point_to_imu` 必须得到原点。

交叉核对（不是来源）：上游 `config/mid360.yaml` 的 `extrinsic_T: [-0.011, -0.02329, 0.04412]`
与上述推导一致。注意本任务**没有**直接照抄该值当作 S 型已标定结果，而是从 S 型手册推出同名结果，
并把它标记为 manufacturer nominal。

### 3.2 这些**不能**证明什么

* 单一静止姿态只能说明“静态时重力沿 +Z、模 ≈1 g”，**不能**证明 X/Y/Z 轴符号、偏航角或旋转外参。
* 手册“同向”是**结构性声明**，不覆盖装配公差、IMU 安装偏角，也不给出标定不确定度。
* 因此：**外参 = UNKNOWN（现场待标定）**；实验配置显式标记 `UNCALIBRATED_NOMINAL`，
  默认 launch 必须显式传 `uncalibrated:=true` 才启动。

## 4. FAST-LIO 消费者语义（[S]）

| 关注点 | 源码事实 | 行号 |
| --- | --- | --- |
| 订阅入口 | 只有 `preprocess.lidar_type == AVIA(1)` 时才订阅 `livox_ros_driver2/CustomMsg`；否则订阅 PointCloud2 | `laserMapping.cpp:921-929` |
| CustomMsg 预处理 | 无论 `lidar_type` 写得几，CustomMsg 都走 `avia_handler` | `preprocess.cpp:47-51` |
| 逐点时间 | `curvature = offset_time / 1e6`（ms，float 精度约 0.06 µs@100 ms） | `preprocess.cpp:180-181` |
| 点过筛 | `line < N_SCANS`、`(tag & 0x30) ∈ {0x00, 0x10}`、`point_filter_num`、`blind` 半径 | `preprocess.cpp:168-193` |
| IMU 初始化 | `mean_acc` 只用于**重力方向**：`grav = -mean_acc/|mean_acc| * G_m_s2` | `IMU_Processing.hpp:156-193` |
| 加速度归一化 | **`acc_avr = acc_avr * G_m_s2 / mean_acc.norm()`** ⇒ 原始 g 与 SI 都会被吸收成同一量级 | `IMU_Processing.hpp:260` |
| 协方差 | `:359` 的 `G/|mean_acc|` 缩放随后被 `:362` 的 `cov_acc_scale` **覆盖** ⇒ 不能声称协方差随单位自动换算 | `IMU_Processing.hpp:359,362` |
| 去畸变 | 先按 `curvature` 排序**副本**再逐点补偿 ⇒ 帧内乱序不影响单点补偿时间 | `IMU_Processing.hpp:224-225,304-336` |
| 帧尾 | `lidar_end_time = 帧base + points.back().curvature/1000`；`<0.5*mean_scantime` 时退回均值 | `laserMapping.cpp:394-410` |
| IMU 配对门槛 | 需要 `last_timestamp_imu >= lidar_end_time` 才组装一帧 | `laserMapping.cpp:415-418` |
| 时间同步 | `time_sync_en` 打开时**自己估** lidar-IMU 偏置（首帧 +0.1 s 偏置） | `laserMapping.cpp:333-338` |
| 外参状态 | `offset_T_L_I/offset_R_L_I` 参与状态；`extrinsic_est_en` 决定是否在线估计 | `laserMapping.cpp:780,895-897` |
| 输出 | `/Odometry`(camera_init→body)、`/cloud_registered`(camera_init)、`/cloud_registered_body`(body)、`/path`；TF `camera_init`→`body` | `laserMapping.cpp:628-664,489-560,930-936` |

**关于“乘不乘 9.81”的明确回答**（本轮最容易被误做的转换）：

* 输入 g（`|mean_acc| ≈ 0.995`）→ FAST-LIO 内部比例 `9.81/0.995 ≈ 9.86`，得到 SI 加速度；
* 输入 m/s²（`|mean_acc| ≈ 9.81`）→ 比例 ≈ 1.00，仍是 SI；
* 两种输入在归一化后**等价**（测试 `test_acc_scale_for_raw_g_and_si_agree_after_fastlio` 验证偏差 < 0.5%）。
* 所以在适配器里再乘 9.81 属于**重复转换**：虽然 FAST-LIO 会把总尺度再吸收一次而“看起来没坏”，
  但它会掩盖真实单位、并让“原始话题”与“算法输入”之间多一层不可核查的隐含换算。**本包不做该换算。**
* 副作用必须写清：FAST-LIO 的归一化把 `|mean_acc|` 的任何**真实标度误差**（如 0.5%）
  一并吸收进重力假设，因此不能反过来用 LIO 输出证明 IMU 标度正确。

## 5. 冻结 driver ↔ FAST-LIO 兼容性矩阵

| 项目 | driver 输出 | FAST-LIO 期望 | 结论 |
| --- | --- | --- | --- |
| 消息类型 | CustomMsg | `lidar_type==1` 时订阅 CustomMsg | ✅ 需配置 `lidar_type: 1` |
| 逐点时间 | ns，相对帧 base | `curvature = offset_time/1e6` ms | ✅ 单位/基准一致 |
| 帧内乱序 | 存在（150/150 帧） | 去畸变前排序副本 | ✅ 不影响逐点补偿 |
| 帧尾 | 末**追加**点 | `points.back()` | ⚠️ 本数据 300/300 帧末点=最大；但**过滤后**末保留点可能早 0–0.457 ms → patch 0002 |
| `line` | 0–3 | `line < N_SCANS` | ✅ 需 `scan_line: 4` |
| `tag` | 0x20 类占 1.01% | 只收 0x00/0x10 | ⚠️ 大气噪点类被丢（上游设计，记录而非修改） |
| acc 单位 | g | 自动归一化 | ✅ 无需换算 |
| gyro 单位 | rad/s | rad/s | ✅ |
| orientation | 无效 | 不读 | ✅ |
| frame_id | 点云 `lidar_link`、IMU `livox_frame` | 只用时间戳，不按 frame 查 TF | ✅（但下游需知道输出是 `camera_init`/`body`） |
| 时间同步 | NoSync（主机 epoch+steady） | `time_sync_en: false` | ✅ 不能开 `time_sync_en` |

## 6. 最小适配（tracked patch，可复现）

两个 patch 在 `extensions/dependencies/patches/fast_lio/`，由构建脚本自动应用/校验/可撤销：

| patch | 原因 | 影响面 |
| --- | --- | --- |
| `0001-deps-pcl_ros-to-tf2_ros.patch` | 上游 `CMakeLists.txt` 声明的 `pcl_ros` **全仓库 0 处引用**且本机未安装（禁止 apt 全局安装）；而 `laserMapping.cpp:59` 实际 include 的 `tf2_ros` 反而没声明（原先靠 pcl_ros 传递依赖） | 仅 CMake/package.xml 依赖行 |
| `0002-frame-end-from-max-point-offset.patch` | 帧尾原取 `points.back().curvature`；逐点按到达序追加，末点不保证最大。改为对**实际保留点**求 max curvature，并用于 `lidar_mean_scantime` 统计 | 只改 `sync_packages()` 帧尾参考；**不排序消息、不改逐点 offset_time、不改下采样集合** |

patch 0002 的实测影响（30 s 窗口）：300 帧中 94 帧（31.3%）的帧尾会被推后，
p90 = 0.273 ms、最大 0.457 ms（相对 100 ms 帧 = 0.27%–0.46%）。
**没有**在本数据中发现“数组末点非最大”的情形，所以该 patch 的收益主要来自
“过滤后末保留点 ≠ 最大保留点”，这一条与 MID-360S 无关，是 FAST-LIO 通用行为。

## 7. 依赖锁定与构建

* 依赖清单：`extensions/dependencies/fast_lio.repos`（主仓 + 子模块精确 SHA、许可状态）。
* 构建脚本：`extensions/tools/build_fast_lio_underlay.sh`
  * 无 sudo；源码在 `extensions/third_party/fast_lio_ws`（已 gitignore，不入库）；
  * 自检：Humble、`/usr/bin/cmake`、`/usr/bin/g++`、冻结 livox underlay、主仓 SHA、子模块 SHA+干净度；
  * 工作区状态机：干净 → apply；已应用 → **除 patch 触碰文件外任何 dirty 都拒绝**，逐字节比对
    「冻结 SHA + patch」；拒绝时不修改工作区；
  * 稀疏安全：不做整树 `git archive`（避免触发 doc/ 大文件下载），只用 `git show HEAD:<path>` 建期望树；
  * 并发：`--parallel-workers 1` + `CMAKE_BUILD_PARALLEL_LEVEL`/`MAKEFLAGS` 限制单包内部编译作业数；
  * 构建后断言：patch 生效（`frame_end_curvature` 存在、`points.back().curvature` 消失、`pcl_ros` 消失、
    `tf2_ros` 显式声明）、逐点时间语义未被改写、`ldd` 无 `not found`。

复现（干净 shell，避免 conda cmake 污染）：

```bash
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
extensions/tools/build_fast_lio_underlay.sh          # 默认 PARALLEL=4
extensions/tools/build_fast_lio_underlay.sh --clean  # 需要重编时
extensions/tools/build_fast_lio_underlay.sh --revert-patches  # 恢复冻结 SHA
```

## 8. 自研输入层（`hei_robot_localization`）

| 文件 | 作用 |
| --- | --- |
| `lio_contract.py` | 输入契约唯一事实来源（纯函数）：外参符号、IMU 归一化语义、帧尾/乱序统计、静止判定、未知项 |
| `mid360s_lio_params.py` | 参数集与门控校验（`lidar_type=1`、`scan_line=4`、`time_sync_en=false`、`extrinsic_est_en=false`、名义外参） |
| `lio_input_audit.py` | 只读审计节点：一个窗口后输出 JSON（不改单位、不转发、不发 TF） |
| `launch/fast_lio_mid360s_offline.launch.py` | 离线回放：强制 `uncalibrated:=true` + `ROS_DOMAIN_ID=88` + `ROS_LOCALHOST_ONLY=1` + bag 目录校验 |
| `config/mid360s_fastlio_uncalibrated.yaml` | FAST-LIO 参数（名义外参、禁止在线估计、不写 PCD） |

**本包不发布任何 TF**，输出契约完全交给 `fastlio_mapping`：`camera_init`→`body` 唯一发布者。
`body` 是 IMU 体坐标，**不是** `base_link`；`base_link` 与 `map` 本轮一律不发。

## 9. 实测与测试结果（本轮真实执行）

### 9.1 构建

| 项 | 结果 |
| --- | --- |
| `build_fast_lio_underlay.sh` | exit 0；1 package finished，**1min 21s**；`ldd` 无 not found；patch SHA256 已记录 |
| `colcon build --packages-select hei_robot_localization` | exit 0 |
| `bash -n` 构建脚本 | exit 0 |

### 9.2 回归测试

`colcon test --packages-select hei_robot_localization` → **52 tests, 0 errors, 0 failures, 0 skipped**（末轮报告字段修正后由 Codex 重建并独立复跑）。
覆盖：外参符号/矩阵语义、g↔SI 归一化等价性、协方差 caveat、帧尾 max 语义、乱序统计不改输入、
参数门控（改 lidar_type / time_sync / extrinsic / 缺失键都要报错）、launch 静态护栏
（未标定 opt-in、离线隔离、无 TF 发布器、无硬件入口）、**构建脚本 patch 状态机行为回归**
（临时仓库：未知 tracked/untracked 必须拒绝且前后工作区不变、篡改必被发现、可逆），
**节点启动/停止生命周期**（进程组 SIGINT→TERM→KILL 有界升级、无孤儿）、
**spin 异常契约**（运行期异常必须传播，只有 context 已关停才允许降级）。

### 9.3 离线冒烟（Domain 88 / localhost only，2 倍速，只回放已有 bag）——早期退出受限，最终复验见 §9.3.1

| 轮次 | 回放数据 | 收到输出 | 停止方式 | 观察到的结果 |
| --- | --- | --- | --- | --- |
| smoke01 | ~60 s | /Odometry **522** 条、点云 **1044** 条（两路点云各 522 帧） | 对**进程组** SIGINT（错误做法） | 双重信号：fastlio_mapping exit -2、审计节点关停抛异常 → **仅作陷阱记录，不作任何通过证据** |
| smoke03 | ~44 s | /Odometry **790** 条（全部有限、时间戳单调）、点云 **1578** 条（仅元数据：frame = camera_init/body、点数 120–2182）、odometry 位置绝对值 ≤ **0.102 m** | 只对 launch 父进程 SIGINT | **SIGINT 实际被忽略**：/proc/<pid>/status 的 SigIgn=0000000001001006。父进程 exit 143 是 **Codex 有界 TERM 清理**，不是算法退出 |
| smoke04 | ~50 s | /Odometry **890** 条（全部有限、时间戳单调）、点云 **1778** 条（仅元数据）、odometry 位置绝对值 ≤ **0.0999 m** | 同一 SIGINT 缺陷 + 有界等待/升级 | 子节点在父进程被清理后自行结束；最终 NO_RESIDUAL |

**根因（已定位）**：后台 & 启动的 shell 会把 SIGINT/SIGQUIT 置为 SIG_IGN，子进程继承该处置；
setsid 不会重置信号处置，因此“对父进程发 SIGINT”在这类 harness 里是空操作。
正确做法：Python Popen(..., preexec_fn=...) 中把 SIGINT/SIGQUIT 显式恢复为 SIG_DFL，
并且只对父进程发**一次** SIGINT 让它转发给子节点（killpg 会造成二次信号）。

**因此 §9.3 中两轮自建 harness 的退出行为一律不作为验收证据**；退出/回收行为改由 Codex 独立 harness 验证，结果见 §9.3.1（**exit 0、三子进程 cleanly、无 escalation**）。

运行中 ros2 node list 恰好 4 个节点：/fastlio_mapping、/lio_input_audit、
/lio_output_collector、/rosbag2_player —— 审计节点**唯一**。
fastlio_mapping 日志出现一次 IMU Initial Done 与一次 Initialize the map kdtree。

**有限性口径**：早期 collector 的 odom_all_finite 只覆盖 /Odometry 的 position/quaternion；**点云 XYZ 的非有限检查只在 §9.3.1 的 Codex 最终逐点验证中做过**，不得用早期元数据冒充。

**边界**：这是**静止数据**的离线冒烟。断言仅限“有限数值、时间合理、frame 契约、节点唯一”，
**不**证明里程计精度，**不**构成运动验收，**不**改变 03-LIO 的 UNKNOWN 状态。

### 9.3.1 Codex 独立复跑（最终证据，退出行为以此为准）

Codex 用独立 harness（Python Popen + preexec_fn 恢复 SIGINT/SIG_DFL，只对父进程发一次 SIGINT）
复跑离线回放，结果 **PASS**：

| 项目 | 结果 |
| --- | --- |
| launch 退出 | **exit 0** |
| 子进程 | 3 个子进程全部 cleanly 退出；**无 escalation、无 process died** |
| /Odometry | **420** 条，覆盖源时间 **41.895 s**，全部有限、时间戳单调、四元数模 ≈ 1 |
| /cloud_registered | **419** 帧，检查 **56,970** 点，XYZ 非有限 0，时间戳单调 |
| /cloud_registered_body | **419** 帧，检查 **792,839** 点，XYZ 非有限 0，时间戳单调 |
| TF | 仅 camera_init → body（无 base_link / map） |
| 审计节点 | 仅在 context 已关停后出现 take_message 警告，退出 0；运行期异常仍会传播（有单测固定） |

原始结果：/home/robo/hei-data/mid360s_lio_20261017/codex_smoke/result.json（由 Codex 自归档）。
Codex 同时独立复跑 143 项 bringup 测试通过，并独立运行本包 51 项测试。

仍需强调：这是**静止 bag** 的离线回放，只证明“有限数值 / 时间单调 / frame 契约 / 可正常退出与回收”，
**不**证明里程计精度，**不**构成运动验收，**不**改变 03-LIO 的 UNKNOWN 状态。
### 9.4 审计节点实测（输入契约的活体核对）

窗口内：`|acc|` 均值 0.993–0.995（判为 g）；**比力单位方向**（specific force，约指向 +up）≈ (0.029, -0.043, 0.999)；**重力单位方向是其相反数**。JSON 键为`specific_force_unit_direction` 与显式取负的 `gravity_unit_direction`；
gyro `|ω|` ≈ 0.007 rad/s、orientation 唯一值恒 `[0,0,0,1]`、静态判定 True、
`header == timebase` 全帧成立、帧内逆序与最大回退与离线统计一致（467,931 ns）。
FAST-LIO 内部比例 `9.81/0.995 ≈ 9.86`，与 §4 推导一致。

## 10. 未执行 / UNKNOWN / 现场最少操作

**未执行**：硬件采集、任何运动、现场外参标定、RTAB-Map、AGX 部署、导航、commit/push。

**UNKNOWN（不得当作已知）**：

1. IMU–LiDAR 现场外参（`R_IL` 偏航/安装偏角、`t_IL` 实测值）；
2. IMU 轴符号与安装方向（手册只声明“同向”，单一静止姿态无法证明）；
3. NoSync 的真实同步误差边界（主机 epoch+steady 映射，非硬件 PTP；未做双流对齐实验）；
4. IMU 噪声密度/随机游走/bias 稳定性（未做 Allan 方差）；
5. `base_link` 与 `body` 的安装外参；
6. 固件版本、供电/线束记录（02-LIVE 遗留）；
7. 上游/子模块**许可状态**（见 §1，需人工确认后才能分发）。

**现场最少操作（交给用户/后续任务）**：

1. 在固定安装状态下采集多姿态数据做标定。**六面静态只验证 acc 轴/单位/bias，不会单独给出完整 6-DoF 外参**；完整 `R_IL`/`t_IL` 需要充分旋转+平移激励的动标（如 LI-Init），或在厂商名义值基础上做独立验证后才替换名义值；
2. 用固定小推车采集直线/横移/转弯/静止/闭环运动数据（需运动授权与现场人员）；
3. 复核 IMU 轴符号（摇动试验/已知转向）与时间同步边界（PPS 或双流对齐）；
4. 之后才允许讨论 04-SLAM 与 `map`→`odom` 归属。

## 11. 复现指令

```bash
# 1) 构建（一次）
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
extensions/tools/build_fast_lio_underlay.sh

# 2) 自研包构建 + 测试
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/third_party/fast_lio_ws/install/local_setup.bash
cd extensions/ros2_ws
colcon build --packages-select hei_robot_localization
colcon test  --packages-select hei_robot_localization && colcon test-result --verbose

# 3) 离线冒烟（务必隔离；只回放已有 bag）
cd ../..
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 launch hei_robot_localization fast_lio_mid360s_offline.launch.py \\
    bag_path:=/home/robo/hei-data/mid360s_live_20261017_retest01 \\
    uncalibrated:=true rate:=2.0 audit_seconds:=6.0 \\
    audit_report:=/home/robo/hei-data/mid360s_lio_20261017/audit.json
```

停止：对 **launch 父进程**发一次 SIGINT（不要对进程组重复发），等待子节点退出后
`pgrep -af "fastlio_mapping|lio_input_audit|rosbag2_player"` 必须为空。

## 12. 交付清单

| 路径 | 说明 |
| --- | --- |
| `extensions/dependencies/fast_lio.repos` | 主仓 + 子模块精确 SHA、许可状态、获取方式 |
| `extensions/tools/build_fast_lio_underlay.sh` | 无 sudo 可复现构建（SHA/patch/子模块/并发/校验） |
| `extensions/dependencies/patches/fast_lio/0001-deps-pcl_ros-to-tf2_ros.patch` | 依赖最小纠正 |
| `extensions/dependencies/patches/fast_lio/0002-frame-end-from-max-point-offset.patch` | 帧尾最小修复 |
| `extensions/ros2_ws/src/hei_robot_localization/` | 契约模块、审计节点、离线 launch、参数、测试、README |
| `docs/MID360S_LIO.md` | 本文 |
| `docs/agent/evidence/mid360s/20261017-lio/` | 原始输出与数值证据索引 |

## 12. 最终接入验收（2026-10-17）

03-LIO-BASE **DONE / PASS**，03-LIO总体 **PARTIAL**。接入包最终52项、既有bringup143项独立回归通过；离线冒烟420条odom及两路各419帧点云、849,809点XYZ全有限，退出0。详见 [独立证据索引](agent/evidence/mid360s/20261017-lio/README.md)。
报告方向字段修正不改变LIO数值链路，历史JSON旧键仅作为原始证据保留，当前报告同时明确比力/up和取负的重力方向。原始数据与Livox产物未修改；本轮未commit/push。
