# hei_robot_localization

HEI ReBot Lift 自研定位输入层。当前只做一件事：把 **MID-360S 原始数据 -> FAST-LIO2**
的输入契约写死、审计出来，并提供一个**只回放已有 bag** 的离线启动。

> 状态（2026-10-17，任务 HEI-LIDAR-20261017-03-LIO-BASE）：
> **UNCALIBRATED / 仅离线**。外参只有厂商手册名义值，现场未标定；
> 本轮**没有**运动数据，因此**不**声称 LIO 可靠、不声称 SLAM/里程计验收通过。
> 不含 RTAB-Map、不含 map->odom、不含 base_link、不含导航。

## 目录

| 路径 | 作用 |
| --- | --- |
| `hei_robot_localization/lio_contract.py` | 输入契约的唯一事实来源（纯函数）：外参符号、IMU 单位归一化、帧尾/逐点时间、静止判定、未知项清单 |
| `hei_robot_localization/mid360s_lio_params.py` | 离线实验参数集与校验；未标定/禁止在线估计等门控 |
| `hei_robot_localization/lio_input_audit.py` | `lio_input_audit`：只读审计节点，统计一个窗口后输出结构化 JSON（不发 TF、不改单
位、不改发任何话题） |
| `launch/fast_lio_mid360s_offline.launch.py` | 离线启动：fast_lio + 审计 + 可选 bag 回放；强制 `uncalibrated:=true` 与离线隔离 |
| `config/mid360s_fastlio_uncalibrated.yaml` | FAST-LIO 参数（含名义外参、`extrinsic_est_en: false`） |
| `test/` | 无 ROS 环境即可跑的回归：契约数值、参数门控、launch 静态护栏、构建脚本 patch 状态机 |

## 输入契约（原始话题，本轮实测 + 源码核对）

| 项目 | 事实 | 依据 |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg`，frame `lidar_link` | driver `lddc.cpp:355`（frame 用配置值） |
| 逐点时间 | `offset_time` 单位 **ns**，相对本帧 `timebase`（= header.stamp） | `lddc.cpp:395`、`lddc.cpp:364-367` |
| 逐点顺序 | 按 UDP 到达序追加，**不排序**；帧内存在相邻逆序 | `pub_handler.cpp:404/432/467` + 正式包实测 |
| `/livox/imu` | `sensor_msgs/msg/Imu`，frame **`livox_frame`**（上游硬编码） | `lddc.cpp:481` |
| 加速度单位 | **g**（不是 m/s²），原样转发 | `lidar_imu_data_queue.h:37-42`、`lddc.cpp:489-494` |
| 角速度单位 | rad/s | 同上 |
| orientation | **无效**（driver 不赋值，恒为默认四元数） | `src/` 全仓库无 orientation 赋值 |
| 时间同步 | NoSync：主机 epoch 锚点 + steady 已过时间，两流共用映射；**不是**硬件 PTP | `docs/MID360S_BRINGUP.md` §3、20261017 验收 |

### FAST-LIO 如何处理这些输入

| 机制 | 结论 | 源码 |
| --- | --- | --- |
| 加速度归一化 | FAST-LIO 用初始化窗口的 `mean_acc.norm()` 把加速度缩放到 `G_m_s2 = 9.81`，**原始 g 与已换算 m/s² 都会被吸收**；因此本包**不做** g→SI 换算，也不把原始消息标成 SI | `IMU_Processing.hpp:260` |
| 重力初始化 | 只取 `mean_acc` 的**方向**，模固定 9.81 | `IMU_Processing.hpp:193` |
| 协方差 | `:359` 的 `G/|mean_acc|` 缩放随后被 `:362` 的 `cov_acc_scale` 覆盖，**不能**声称协方差随单位自动换算 | `IMU_Processing.hpp:359,362` |
| 点云入口 | 只有 `preprocess.lidar_type == 1 (AVIA)` 才订阅 CustomMsg | `laserMapping.cpp:921-929` |
| 逐点时间 | `curvature = offset_time / 1e6`（ms），用于去畸变 | `preprocess.cpp:180` |
| 去畸变顺序 | 先按 `curvature` 排序**副本**再逐点补偿，因此帧内乱序不影响单点补偿 | `IMU_Processing.hpp:224-225` |
| 帧尾 | 上游用 `points.back().curvature`，本仓库 patch 改为**实际保留点的 max**（见下） | `laserMapping.cpp:399/406/407` -> patch 0002 |

## 外参：从手册名义值到 FAST-LIO 参数

FAST-LIO 对每个点做 `p_I = R_IL * p_L + t_IL`（`IMU_Processing.hpp:327`、
`laserMapping.cpp:895-897`），配置项 `mapping.extrinsic_T` 就是 **t_IL**（LiDAR 原点在
IMU 系中的位置）。

Livox Mid-360S 用户手册（2026-06-01）§3.5 "IMU Data"（印刷页 22）写明：

> The IMU data contains the 3-axis acceleration and 3-axis angular velocity, the
> directions of which are the same with that of the point cloud coordinates. The
> position of the IMU chip in the point cloud coordinates is
> x=11.0 mm, y=23.29 mm, z=-44.12 mm.

即厂商给的是 **r_LI**（IMU 芯片在点云/LiDAR 系中的位置），方向同向意味着 R_IL = I。
因此：

```text
p_I = R_IL (p_L - r_LI) = R_IL p_L + t_IL
t_IL = -R_IL · r_LI = -r_LI = [-0.011, -0.02329, +0.04412] m     (R_IL = I)
```

**直接把芯片坐标 [+0.011, +0.02329, -0.04412] 填进 extrinsic_T 是错的**（符号全反）。
反向核对：把芯片位置当作 LiDAR 系中的点送进 `lidar_point_to_imu` 必须得到原点，
测试 `test_imu_origin_maps_to_zero_in_imu_frame` 固定了这一点。

以上是 **manufacturer nominal**，不是本机标定结果。上游 `config/mid360.yaml` 的
`extrinsic_T: [-0.011, -0.02329, 0.04412]` 与本推导一致，仅作交叉核对。

## 输出契约（由 fast_lio 自己发布，本包不发布任何 TF）

| 话题 | 类型 | frame |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/msg/Odometry` | `camera_init` -> `body` |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | `camera_init`（世界系去畸变单帧云） |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | `body`（IMU 体系） |
| `/path` | `nav_msgs/msg/Path` | `camera_init` |
| TF | `camera_init` -> `body` | 由 `fastlio_mapping` 唯一发布（`laserMapping.cpp:647-658`） |

* `body` 是 **IMU 体坐标**（FAST-LIO 状态定义在 IMU 系），不是 `base_link`。
* `base_link` 与 `body` 的相对位姿**未标定**，本包不发布该变换，也不发布任何假 TF。
* 将来 `map -> odom` 由 RTAB-Map 独占；LIO 适配器只负责 `camera_init`/`body` 这一段。

## 构建

三个 workspace 按顺序 source；第三方 underlay 不在本仓库入库
（见 `extensions/dependencies/fast_lio.repos`、`livox.repos`）。

```bash
# 1) 冻结的 Livox driver underlay（已有则跳过；本任务不重建）
source /opt/ros/humble/setup.bash
extensions/tools/build_mid360s_underlay.sh

# 2) FAST-LIO underlay（本任务新增；脚本自检 SHA/子模块/patch，无 sudo）
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
extensions/tools/build_fast_lio_underlay.sh

# 3) 本包
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/third_party/fast_lio_ws/install/local_setup.bash
cd extensions/ros2_ws
colcon build --packages-select hei_robot_localization
colcon test --packages-select hei_robot_localization
colcon test-result --verbose
```

## 离线运行（只回放已有 bag，不接硬件）

```bash
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/third_party/fast_lio_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1

ros2 launch hei_robot_localization fast_lio_mid360s_offline.launch.py \
    bag_path:=/home/robo/hei-data/mid360s_live_20261017_retest01 \
    uncalibrated:=true \
    rate:=2.0 \
    audit_report:=/home/robo/hei-data/mid360s_lio_20261017/audit_report.json

# 停止：两个终端都 Ctrl-C；确认无残留
pgrep -af "fastlio_mapping|lio_input_audit|ros2 bag play"
```

launch 会拒绝三种情况：未传 `uncalibrated:=true`；`ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY`
不是 88/1；`bag_path` 不是含 `metadata.yaml` 的目录。

## 明确未做 / 未知

* **未**做现场外参标定，**未**做 IMU 轴符号/偏航验证（单一静止姿态不足以证明）。
* **未**接入硬件采集、**未**做任何运动数据或闭环测试；**不**声称 LIO 可靠。
* **未**发布 `base_link`、`map`、`odom` 中的任何一个变换；`camera_init` 是 LIO 局部
  世界系，不是地球/地图基准。
* IMU 噪声/随机游走/bias 稳定性、NoSync 的真实同步误差边界均未标定。
