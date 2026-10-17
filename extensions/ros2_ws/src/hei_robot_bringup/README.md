# hei_robot_bringup

HEI ReBot Lift 自研 ROS 2 Humble bringup 包。当前只覆盖 **MID-360S 原始数据链路**：
参数化启动、现场配置生成、原始 rosbag 录制指令与离线验收工具。
**不含** LIO、SLAM、定位、建图、导航、MoveIt 或任何底盘控制（属 03-LIO 及以后阶段）。

## 状态（2026-10-16）

- 软件准备已完成并经 Codex Review；真实 MID-360S 已连接，PointCloud2/RViz 链路与约 15 秒点云频率观察已完成。
- 02-LIVE 尚未验收：IMU 频率/字段、CustomMsg 逐点时间、真实 rosbag/QC/回放未完成；driver Ctrl+C 退出会触发 SIGABRT。详见
  [`docs/MID360S_BRINGUP.md`](../../../../docs/MID360S_BRINGUP.md)。

## 内容

| 路径 | 作用 |
| --- | --- |
| `launch/mid360s_bringup.launch.py` | 启动**唯一**一个 `livox_ros_driver2` 实例；`config_path` 必填；默认 10 Hz PointCloud2；可选 CustomMsg / RViz |
| `hei_robot_bringup/mid360s_config.py` | 端口结构、IPv4 校验、现场配置读写与上游 IMU 单位事实的单一来源 |
| `hei_robot_bringup/make_config.py` | `mid360s_make_config`：生成现场配置 JSON（无示例 IP 默认值，不覆盖已有文件，只写被忽略目录） |
| `hei_robot_bringup/bag_qc.py` | `mid360s_bag_qc`：原始 bag 离线质量验收 |
| `rviz/mid360s.rviz` | Fixed Frame = `lidar_link`，显示 `/livox/lidar` 与 `/livox/imu` |
| `test/` | 无硬件测试（配置校验、CLI、bag 判定、launch、构建脚本护栏） |

## 构建与运行

依赖第三方 underlay（Livox SDK2 + livox_ros_driver2），见
[`extensions/dependencies/livox.repos`](../../../dependencies/livox.repos) 与
[`docs/MID360S_BRINGUP.md`](../../../../docs/MID360S_BRINGUP.md)（含不用 vcstool 的精确 SHA 获取方式）。

不要在已 source 本工作空间的终端里反复构建；三个 workspace 必须按顺序 source。
**每个命令块都从仓库根目录开始。**

```bash
# 1) 构建第三方 underlay（无 sudo；脚本自带环境/SHA/工作区自检）
source /opt/ros/humble/setup.bash
extensions/tools/build_mid360s_underlay.sh

# 2) 构建本包
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
cd extensions/ros2_ws && colcon build --symlink-install
cd ../..

# 3) 运行（config_path 必填；真实启动 NOT_RUN，需雷达已接并获授权）
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 launch hei_robot_bringup mid360s_bringup.launch.py \
    config_path:=$PWD/extensions/config/local/mid360s_config.json
```

正式原始包要求 `pointcloud_format:=custom`（含逐点 `offset_time`）：先停掉显示用的
PointCloud2 driver，再用 custom 单独启动**同一个** driver，然后录包。完整顺序见
[`docs/MID360S_BRINGUP.md`](../../../../docs/MID360S_BRINGUP.md) §4.7。

## 验收工具口径

`mid360s_bag_qc` 退出码：`0` PASS ｜ `1` FAIL ｜ `2` 用法错误 ｜ `3` INCONCLUSIVE。

关键判据：两条流各自的覆盖时长/断流/帧头单调且非冻结、两流**源帧头时间范围重叠且覆盖
要求时长**、类型可导入、解码不失败、点云与 CustomMsg 结构合法。帧头检查**默认全量**；
显式 `--header-sample` 会丢失信息，因此只能 INCONCLUSIVE、不能 PASS。
**不做** IMU 单位换算，也不证明硬件时间同步精度。

## 已知上游限制（未修改第三方源码）

1. **IMU frame 与点云 frame 不同**：`livox_ros_driver2` 1.2.6 `src/lddc.cpp:481` 写死
   `imu_msg.header.frame_id = "livox_frame"`，不受 `frame_id` 参数影响；点云与 CustomMsg
   才使用 `frame_id`（本包默认 `lidar_link`）。frame 与 topic 本就无需同名，这不是
   「frame 与 topic 不一致」。本包**不发布** identity TF 去「对齐」它——那是未经测量的外参。
2. **原始 IMU 不是 SI 单位、也没有姿态**：`src/comm/pub_handler.cpp:119-124` 原样拷贝，
   `src/lddc.cpp:489-494` 原样写入且不设 `orientation`/`covariance`；
   `src/comm/lidar_imu_data_queue.h:37-42` 自述 acc 单位 **g**、gyro 单位 **rad/s**。
   因此 `linear_acceleration` 是 **g**（非 m/s²）。原始 bag 保留原值，本阶段不做换算。
3. **`lidar_summary_info.lidar_type` 是 driver 位标志 8**，不是 SDK 的 S 型枚举 35。
4. **rosbag2 sqlite3 读取时按时间戳排序**，所以「接收时间倒退」对这类包几乎不可触发；
   真正能发现时间基准异常的是消息内帧头 `header.stamp`。
5. **`ros2 launch` 需要可写的 ROS 日志目录**（默认 `~/.ros/log`），受限环境下需
   `export ROS_LOG_DIR=<可写目录>`。

## 测试

```bash
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
cd extensions/ros2_ws && colcon test --packages-select hei_robot_bringup
colcon test-result --verbose
```

测试全部使用合成数据或纯静态检查，**不连接任何硬件**；合成数据在日志与报告中一律
标记 SYNTHETIC，不代表真实雷达/IMU 观测。
