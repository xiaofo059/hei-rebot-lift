# MID-360S 接入与原始数据验收（笔记本侧）

> 本文是 MID-360S 分阶段接入的操作与验收依据：阶段路线、依赖冻结 SHA、
> **操作指令（逐段标注已执行 / NOT_RUN）**、网络重叠风险、录包与离线验收阈值、故障排查。
>
> 架构与安全边界以 [README_FOR_AGENTS.md](../README_FOR_AGENTS.md) 为准；
> Git/双机流程以 [git_workflow_zh.md](git_workflow_zh.md) 为准；
> 多 Agent 分工与验收分层以 [MULTI_AGENT_WORKFLOW.md](MULTI_AGENT_WORKFLOW.md) 为准；
> 任务台账与阶段状态以 [agent/TASK_BOARD.md](agent/TASK_BOARD.md) 为准（Codex 单写）。
>
> **状态快照 2026-10-16（Asia/Shanghai）**。本文不固化本机 IP 作为配置来源；
> 现场值一律放被忽略的 `extensions/config/local/`。
>
> **验收口径（Codex 2026-10-16 追加）**：02-PREP 软件准备已独立审查验收：
> pytest 126 项通过，5 个异常合成 bag 均被拒绝、正常合成对照通过。
> 证据见 [Codex 验收记录](agent/evidence/20261016-mid360s-codex-review.json)；真实雷达相关项仍一律 NOT_RUN。

## 1. 阶段路线与当前状态

阶段编号与 [TASK_BOARD](agent/TASK_BOARD.md) 的 MID-360S 任务表一致；
接线、录包、回放等属于 **02-LIVE 的子步骤**，不单独编号。

| 阶段（TASK_BOARD） | 内容 | 状态（2026-10-16） |
| --- | --- | --- |
| **01-SOFTWARE** | 软件/网络只读基线（真实环境、ROS 工具、依赖可行性） | **DONE**（Codex 复核接受） |
| **01-HARDWARE** | 型号/固件、供电接线、实际 IP | **BLOCKED**（用户确认雷达尚未连接） |
| **02-PREP** | 不依赖实物的软件准备：SDK/driver underlay、自研 bringup、配置生成、录包与验收工具 | **DONE**（Codex 独立复审 RF-LIDAR-001～005 关闭，仅软件准备） |
| **02-LIVE** | 接线后真实调通：只读连通性、话题频率、RViz、短包检查、30 分钟原始包与回放 | **BLOCKED / NOT_RUN**（等用户接好雷达） |
| **03-LIO** | FAST-LIO2 里程计与建图 | PLANNED，本任务不含 |
| **04-SLAM** | RTAB-Map 三维闭环、地图与重定位 | PLANNED，本任务不含 |
| **05-AGX** | AGX aarch64 部署与固定安装 | PLANNED，本任务不含 |
| **06L/06A-BRIDGE** | LeRobot↔ROS 桥接（假 host → 实机） | PLANNED，本任务不含 |
| **07L/07A-NAV** | 二维导航地图、Nav2 与实机导航 | PLANNED，本任务不含 |

**结论口径**：02-PREP 只证明「软件可构建、可导入、参数正确、验收工具不虚报」。
它**不**证明雷达可用、时间同步正确、点云物理正确，也不包含任何 LIO/SLAM/导航能力。

## 2. 交付物

| 路径 | 说明 |
| --- | --- |
| `extensions/dependencies/livox.repos` | 两个第三方仓库的 40 位 SHA 冻结清单 |
| `extensions/tools/build_mid360s_underlay.sh` | 无 sudo 构建 SDK2 + driver underlay，含环境/SHA/工作区自检 |
| `extensions/ros2_ws/src/hei_robot_bringup/` | 自研 ament 包：launch、配置生成、bag 验收、测试 |
| `docs/agent/evidence/mid360s/` | 本任务原始证据（构建、集成检查、合成数据验收） |

`.gitignore` 已补 `/extensions/third_party/` 与 `/extensions/config/local/`，
避免第三方源码与本机现场配置被误提交。

## 3. 依赖与冻结 SHA

| 组件 | 版本 | 冻结 SHA |
| --- | --- | --- |
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | tag `v1.3.1` | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | tag `1.2.6` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |

**为什么这两个版本**（S 型支持证据，均已核对冻结源码）：

- SDK2 `include/livox_lidar_def.h:69`：`kLivoxLidarTypeMid360s = 35`；CHANGELOG v1.3.1「Support Mid-360s Lidar」。
- driver `src/comm/pub_handler.cpp:135` 同时处理 `kLivoxLidarTypeMid360` 与 `kLivoxLidarTypeMid360s`；
  自带 `config/MID360s_config.json` 与 `launch_ROS2/msg_MID360s_launch.py`；CHANGELOG 1.2.5「Support Mid-360s Lidar」。

⚠️ **不能用旧 MID360（非 S）配置冒充 S 型**：S 型的 `Mid360s` 顶层键与数组形式
`host_net_info` 和 MID360 的对象形式不同；`hei_robot_bringup` 的配置校验会直接拒绝
MID360 结构，避免「启动成功但收不到点云」。

### 3.1 不使用 vcstool 的精确获取方式（新机器可复现）

`vcstool` **不是必须**（本机未安装）。按 SHA 精确获取、**不覆盖已存在目录**：

```bash
# 从仓库根目录开始
mkdir -p extensions/third_party
cd extensions/third_party

# Livox-SDK2（tag v1.3.1）
[ -e livox_sdk2_src ] && echo "livox_sdk2_src 已存在，跳过（本命令不覆盖）" || {
  git clone https://github.com/Livox-SDK/Livox-SDK2.git livox_sdk2_src
  git -C livox_sdk2_src checkout --detach f5d9375f84efe2b15bc0a052d3e18482ed13adf4
}

# livox_ros_driver2（tag 1.2.6）
mkdir -p livox_ws/src
[ -e livox_ws/src/livox_ros_driver2 ] && echo "driver 已存在，跳过（本命令不覆盖）" || {
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ws/src/livox_ros_driver2
  git -C livox_ws/src/livox_ros_driver2 checkout --detach 13eb05e4e6dd7a765b934d0c5fd6236676a57b49
}

# 校验（必须与 livox.repos 一致）
git -C livox_sdk2_src rev-parse HEAD
git -C livox_ws/src/livox_ros_driver2 rev-parse HEAD
cd ../..
```

> 若有 `vcstool`，等价命令为
> `vcs import extensions/third_party < extensions/dependencies/livox.repos`。
> 构建脚本会自己再次校验 SHA，并拒绝「已跟踪文件被本地修改」的工作区。

## 4. 操作指令（逐段标注执行状态）

> 标注含义：**[已执行]** = 本机 2026-10-16 实际运行并留存输出；
> **[NOT_RUN]** = 需要真实雷达或现场条件，本任务未执行、也不得声称验证过。

### 4.1 进入干净 ROS shell — [已执行]

本机 `PATH` 里 conda base 的 **cmake 4.1.3** 会覆盖系统 cmake 3.22.1，而 Livox-SDK2 的
`cmake_minimum_required(VERSION 3.0)` 在 CMake ≥ 4 被直接拒绝
（`Compatibility with CMake < 3.5 has been removed`）。因此**不要**在 conda 环境里构建：

```bash
cd /home/robo/robotopia/hei-rebot-lift
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    PATH=/usr/bin:/bin bash --noprofile --norc
source /opt/ros/humble/setup.bash
```

> 用 `source /opt/ros/humble/setup.bash` 建立 ROS 环境；**不要**手写 `PYTHONPATH` 代替。
> `rclpy` 位于 `/opt/ros/humble/local/lib/python3.10/dist-packages`，只设
> `.../lib/python3.10/site-packages` 会缺 `rclpy`。

### 4.2 构建第三方 underlay（无 sudo）— [已执行]

```bash
# 仍在仓库根目录
extensions/tools/build_mid360s_underlay.sh          # 增量
extensions/tools/build_mid360s_underlay.sh --clean  # 全量重建
```

脚本行为与官方 `build.sh` 的等价关系与差异：

- 官方 `build.sh` 会 `rm -rf ../../build/ ../../devel/ ../../install/`，**只能在独立
  `livox_ws` 里执行**，不能放进自研 workspace。本脚本只清理自己的 `livox_ws` 目录，
  且**只删除本脚本本次创建的临时 `launch/`**；若该目录预先存在则直接报错退出。
- 官方用 `colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble`；本脚本保持
  相同参数，并额外显式绑定本项目内的 SDK：

  ```text
  -DCMAKE_PREFIX_PATH=<...>/livox_sdk2_install
  -DLIVOX_LIDAR_SDK_LIBRARY=<...>/lib/liblivox_lidar_sdk_shared.so
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=<...>/include
  -DCMAKE_INSTALL_RPATH=<...>/livox_sdk2_install/lib -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
  ```

  官方 `CMakeLists.txt:249` 把 `/usr/local/lib` 写进 `find_library`；显式传入上述变量后
  **不需要**全局安装 SDK，也不依赖 `/usr/local`。
- SDK 安装前缀：`extensions/third_party/livox_sdk2_install`（**无 sudo**）。
- 脚本校验：cmake 必须是 `/usr/bin/cmake`、两个仓库 HEAD 必须等于 `livox.repos` 的 SHA、
  **已跟踪文件不得有本地修改**（否则构建结果不对应冻结版本）；
  最后对 node 与组件库都跑 `ldd`，`ldd` 自身失败或出现 `not found` 均直接失败。
- 第三方源码**未做任何修改**（脚本跑完 `git status` 仍干净）。

### 4.3 构建自研包 — [已执行]

```bash
# 从仓库根目录开始
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
cd extensions/ros2_ws
colcon build --symlink-install
cd ../..            # 回到仓库根目录，后续命令的相对路径才成立
```

### 4.4 生成现场配置 — [已执行]（工具，占位地址）；现场真实地址生成 [NOT_RUN]

```bash
# 从仓库根目录开始
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 run hei_robot_bringup mid360s_make_config \
    --host-ip <本机有线网卡IP> --lidar-ip <雷达IP> \
    --output extensions/config/local/mid360s_config.json
```

工具特性：`--host-ip/--lidar-ip` **必填且无示例默认值**；拒绝非法/组播/未指定/广播/环回
地址与 host==lidar；**不覆盖已有文件**；只写入被 git 忽略的目录（用 `git check-ignore`
判定，规则不会与 `.gitignore` 漂移）；首次运行会自行创建缺失的多级目录。
`--print-only` 可只打印不落盘。

### 4.5 启动（**需要雷达已接、且用户已授权**）

仅解析参数、不启动设备 — **[已执行]**：

```bash
# 从仓库根目录开始
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 launch hei_robot_bringup mid360s_bringup.launch.py --show-args
```

真实启动 — **[NOT_RUN]**（雷达未连接）：

```bash
ros2 launch hei_robot_bringup mid360s_bringup.launch.py \
    config_path:=$PWD/extensions/config/local/mid360s_config.json \
    pointcloud_format:=pointcloud2 publish_freq:=10.0 launch_rviz:=true
```

- `config_path` **必填**；缺失时 launch 在启动任何节点前失败，并提示如何生成配置。
- 同时**只有一个** driver 实例；原始点云帧 `lidar_link`；**不发布**任何 base_link/IMU 外参 TF。

### 4.6 短包检查（1–2 分钟）— [NOT_RUN]

短包只用于「链路是否通、类型/频率是否正常」，**不能**替代正式验收，也不得据此宣称通过。

```bash
# 从仓库根目录开始（三层 source 同 4.5）
ros2 topic hz /livox/lidar          # 只读观察
ros2 topic hz /livox/imu
ros2 bag record -o data/mid360s/short_$(date +%Y%m%d_%H%M%S) /livox/lidar /livox/imu
ros2 run hei_robot_bringup mid360s_bag_qc --bag data/mid360s/short_<时间戳> \
    --expected-duration 60
```

### 4.7 正式原始包（30 分钟验收）— [NOT_RUN]

**正式包必须录 CustomMsg**（含逐点 `offset_time`），因此流程是：
先停掉正在显示 PointCloud2 的那个 driver，再用 `pointcloud_format:=custom` 单独启动
**同一个** driver（任何时刻只允许一个 driver 占用雷达），然后录包。

```bash
# 1) 在跑 pointcloud2 的终端按 Ctrl+C，确认没有残留 driver
pgrep -x livox_ros_driver2_node || echo "无残留 driver"
ros2 topic list | grep -c '^/livox/' || true

# 2) 用 custom 格式重新启动唯一 driver（新终端，从仓库根目录开始）
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 launch hei_robot_bringup mid360s_bringup.launch.py \
    config_path:=$PWD/extensions/config/local/mid360s_config.json \
    pointcloud_format:=custom publish_freq:=10.0

# 3) 另一个终端录包（存在的 /tf /tf_static 一并录；若不存在则从命令中删掉它们）
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 bag record -o data/mid360s/raw_$(date +%Y%m%d_%H%M%S) \
    /livox/lidar /livox/imu /tf /tf_static
```

- **建议录 31 分钟**：覆盖启停边界，确保**有效覆盖 ≥ 1800 s**（判据仍按 1800 s 计）。
- 磁盘估算：约 0.5 MB/帧 × 10 Hz ≈ 5 MB/s，30 分钟约 9–10 GB（估算，随场景变化；
  录前 `df -h`）。大批量原始包放仓库外数据目录（如 `~/hei-data/`），不要提交 Git。
- 录到的 IMU 是**原始未换算值**：`linear_acceleration` 单位是 **g**（不是 m/s²），
  `orientation` 未发布；原始包按原值保存，本阶段不做 SI 转换（见 §7.2）。

### 4.8 回放 — [NOT_RUN]

```bash
# 从仓库根目录开始（三层 source 同 4.5）
ros2 bag info data/mid360s/raw_<时间戳>
ros2 bag play data/mid360s/raw_<时间戳> --clock
```

回放**尚未执行过一次**；本任务不得声称回放链路已验证。回放属 02-LIVE 之后的内容。

### 4.9 离线验收 — [已执行]（**SYNTHETIC 合成数据**）

以下命令在本机对**合成 bag** 跑通过（FAIL/INCONCLUSIVE/PASS 三条路径都验证过），
**不代表任何真实数据已验收**：

```bash
# 从仓库根目录开始（三层 source 同 4.5）
ros2 run hei_robot_bringup mid360s_bag_qc \
    --bag data/mid360s/raw_<时间戳> \
    --pointcloud-format custom \
    --expected-duration 1800 --lidar-hz-min 9 --lidar-hz-max 11 --max-gap 0.5 \
    --imu-hz-min <标称下限> --imu-hz-max <标称上限> \
    --json-out docs/agent/evidence/mid360s/qc_<日期>.json
```

退出码：`0` PASS ｜ `1` FAIL ｜ `2` 用法/输入错误 ｜ `3` INCONCLUSIVE（缺少判据或做了采样）。

## 5. 网络：重叠风险与接线后的下一步

### 5.1 重叠风险（2026-10-16 本机快照，现场必须重新核对）

| 接口 | 快照值 | 说明 |
| --- | --- | --- |
| `enp7s0`（有线） | `192.168.50.1/24` | 雷达应接在这一侧；同网段已有设备 `192.168.50.2` |
| `wlp0s20f3`（WiFi） | `192.168.1.144/23` | 路由表含 `192.168.0.0/23 dev wlp0s20f3` |
| 默认路由 | 经 WiFi | 有线侧无默认路由 |

**关键冲突**：WiFi 的 `/23` 覆盖 `192.168.0.0–192.168.1.255`，而 Livox 官方示例配置常用
`192.168.1.x`。若照抄官方示例地址：发往 `192.168.1.x` 的报文会按路由表从 **WiFi** 出去，
不会走有线网卡；雷达即使接好，driver 也收不到点云（表现为「`Init lds lidar successfully`
后无数据」），并可能与本机 `192.168.1.144/23` 产生 ARP/地址冲突。
因此 `hei_robot_bringup` **不内置任何示例 IP**，强制填写现场真实地址。

### 5.2 本轮的约束边界

**本轮（`01-HARDWARE` 未完成、雷达未接）明确禁止修改网卡/IP/路由/防火墙**——
这是**本次任务范围内的约束**，用于在硬件状态未知时避免把网络改坏。

它**不是**长期规则：这不表示以后每次普通雷达接入都要重新走一遍授权。
处理原则：

1. 保持网络配置**原样**，直到用户确认已真实接线并给出雷达实际 IP；
2. 之后的地址/路由方案按**该任务的授权范围 + 现场证据**决定，优先选择**可回滚**的方案；
3. 需要改动主机网络时，先记录改前状态（`ip -br address`、`ip route`）并给出回滚命令。

### 5.3 接线后的下一步（按顺序）

1. 只读核对：`ip -br address`、`ip route`、`ip neigh`（不改任何配置）。
2. 确认雷达实际 IP（标签/官方工具读数），与用户确认采用哪个网段。
3. 用 `mid360s_make_config` 生成现场配置（`host_ip` 填有线网卡地址）。
4. 起 `mid360s_bringup.launch.py`，确认 driver 日志出现
   `Config file : ...` 与 `Init lds lidar successfully!`。
5. 只读观察数据是否在流：`ros2 topic hz /livox/lidar`、`ros2 topic hz /livox/imu`。
   注意 `/livox/imu` 的 frame 会是 `livox_frame`（上游硬编码，见 §7.1）。
6. 短包（§4.6）→ 正式 31 分钟包（§4.7）→ 离线验收（§4.9）。

> DDS/Domain ID 与雷达网段是**两件独立的事**：设了 `ROS_DOMAIN_ID` 不代表雷达网络已通。

### 5.4 时间基准的口径

**没有 PTP ≠ 时间无效**。未锁定 gPTP 时，雷达上电后的**相对时间基准**（设备内部计时）
对点云/IMU 的时间顺序与后续建图仍可能可用。本工具据此判定：

- **可接受**：帧头时间单调、非冻结，两流时间范围重叠；
- **不合格**：帧头时间**全零**或**冻结**（所有消息时间相同）、时间倒退、两流完全不重叠。

`mid360s_bag_qc` 只做上述数值一致性检查，**不**证明硬件时间同步精度（gPTP/PPS 是否锁定）。

## 6. 验收阈值与判据（与工具当前实现一致）

| 项目 | 默认判据 | 说明 |
| --- | --- | --- |
| 帧头检查范围 | **默认全量** | 显式 `--header-sample N` 会丢失信息，整体只能 INCONCLUSIVE，**不能 PASS**；`0`/负数直接拒绝 |
| 点云覆盖时长 | ≥ 1800 s | 短包不会通过 |
| **IMU 覆盖时长** | ≥ 1800 s | 两条流都要满足，IMU 短流会 FAIL |
| 点云平均频率 | 9–11 Hz | 与 launch 默认 10 Hz 一致 |
| **两流最大断流** | ≤ 0.5 s | 点云与 IMU 各自都要满足（接收时间） |
| 帧头时间（**逐相邻对**） | 严格递增 | **中段重复/冻结**与倒退都判 FAIL；不只比较整段唯一值集合 |
| **源帧头最大间隔** | ≤ 0.5 s | 针对传感器时间跳变；接收时间可能仍然连续 |
| 接收时间倒退 | 0 次 | 注：rosbag2 sqlite3 读取时按时间戳排序，该检查对这类包基本不可触发 |
| 话题区分 | `--lidar-topic` ≠ `--imu-topic` | 相同会合并两条流统计 → 直接用法错误 |
| **两流时间范围重叠** | > 0 且重叠 ≥ 1800 s | 证明同一采集窗口；完全不重叠 → FAIL |
| 类型可导入 / 解码 | 必须成功 | CustomMsg 类型导入失败、任一消息反序列化失败 → FAIL（不再只 WARN） |
| PointCloud2 结构 | 合法非空 | `width/height > 0`、`fields` 含 x/y/z、字段不越界、`row_step = point_step×width`、`data` 长度匹配 |
| PointCloud2 点级时间 | 必须含 `timestamp` 字段 | 缺失 → FAIL |
| CustomMsg 结构 | 合法非空 | `point_num == len(points)`、点非空、逐点 `offset_time` 存在且单调不冻结 |
| IMU 频率 | **必须显式给出标称范围** | 未给出 → 整体 INCONCLUSIVE，不得判通过 |

**IMU 标称频率当前未知**（尚未接实物、未查规格书）。因此**首次真实验收无法直接给出
「整体通过」**，必须先确认 MID-360S 的 IMU 标称频率，再用 `--imu-hz-min/--imu-hz-max` 复验。

⚠️ 本工具只做**数据时间基准与消息结构的数值一致性检查**，**不能**证明硬件时间同步精度，
也**不能**替代点云/IMU 物理正确性检查；它**不做** SI 换算（见 §7.2）。

## 7. 已知限制（未修改第三方源码）

1. **IMU frame 与点云 frame 不同**：`livox_ros_driver2` 1.2.6 `src/lddc.cpp:481`
   `imu_msg.header.frame_id = "livox_frame"` 为硬编码，不受 `frame_id` 参数影响；点云
   (`lddc.cpp:263,355,421`) 才使用配置的 `frame_id`（本包默认 `lidar_link`）。
   frame 与 topic 名称本就无需同名，**这不是「frame 与 topic 不一致」**；但下游按 frame
   解析 TF 时必须显式处理该差异。本包**不发布** identity TF 去「补齐」它——那是未经测量
   的外参。后续 FAST-LIO2 需显式配置 lidar→IMU 外参，或另开任务评估上游补丁。
2. **原始 IMU 不是 SI 单位，且没有姿态**（driver 1.2.6 冻结源码核对）：
   - `src/comm/pub_handler.cpp:119-124` 把报文中的 gyro/acc **原样拷贝**，无任何换算；
   - `src/lddc.cpp:489-494` 原样写入 `sensor_msgs/Imu`，全仓库 `src/` 中**没有**
     `orientation`/`covariance` 赋值（`grep` 命中 0）；
   - `src/comm/lidar_imu_data_queue.h:37-42` driver 自述
     **gyro 单位 `rad/s`、acc 单位 `g`**（注释注明 "Based on the IMU Data Type in Livox
     communication protocol"）。
   官方协议参考（由 Codex 提供）：
   [MID-360 系列以太网协议](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)。

   ⇒ 原始 bag 中 `linear_acceleration` 是 **g**（不是 m/s²），`orientation` 恒为默认值。
   **原始 bag 保留原值，本阶段不做 SI 转换**；转换属后续阶段的显式设计决策。
   在转换与标定完成前，**不得**把原始 `sensor_msgs/Imu` 当作已符合 SI 的融合输入、
   也不得当作已具备有效姿态。`mid360s_bag_qc` 会在输出与 JSON `note` 中重复该口径。

   > 证据边界：本机 DSH 沙箱无法访问上述 wiki（`web_fetch` 返回
   > `URL hostname ... resolves to a non-public IP address`），故单位结论来自
   > **冻结源码自述**，不是本次对 wiki 的独立核对；引用链接按 Codex 提供记录。
3. **`lidar_summary_info.lidar_type` 是 driver 内部位标志 8**（`lds_lidar.cpp:107`
   `lidar_type & kLivoxLidarType`），**不是** SDK 的 S 型枚举 35。写成 35 会让 driver 走错分支。
4. **rosbag2 sqlite3 读取时按时间戳排序**（实测：写入 `1000,2000,3000,2500,4000` → 读出
   `1000,2000,2500,3000,4000`）。所以「接收时间倒退」对这类包几乎不可触发；验收以
   消息内帧头时间为准。
5. **`publish_freq` 会被 driver 静默夹取到 `[0.5, 100]`**（`livox_ros_driver2.cpp:74-81`），
   故 launch 在越界时直接拒绝而不是静默生效。
6. **`ros2 launch` 需要可写的 ROS 日志目录**（默认 `~/.ros/log`）。受限/只读环境下
   需 `export ROS_LOG_DIR=<可写目录>`，否则 `--show-args` 也会因创建日志目录失败。

## 8. 故障排查

| 现象 | 优先检查 |
| --- | --- |
| 构建报 `Compatibility with CMake < 3.5 has been removed` | 用了 conda 的 cmake；回到 §4.1 干净 shell |
| 构建报「存在已跟踪文件的本地修改」 | 第三方仓库被本地改过；先 `git -C <repo> stash` 或 `checkout -- .` |
| 构建报 `launch 已存在，拒绝删除或覆盖` | 上次异常退出的残留或用户自有内容；确认后人工删除再重跑 |
| `ros2` 报 `ModuleNotFoundError: rclpy` | 没 `source /opt/ros/humble/setup.bash`，或手写 PYTHONPATH 漏了 `local/lib/python3.10/dist-packages` |
| `ldd` 报 `liblivox_lidar_sdk_shared.so => not found` | 未用本项目的构建脚本（缺 RPATH）；重跑 `build_underlay.sh`，或临时 `export LD_LIBRARY_PATH=<sdk_install>/lib` |
| `ros2 pkg prefix livox_ros_driver2` 找不到 | 没有 source `extensions/third_party/livox_ws/install/local_setup.bash` |
| launch 报 `missing required argument 'config_path'` | 正常保护；先用 `mid360s_make_config` 生成现场配置 |
| launch 报缺 `Mid360s` 键 | 误用了 MID360（非 S）配置；用本包生成器重建 |
| `--show-args` 报 `InvalidLaunchFileError` + 只读 `~/.ros/log` | `export ROS_LOG_DIR=<可写目录>` |
| driver 日志 `Init lds lidar successfully` 但无点云 | 先查 §5.1 网段冲突（最常见）；再查雷达 IP/端口、防火墙 |
| `bag_qc` 报 `lidar.type` 不匹配 | 录制时用了 `pointcloud_format:=custom`；验收加 `--pointcloud-format custom` |
| `bag_qc` 报 `lidar.duration` / `imu.duration` 不足 | 包太短；正式包建议录 31 分钟 |
| `bag_qc` 报 `streams.header_overlap` 为负 | 两条流时间范围不重叠，不是同一采集窗口 |
| `bag_qc` 报 `*-decode` 失败 | 载荷不是合法 CDR；检查录制/回放链路，不要指望「跳过」 |
| `bag_qc` 结论 `INCONCLUSIVE` | 未给 `--imu-hz-min/--imu-hz-max`，或用了 `--header-sample` |
| `bag_qc` 报 `*-header_monotonic` | 帧头有倒退或**中段重复/冻结**（含全零）；逐相邻对检查，不是只看整段唯一值 |
| `bag_qc` 报 `*-header_gap` | 源帧头出现大跳变（接收时间可能仍连续），下游时间基准不可用 |
| `bag_qc` 报两话题不能相同 | `--lidar-topic`/`--imu-topic` 指到同一话题；必须分开 |
| 时间全零/冻结 | 雷达未获得有效时间基准；注意「未锁 PTP」本身不等于不合格（§5.4） |

## 9. 本任务的未执行项（NOT_RUN）

- 真实雷达连接、点云/IMU 实际数据、RViz 真实显示（`01-HARDWARE` BLOCKED）。
- 正式原始包录制（§4.7）与 `ros2 bag play` 回放（§4.8）：**一次都没跑过**。
- 任何 LIO/SLAM/定位/建图/导航（03-LIO 起）。
- 硬件时间同步精度（gPTP/PPS 锁定状态）、外参标定、坐标轴与安装方向核对。
- AGX 侧部署与 aarch64 构建。
- 网络配置变更（本轮明确禁止）。
- `rosdep install` / apt 安装：本机 `sudo -n true` 失败（DSH 沙箱 `no new privileges`
  标志；这与「本机 sudo 需要密码」是不同限制），故未执行任何需要提权的安装。
  `vcstool` 未安装，本任务用 `git clone` + SHA 校验代替（§3.1），不影响可复现性。

## 10. 维护

- 依赖升级：改 `extensions/dependencies/livox.repos` 的 SHA，重跑构建脚本与全部测试。
- 阶段推进时同步 §1 状态表与 [TASK_BOARD](agent/TASK_BOARD.md)（后者由 Codex 单写）。
- 现场 IP、串口、序列号等本机值放 `extensions/config/local/`（已忽略），不进本文。
