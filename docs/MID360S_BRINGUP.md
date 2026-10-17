# MID-360S 接入与原始数据验收（笔记本侧）

> 本文是 MID-360S 分阶段接入的操作与验收依据：阶段路线、依赖冻结 SHA、
> **操作指令（逐段标注已执行 / NOT_RUN）**、网络重叠风险、录包与离线验收阈值、故障排查。
>
> 架构与安全边界以 [README_FOR_AGENTS.md](../README_FOR_AGENTS.md) 为准；
> Git/双机流程以 [git_workflow_zh.md](git_workflow_zh.md) 为准；
> 多 Agent 分工与验收分层以 [MULTI_AGENT_WORKFLOW.md](MULTI_AGENT_WORKFLOW.md) 为准；
> 任务台账与阶段状态以 [agent/TASK_BOARD.md](agent/TASK_BOARD.md) 为准（Codex 单写）。
>
> **状态快照 2026-10-17（Asia/Shanghai）**。本文不固化本机 IP 作为配置来源；
> 现场值一律放被忽略的 `extensions/config/local/`。
>
> **验收口径（Codex 2026-10-16 更新）**：02-PREP 软件准备已独立审查验收：
> pytest 126 项通过，5 个异常合成 bag 均被拒绝、正常合成对照通过。
> 证据见 [Codex 验收记录](agent/evidence/20261016-mid360s-codex-review.json)。2026-10-16 首次
> 正式实采 **FAIL**（源时间回拨/0.713s gap、退出 SIGBUS）。2026-10-17 已完成时钟及符号隔离
> 修复，独立 143 tests、SDK A/B 和真实启停通过；新正式包已完成1864.757s实采，全量 QC 与完整回放均 PASS。
> 修复源码证据见 [20261017-fix](agent/evidence/mid360s/20261017-fix/README.md)，最新结果见 §4.10。

## 1. 阶段路线与当前状态

阶段编号与 [TASK_BOARD](agent/TASK_BOARD.md) 的 MID-360S 任务表一致；
接线、录包、回放等属于 **02-LIVE 的子步骤**，不单独编号。

| 阶段（TASK_BOARD） | 内容 | 状态（2026-10-17） |
| --- | --- | --- |
| **01-SOFTWARE** | 软件/网络只读基线（真实环境、ROS 工具、依赖可行性） | **DONE**（Codex 复核接受） |
| **01-HARDWARE** | 型号/固件、供电接线、实际 IP | **部分完成**（在线与点云已确认；固件/供电/线束记录待补齐） |
| **02-PREP** | 不依赖实物的软件准备：SDK/driver underlay、自研 bringup、配置生成、录包与验收工具 | **DONE**（Codex 独立复审 RF-LIDAR-001～005 关闭，仅软件准备） |
| **02-LIVE** | 接线后真实调通：只读连通性、话题频率、RViz、短包检查、30 分钟原始包与回放 | **DONE**（2026-10-17 修复、长时数据 QC、完整回放及正常退出通过；物理标定/硬件对时另验） |
| **03-LIO** | FAST-LIO2 里程计与建图 | PLANNED，本任务不含 |
| **04-SLAM** | RTAB-Map 三维闭环、地图与重定位 | PLANNED，本任务不含 |
| **05-AGX** | AGX aarch64 部署与固定安装 | PLANNED，本任务不含 |
| **06L/06A-BRIDGE** | LeRobot↔ROS 桥接（假 host → 实机） | PLANNED，本任务不含 |
| **07L/07A-NAV** | 二维导航地图、Nav2 与实机导航 | PLANNED，本任务不含 |

**结论口径**：02-PREP 只证明「软件可构建、可导入、参数正确、验收工具不虚报」。
它**不**证明雷达可用、时间同步正确、点云物理正确，也不包含任何 LIO/SLAM/导航能力。

**真实数据进展（2026-10-16）**：笔记本 USB ASIX 接口 `enx9c69d3cd61a9` 为
`192.168.1.5/32`，MID-360S `192.168.1.135` 有直连 `/32` 路由。PointCloud2/RViz 显示正常；
CustomMsg + IMU 正式原始包录制 1866.9 秒，双流结构和 IMU 频率达标，但 LiDAR 消息源时间戳
检查失败（2 次倒退、最大间隔 0.713 秒）。完整 QC 和复现的驱动 SIGBUS 记录见 §4.7–4.9。

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
> 构建脚本会再次校验 SHA，只接受干净基线或「基线 + 已跟踪补丁」的精确内容；未知修改在应用补丁前拒绝。

### 3.2 修复补丁（2026-10-17）

两份补丁由 `extensions/tools/build_mid360s_underlay.sh` 自动应用，维护说明及 SHA 见
[patches/README.md](../extensions/dependencies/patches/README.md)。不直接覆盖冻结来源，也不依赖系统全局 SDK：

- SDK shared 库只导出 56 个公开 C API，内部 spdlog/fmt 符号保持本地，避免与 ROS 系统日志库 ABI 冲突。
- driver NoSync 时间戳以一次 system_clock epoch 锚点 + steady_clock elapsed 生成，点云/IMU 共用映射；
  非同步帧调度也用 steady_clock。PTP/GPS 设备时间路径原样保留，真实设备时间异常仍会暴露。
- 这是主机时间近似映射，并非设备硬件同步；time_type 动态切换仍可能发生时间域跳变。部署/换版本前须另验。

## 4. 操作指令（逐段标注执行状态）

> 标注含义：**[已执行]** = 本机实际运行并留存输出（具体日期见各节）；
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

构建或撤销补丁前，先停止正在使用本 underlay 的 driver，避免覆盖进程已映射的共享库；本轮实采期间产物保持冻结。

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
  源码须精确等于「冻结基线 + tracked patch」；未知 tracked 或非忽略 untracked 文件在 apply/revert 前拒绝；
  最后对 node 与组件库都跑 `ldd`，`ldd` 自身失败或出现 `not found` 均直接失败。
- 2026-10-16 初版构建无补丁；2026-10-17 开始采用 §3.2 两份补丁。原 SHA 不变，补丁新增文件和变更受逐字节校验。

### 4.3 构建自研包 — [已执行]

```bash
# 从仓库根目录开始
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
cd extensions/ros2_ws
colcon build --symlink-install
cd ../..            # 回到仓库根目录，后续命令的相对路径才成立
```

### 4.4 生成现场配置 — [已执行]（工具，占位地址及真实现场地址）

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

真实启动 — **[已执行，PointCloud2/RViz 部分通过；退出清理失败]**：

```bash
ros2 launch hei_robot_bringup mid360s_bringup.launch.py \
    config_path:=$PWD/extensions/config/local/mid360s_config.json \
    pointcloud_format:=pointcloud2 publish_freq:=10.0 launch_rviz:=true
```

- `config_path` **必填**；缺失时 launch 在启动任何节点前失败，并提示如何生成配置。
- 同时**只有一个** driver 实例；原始点云帧 `lidar_link`；**不发布**任何 base_link/IMU 外参 TF。
- 实测 `/livox/lidar` 约 9.6 Hz（约 15 秒统计），用户截图 RViz 约 31 FPS、点云 Status OK。
  复现 `Ctrl+C` 后 core PID 306909；GDB 栈为 `free()` → system `libspdlog.so.1` 的
  `spdlog::logger` control-block dispose → Livox SDK 全局 `shared_ptr<spdlog::logger>` 析构。
  SDK 自带 spdlog 1.3.1，而系统/ROS 提供 1.9.2；2026-10-17 已用无硬件 A/B 确认冲突并修复共享库符号隔离。
  修复后 3 次实际 driver SIGINT 均 exit0；旧 core 栈保留为历史证据。
- RViz 的 Livox IMU display 插件在 Humble 下无法加载；这不影响 PointCloud2 显示，IMU
  topic 本身仍须独立检查。

### 4.6 短包检查（1–2 分钟）— [已执行：一包通过，一包失败]

PointCloud2 已单独测得约 9.6 Hz。真实 CustomMsg/IMU 短包已独立检查：58.9 秒稳定包通过；
95.6 秒启动期包因 LiDAR/IMU 双流接收与源时间间隔超过 0.5 秒而失败。正式 31 分钟包另见 §4.7。
RViz 的 FPS 是显示刷新率，不代表雷达发布频率。短包只作诊断，不替代正式 1800 秒验收。

```bash
# 从仓库根目录开始，先 source 三层环境（同 §4.5）
ros2 topic type /livox/lidar          # 当前 PointCloud2 模式应为 sensor_msgs/msg/PointCloud2
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu               # 观察至少 30 秒；记录实际均值和波动
ros2 topic echo --once /livox/imu      # 核对 header、frame_id 与字段
```

先在运行 PointCloud2 的终端按 Ctrl+C；确认没有残留 driver 后，在新终端**只启动一个**
CustomMsg driver（RViz 关闭，避免把点云显示插件问题混入数据测试）：

```bash
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 launch hei_robot_bringup mid360s_bringup.launch.py \
    config_path:=$PWD/extensions/config/local/mid360s_config.json \
    pointcloud_format:=custom publish_freq:=10.0 launch_rviz:=false
```

另开终端确认类型和频率，再录 1–2 分钟；先停止 rosbag 进程并等待 bag 关闭，再停止 driver：

```bash
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
ros2 topic type /livox/lidar          # 应为 livox_ros_driver2/msg/CustomMsg
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
mkdir -p ~/hei-data
df -h ~/hei-data
SHORT_BAG="$HOME/hei-data/mid360s_short_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$SHORT_BAG" \
    /livox/lidar /livox/imu
```

停止录包后检查实际生成的 bag 目录，并全量检查消息类型、解码、结构、源时间与双流覆盖：

```bash
ros2 run hei_robot_bringup mid360s_bag_qc \
    --bag "$SHORT_BAG" \
    --pointcloud-format custom --expected-duration 60 --max-gap 0.5 \
    --json-out "${SHORT_BAG}_qc.json"
```

未给 `--imu-hz-min/--imu-hz-max` 时，工具应返回 **INCONCLUSIVE**，这是预期结果；先核实
设备当前 IMU 频率档位，再用认可的上下限复验。短包若其余结构与时间检查通过，也仍不能
替代正式包。当前 MID-360S ROS 配置 JSON 只配置网络，不包含 IMU 输出档位。

### 4.7 正式原始包（30 分钟验收）— [已执行；离线 QC FAIL]

**正式包必须录 CustomMsg**（含逐点 `offset_time`），因此流程是：
先停掉正在显示 PointCloud2 的那个 driver，再用 `pointcloud_format:=custom` 单独启动
**同一个** driver（任何时刻只允许一个 driver 占用雷达），然后录包。

本轮已观察到初始化最初几秒的缓存与时间间隔瞬态。保留启动日志，并在两路数据稳定预热至少10s后开始
完整正式窗口；不通过删改旧bag或重写时间戳处理异常。正式窗口中任何超限仍按原门限FAIL。

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

**本次实测（2026-10-16，Codex 独立执行）**：录制目录
`/home/robo/hei-data/mid360s_live_20261016_1827`，bag 时长 **1866.911 s**（31:06.9），约
7.1 GiB；包含 `/livox/lidar` CustomMsg **18,669** 条和 `/livox/imu` **373,386** 条。检查器完整
解码，无消息解码失败、无 CustomMsg 结构错误；累计检查 **373,383,456** 个点。

- **通过项**：有效时长 ≥1800 s；IMU 平均 **200.001 Hz**，接收最大间隔 0.0218 s，源时间最大间隔
  0.0178 s；LiDAR bag 接收最大间隔 0.227 s；双流覆盖约 1866.8 s。
- **失败项**：LiDAR `header.stamp` 倒退 **2 次**，最大源时间间隔 **0.713432 s**（门限 0.5 s）。
  异常附近 6 秒片段中见一次 140,736 点的大帧，`offset_time` 最大 702.866 ms，前后还伴随
  700.107 ms 源时间跳跃和 −600.660 ms 回退。它与异常相关，但根因尚未确认，不能仅凭此归因于
  网络或驱动队列。
- CustomMsg 帧内 `offset_time` 顺序逆转按上游实现仅作 INFO 统计（全包 2,899,209 处）；点按
  UDP 到达次序追加，不保证按 offset 排序。该项不作为 FAIL 判据；全包最大 offset 702.866 ms。
- IMU 原始单位信息：加速度为 g、角速度为 rad/s、orientation 无有效值。此阶段保留原始数据，
  下游 LIO 集成前必须显式做单位/坐标适配。
- QC JSON：`/home/robo/hei-data/mid360s_live_20261016_1827_qc.json`。总体退出码 1，02-LIVE **未通过**。

### 4.8 回放 — [2026-10-17 新正式包完整回放 PASS]

```bash
# 从仓库根目录开始（三层 source 同 4.5）
ros2 bag info data/mid360s/raw_<时间戳>
ros2 bag play data/mid360s/raw_<时间戳> --clock
```

隔离 ROS Domain 77 下，58.9 秒真实短包曾以 2 倍速播放；订阅端确认 CustomMsg 类型且收到 IMU，
播放器退出码 0。2026-10-16 旧不合格正式包留存，未完整回放；2026-10-17 新正式包
`/home/robo/hei-data/mid360s_live_20261017_retest01` 已在Domain87/localhost隔离以2倍速完整回放：
player exit0，订阅端收到18,648 CustomMsg和372,964 IMU，两路完整消息序列的长度前缀CDR SHA256
均与原包逐话题序列完全一致。证据 `/home/robo/hei-data/mid360s_retest_20261017_evidence/formal_replay01/result.json`。

### 4.9 离线验收 — [合成数据已执行；真实正式包 FAIL]

以下命令此前在本机对**合成 bag** 跑通过（FAIL/INCONCLUSIVE/PASS 三条路径都验证过）；本次又
对真实短包及正式包运行。真实正式包结果以 §4.7 为准：

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

### 4.10 2026-10-17 修复后的正式复测 — [DONE / PASS]

- 独立验证：143 项包测试、underlay 回归全部通过；SDK 原库/修复库各3次×两加载顺序 A/B，修复库均正常退出；
  实际 CustomMsg→PointCloud2→CustomMsg 各15s，3次 SIGINT 均 exit0。初始缓存/源间隔另存，不充当稳定数据验收。
- 新 bag `/home/robo/hei-data/mid360s_live_20261017_retest01`；10s 预热后实际包时长1864.757s，单 driver，
  ROS_DOMAIN_ID=86 / ROS_LOCALHOST_ONLY=1。实时 raw monitor 同时记录源 header 和 monotonic 到达间隔、主机时钟跳变。
- 原始证据、构建产物哈希、运行脚本、预热/启停结果在 `/home/robo/hei-data/mid360s_retest_20261017_evidence/`。
  正式包18,648条CustomMsg、372,964条IMU，7.1GiB；录包器与driver均正常exit0。实时监测无header倒退/重复，
  最大source gap分别0.111439s/0.016172s，未检测到>5ms主机时钟步进。正式完整回放已PASS，
  全量逐点 QC 同样 PASS，退出码0；02-LIVE 本轮数据与生命周期验收 DONE。
- 全量解码18,648帧/372,964条IMU，检查372,972,288点；30项检查中27 PASS、3 INFO、0 FAIL。
  点云10.000Hz、IMU200.006Hz；两路header倒退/重复均0；接收最大gap分别0.111918s/0.018016s，
  源最大gap分别0.111439s/0.016172s，双流源窗口重叠1864.653s。
  CustomMsg逐点offset未排序属于已解释INFO，结构/有效范围检查通过，没有放宽门限。
- 独立验收索引与JSON见 [20261017-acceptance](agent/evidence/mid360s/20261017-acceptance/README.md)。
- 另经日常 `ros2 launch` 启动PointCloud2并Ctrl+C，launch父进程与driver子进程均正常结束；日志中的PointCloud2格式已核对。

## 5. 网络：当前拓扑与数据验收前检查

### 5.1 网卡与路由（2026-10-16 实测快照；变化后须重新核对）

| 接口 | 快照值 | 说明 |
| --- | --- | --- |
| `enx9c69d3cd61a9`（USB ASIX） | `192.168.1.5/32` | 雷达侧；实测直连路由 `192.168.1.135/32` |
| MID-360S | `192.168.1.135` | 实测邻居项可达；当前 PointCloud2 已成功收流 |
| `wlp0s20f3`（WiFi） | `192.168.1.144/23` | 覆盖 `192.168.0.0/23`，默认路由经 WiFi |
| `enp7s0`（机器人高速链路） | `192.168.50.1/24` | 与雷达 USB 网口分开；机器人端 IP 按用户提供为 `192.168.50.2` |

WiFi `/23` 与雷达地址段有重叠，但本次实测存在更具体的 `192.168.1.135/32 dev
enx9c69d3cd61a9` 路由，雷达数据已实际收到。USB 网卡更换或地址变化后，先用
`ip route get <雷达IP>` 确认选路；`enp7s0` 是机器人高速链路，不能与雷达网口混为一谈。
`hei_robot_bringup` 仍不内置示例 IP，现场值由被忽略的本机配置提供。

2026-10-17 复测快照：USB/雷达地址与直连路由不变；WiFi 实际为 `172.22.22.189/24`，机器人高速口 DOWN。
不应将上表昨日 WiFi 地址用于今天的配置；本轮未修改任何网络设置。

### 5.2 本轮的约束边界

本次已按用户授权配置并实测 USB 网卡与雷达直连路由；当前采集/下一步数据检查期间保持网络配置
不变。之后若网络或网卡有调整，先读取 `ip -br address`、`ip route`、`ip route get <雷达IP>`，
再按该任务授权与现场证据选择可回滚方案，并记录回滚命令。

### 5.3 当前进度与后续步骤（按顺序）

1. 已完成：网卡/路由/邻居核对、现场配置、PointCloud2/RViz、CustomMsg + IMU 短包和 31 分钟正式录制。
2. 2026-10-17 已诊断旧 bag：主机系统时钟约598ms回拨，NoSync打戳和帧调度均受影响；SQLite排序还掩盖了IMU回拨。
   谁调整了时钟缺乏旧日志证据。SDK/system spdlog冲突已通过独立A/B确认。
3. 两份 tracked patch、构建/回归和独立 Review 已通过；3次短测、正式采集及日常launch共5次实际 driver 正常退出。长时复测见 §4.10。
4. 已按原门限完成新包全量逐点 QC 和完整回放，两者 PASS；旧包 FAIL 与启动瞬态原样保留。
5. 02-LIVE 数值数据/生命周期验收通过，可安排 03-LIO；融合前补齐单位、外参、轴向检查。注意 `/livox/imu` 的 frame 是 `livox_frame`
   （上游硬编码，见 §7.1）。

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
| CustomMsg 结构 | 合法非空 | `point_num == len(points)`、点非空、逐点 `offset_time` 存在且可表示为 uint32；帧内逆序/分布/冻结仅作 INFO |
| IMU 频率 | **必须显式给出标称范围** | 未给出 → 整体 INCONCLUSIVE，不得判通过 |

当前两路原始 UDP 已确认 NoSync；本轮按实测约200Hz工作档采用 `--imu-hz-min 190 --imu-hz-max 210`。
官方 SDK 当前接口说明 MID-360S 支持200/500/100/50Hz档（[官方 API](https://github.com/Livox-SDK/Livox-SDK2/blob/master/include/livox_lidar_api.h)），
不能据型号假定所有固件/配置固定为200Hz；本轮未独立读出设备 IMU 档位配置键。改变档位需重新给验收范围。

⚠️ 本工具只做**数据时间基准与消息结构的数值一致性检查**，**不能**证明硬件时间同步精度，
也**不能**替代点云/IMU 物理正确性检查；它**不做** SI 换算（见 §7.2）。

## 7. 已知限制（冻结基线 + §3.2 补丁）

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
| 构建拒绝未知 dirty / patch 状态不一致 | 检查第三方源差异与 tracked patch；先保护用户改动，不用强制清理绕过来源校验 |
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

## 9. 本任务尚未完成的验证项

- 旧正式包的 FAIL 保留；新正式长时 QC/完整回放已 PASS。以下为后续阶段尚未验证范围：
- 固件版本、供电/线束记录、外参标定、点云物理正确性和精确硬件时间同步（gPTP/PPS）。
- 任何 LIO/SLAM/定位/建图/导航（03-LIO 起）。
- 硬件时间同步精度（gPTP/PPS 锁定状态）、外参标定、坐标轴与安装方向核对。
- AGX 侧部署与 aarch64 构建。
- `rosdep install` / apt 安装：本机 `sudo -n true` 失败（DSH 沙箱 `no new privileges`
  标志；这与「本机 sudo 需要密码」是不同限制），故未执行任何需要提权的安装。
  `vcstool` 未安装，本任务用 `git clone` + SHA 校验代替（§3.1），不影响可复现性。

## 10. 维护

- 依赖升级：改 `extensions/dependencies/livox.repos` 的 SHA，重跑构建脚本与全部测试。
- 阶段推进时同步 §1 状态表与 [TASK_BOARD](agent/TASK_BOARD.md)（后者由 Codex 单写）。
- 现场 IP、串口、序列号等本机值放 `extensions/config/local/`（已忽略），不进本文。
