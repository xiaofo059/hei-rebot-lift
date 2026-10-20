# hei_robot_slam

MID-360S 三维 SLAM 电脑端接入层：FAST-LIO2 前端 + RTAB-Map 0.23.13 后端。

> 状态（2026-10-19，任务 HEI-LIDAR-04-SLAM-LAPTOP-BASE）：软件部署与集成冒烟完成，
> 04-SLAM 仍为 PARTIAL。已用桌面运动包全量回放验证建图/后端优化/导出/只读定位链路；
> 没有 30-50 米闭环与 3 位置重定位验收，没有现场外参标定，没有实机/AGX 验证。
> 最新一次新参数完整 2x 回放：55 持久节点 / 55 扫描，25 条独立 type2 自动几何闭环，

## 本包做什么 / 不做什么

| 做 | 不做 |
| --- | --- |
| 给 RTAB-Map 提供最薄 XYZI 字段适配（丢弃 FAST-LIO 未估计的零法线字段） | 不改 FAST-LIO/Livox 源码（旧 LIO 包不动） |
| 离线建图 launch（唯一新 DB、唯一输出目录、禁 -d/delete_db_on_start） | 不启动机器人底盘/机械臂/升降/导航 |
| 只读定位 launch（Mem/LocalizationReadOnly=true，DB SHA256 运行前后必须一致） | 不伪造 base_link/odom，不重复发布 TF |
| 实时雷达只读定位 launch（已有 DB、FAST-LIO 与 live driver） | 不写回原图；现场 live 定位尚待实测 |
| 参数/帧/模式契约校验（启动前拒绝违规配置） | 不声称地图精度或现场验收通过 |
| 数据库节点/link/回环只读检查与严格导出（PLY body + 9 列有限 pose 校验） | 不把 neighbor/prior/用户手工 link 计为自动回环 |
| 有界进程回收 harness（SIGINT 只给 launch 父进程，12 s 有界升级，残留检测） | 不自动提交、不碰 TASK_BOARD/EXECUTION_LOG |

## 依赖：RTAB-Map 0.23.13（无 root 本地前缀）

**环境隔离**：请使用清除 Conda 变量的新 shell（Conda 会污染 PYTHONPATH/LD_LIBRARY_PATH/
下载 36 个固定版本官方 deb（含 SHA256 校验）→ 解包到 gitignored 的
extensions/third_party/rtabmap_local/ → 生成 env.sh → 核验导入/动态链接。

```bash
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/third_party/fast_lio_ws/install/local_setup.bash
source extensions/third_party/rtabmap_local/env.sh
source extensions/ros2_ws/install/local_setup.bash
```

注意：env.sh 的 PYTHONPATH 必须同时包含
prefix/opt/ros/humble/local/lib/python3.10/dist-packages（rtabmap_msgs 消息）与
prefix/opt/ros/humble/lib/python3.10/site-packages；缺前者会导致
from rtabmap_msgs.msg import MapGraph 失败（本轮已实测修复）。


## 两种离线模式

### 1) 离线建图（只回放已有 bag，默认）

```bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=1
ros2 launch hei_robot_slam slam_offline_mapping.launch.py \
    bag_path:=/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456 \
    uncalibrated:=true probe:=true
```

* 必须显式 uncalibrated:=true（外参只有厂商名义值）；
* 必须 Domain 89 + localhost only（前端/后端同域）；
* 每次在 /home/robo/hei-data/mid360s_slam_20261019/<session>/ 新建唯一目录，
  数据库为 rtabmap_mapping.db，已存在即拒绝；
* 全链路 use_sim_time=true 且 ros2 bag play --clock 100（历史包源时间与后端 TF 同时间域）；
* probe:=true 会写 run_dir/probe_report.json（stamp/frame/法线/TF/后端云取证）。

### 2) 只读定位（原建图 DB）

```bash
ros2 launch hei_robot_slam slam_localization.launch.py \
    bag_path:=/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456 \
    database_path:=/home/robo/hei-data/mid360s_slam_20261019/<mapping_run>/rtabmap_mapping.db \
    uncalibrated:=true probe:=true
```

手动初始位姿入口（本机 0.23.13 实测存在）：

```bash
ros2 topic pub --once --qos-reliability reliable /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

定位模式固定 Mem/IncrementalMemory=false、Mem/InitWMWithAllNodes=true、
Mem/LocalizationReadOnly=true；退出不回写数据库（实测 DB SHA256 不变）。

### 3) live 硬件建图（本轮未实测，需显式授权）

```bash
ros2 launch hei_robot_slam slam_live_mapping.launch.py \
    config_path:=extensions/config/local/mid360s_config.json \
    allow_live_hardware:=true uncalibrated:=true
```

只启动 MID-360S driver + FAST-LIO + XYZI 适配 + RTAB-Map；不启动机器人本体。
本轮没有硬件授权，live launch 未在真实传感器上执行，文档不声称 live 已验证。

### 4) 实时建图后用实时雷达只读定位

地图数据库保存在本包的 `map/` 目录；`map/.gitignore` 排除生成的数据，避免将大型数据库和点云提交到 Git。每次建图使用唯一 session：

```bash
map_root=/home/robo/robotopia/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_slam/map
map_session="mid360s_live_$(date +%Y%m%d_%H%M%S)"

ros2 launch hei_robot_slam slam_live_mapping.launch.py \
    config_path:=/home/robo/robotopia/hei-rebot-lift/extensions/config/local/mid360s_config.json \
    output_root:="$map_root" session:="$map_session" \
    allow_live_hardware:=true uncalibrated:=true probe:=true rviz:=true
```

正常完成后按一次 Ctrl+C 并等节点退出。数据库位于
`$map_root/$map_session/rtabmap_mapping.db`。实时定位 launch 使用这个现有库，只读加载地图：

```bash
database_path="$map_root/$map_session/rtabmap_mapping.db"
test -s "$database_path" || { echo "地图数据库不存在或为空：$database_path"; exit 1; }
sha256sum "$database_path"     # 记录启动前 SHA256

ros2 launch hei_robot_slam slam_live_localization.launch.py \
    config_path:=/home/robo/robotopia/hei-rebot-lift/extensions/config/local/mid360s_config.json \
    database_path:="$database_path" \
    allow_live_hardware:=true uncalibrated:=true probe:=true rviz:=true
```

定位启动后，RViz 的 `/cloud_map` 显示已保存地图，`/slam/lidar_cloud_xyzi` 显示当前扫描。将雷达放回建图起点附近，并在另一个已 source 相同环境的终端给 `/initialpose` 发布粗略初值：

```bash
timeout -s INT 15s ros2 topic pub --once --qos-reliability reliable /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

上面的零位初值只适用于雷达回到建图起点和相近朝向的冒烟测试；在地图其他位置时，应提供该位置在 map 坐标系下的近似位姿。停止定位后再次运行 `sha256sum "$database_path"`，应与启动前一致。此 launch 已实现但尚未在真实雷达上实测；单一起点测试也不代替多位置现场精度验收。

## frame / topic / 时间契约

| 项目 | 契约 |
| --- | --- |
| 前端 TF | fastlio_mapping 唯一发布 camera_init -> body |
| 后端 TF | rtabmap_slam/rtabmap 唯一发布 map -> camera_init |
| 禁止 | 不发布 base_link / odom / base_footprint；不重复发 camera_init -> body |
| 里程计 | 单一来源 FAST-LIO /Odometry；RTAB 用 odom_frame_id=camera_init 的 TF 取 odom，不启动 ICP odometry 节点 |
| 点云 | /cloud_registered_body（去畸变单帧，body，stamp 与 odom 相同）-> XYZI 适配 -> /slam/lidar_cloud_xyzi -> RTAB scan_cloud |
| 时间 | 离线全链路 use_sim_time=true + bag --clock 100；live 全链路 false |
| 6DoF | Reg/Force3DoF=false、Icp/Force4DoF=false，真 6DoF |
| 回环 | 3D 几何 ICP；Icp/Strategy=1（libpointmatcher）+ Icp/PointToPlane=true；不为数据强行降阈值 |

### 纯激光 proximity 修正（本轮实测发现并修复）

RTAB-Map 0.23.13 的 one-to-one proximity 分支默认 RGBD/ProximityOdomGuess=false，
会传空 guess 进入 computeTransform；无图像时该路径报
`Missing visual features or missing raw data ... Transform cannot be estimated`，
导致视觉一侧的候选被丢弃。源码依据：rtabmap 0.23.13-humble，
Rtabmap.cpp one-to-one proximity 分支（guess 仅在 ProximityOdomGuess=true 时非空），
Memory.cpp computeTransform 的视觉数据检查。本包两个模式都显式设
`RGBD/ProximityOdomGuess: "true"`（用 odom 相对位姿作为 ICP guess），
不降低 ICP / 图优化 / 回环阈值。实测：修复前一次 66 s 定位回放有 15 条上述 WARN，
修复后 36 条 /info 全部有 ref_id，9 条 proximity_detection_id（1,29,31,33,36,38,40,44,47），
WARN 为 0；初始位姿发布前 proximity 接受为 0，发布后从 map 节点 1 开始连续接受。

## 参数文件

| 文件 | 用途 |
| --- | --- |
| config/rtabmap_3d_lidar_mapping.yaml | 建图：Mem/IncrementalMemory=true，唯一新 DB |
| config/rtabmap_3d_lidar_localization.yaml | 定位：只读 DB（仅 3 个模式开关与建图不同） |
| config/slam_mapping.rviz | RViz2 建图视图：显示 /cloud_map 和建图过程轨迹 /mapPath |
| config/slam_localization.rviz | RViz2 定位视图：显示 /cloud_map 与实时扫描，历史建图轨迹 /mapPath 默认隐藏 |

所有参数在启动前由 hei_robot_slam.slam_contract 校验：frame 不符、3DoF/4DoF、缺 3D 点云订阅、
auto-delete 数据库、定位非只读、缺 ProximityOdomGuess 等都会拒绝启动。

## 定位连续取证（探针 JSONL / summary）

`slam_cloud_probe` 在 `report_path` 存在时自动启用（任务 HEI-SLAM-OBS-20261020），不改变旧契约计数，
在 report 旁派生**独占新文件**（已存在则自动加 `__02` 等后缀，不覆盖既有时间线）：

* `<report_stem>.localization_events.jsonl`：逐事件时间线，每条 flush；
* `<report_stem>.localization_summary.json`：累计 + 最近/最后事件，周期写、关停写 final。

事件类型：`info`、`initialpose`、`rosout`、`backend_log`、`tf_map_to_camera_init`、
`localization_pose`。每条事件带 `source_stamp_ns`、`recv_wall_ns`、`recv_mono_ns`；
缺字段/时间戳未知时置 `null` 并标 `source_stamp_unknown`；stats 非有限值安全置 `null`，
不伪造 ICP fitness。

**语义（不要混用）**

| 概念 | 判定 | 说明 |
| --- | --- | --- |
| accepted | 仅 `/info` 非零 `proximity_detection_id` / `loop_closure_id` | 后端事件证据，不是精度验收 |
| pending | `Localization was good, but waiting ...` | 候选延迟确认，不是 accepted |
| candidate_link_added | 后端日志 `Add local loop closure in SPACE` | link 加入候选，仍可能被图优化拒绝，不计 accepted |
| no_current_event | 当前帧 accepted id 为 0 | 正常现象，不清空历史累计，不等于失败 |
| TF / localization_pose | 只记录 | 不代表 accepted |

**pending 日志可观测性**：rtabmap 核心 UWARN（如 `Localization was good`、`Update map correction`）
通常直写 stderr（ANSI + Rtabmap.cpp 时间戳），**不发布到 `/rosout`**。需要完整 pending 证据时，
以只读方式启动 observer 并显式传后端日志路径（从启动时 EOF 增量 tail，不修改源文件）：

```bash
ros2 run hei_robot_slam slam_cloud_probe --ros-args \
  -p report_path:=/home/robo/hei-data/.../probe_report.json \
  -p backend_log_path:=/tmp/hei_slam_localization_newmap.log
```

未配置时 summary 的 `pending_observability.pending_coverage=rosout_only_partial` 且 `complete=false`，
明确说明 pending 日志可观测性不足（不允许把 rosout 当完整捕获）。

**读 summary**：`cumulative` 是历史累计（清零帧不改变）；`current`/`last` 是当前/最后事件；
`derived.state_hint` 给出当前证据状态；`derived.accepted_since_latest_initialpose` 用
last accepted seq 与 latest initialpose seq 比较——新 `/initialpose` 后历史累计仍保留，但该字段为
false 时表示这次初值之后尚无接受事件；`derived.pending_current_initialpose_attempt` 对应为 true。
`event_status` 明确区分 `active` / `disabled_no_report_path` / `disabled_by_parameter` / `failed`（含 error）。
`cumulative.optimization_frames` 只统计 iterations>0（0 或字段缺失不计）。

**边界**：本功能只做观测，不修改算法/参数/TF/点云/驱动；所有 ROS 测试只用 Domain 91 合成数据
（Domain 89 属于现场会话，禁止测试占用）。accepted 计数不是地图/定位精度验收，现场结论仍需独立测量。


## 工具

```bash
# 只读检查数据库（节点/Data.scan/link 类型/自动回环）
ros2 run hei_robot_slam slam_db_tools inspect <db> --json out.json

# 严格导出（PLY body 顶点/有限 XYZ/pose 9 列有限；DB 前后 SHA256 必须一致）
ros2 run hei_robot_slam slam_db_tools export <db> --output-dir <全新目录> --json out.json

# 有界启动/停止（只给 launch 父进程一次 SIGINT；12 s 后 TERM/KILL 有界升级）
ros2 run hei_robot_slam slam_run_bounded --log run.log --result run.json --duration-s 260 -- \
    ros2 launch hei_robot_slam slam_offline_mapping.launch.py ...
```

导出接口按本机 0.23.13 `--help` + 实测固定：
`--cloud --scan --poses --poses_format 11 --opt 2 --voxel 0.05 --max_range 0`。
**实测注意**：0.23.13 若不给 `--voxel`，assembled cloud 为 0 点并报
`Export failed! The cloud is empty.`，因此本工具显式传值并在结果 JSON 记录实际命令。

## 测试

```bash
cd extensions/ros2_ws/src/hei_robot_slam
python3 -m pytest test/ -q        # 纯单测 + 2 个真实节点启动回归
```

覆盖：契约/DB 保护/参数门控、launch 静态护栏（无 -d、唯一 LIO、无 TF 发布器、live 仅雷达链路）、
XYZI 布局拒绝（大端/行填充/畸形/非 float32 xyz）、Link 枚举与 Data.scan 统计、
导出实体校验（截断 PLY/非有限 XYZ/坏 pose 行）、有界回收（忽略 SIGINT/孤儿/子节点死亡检测）、
真实节点启动（cloud_probe/cloud_adapter 不崩溃可干净退出）。

## 目录

| 路径 | 说明 |
| --- | --- |
| hei_robot_slam/slam_contract.py | frame/模式/DB/参数契约（唯一事实来源，纯函数） |
| hei_robot_slam/launch_support.py | launch 共用解析/唯一目录/launch_info 取证 |
| hei_robot_slam/cloud_xyzi.py | PointCloud2 -> XYZI 纯函数投影与布局拒绝 |
| hei_robot_slam/cloud_adapter.py | slam_cloud_adapter 节点（薄适配） |
| hei_robot_slam/cloud_probe.py | slam_cloud_probe 只读取证探针（stamp/frame/法线/TF/后端云） |
| hei_robot_slam/db_tools.py | slam_db_tools 检查/导出/严格校验 |
| hei_robot_slam/ply_cloud.py | 静态 PLY 地图云发布为 ROS PointCloud2（不加载轨迹） |
| hei_robot_slam/run_bounded.py | slam_run_bounded 有界回收 harness |
| launch/ | 离线建图 / 只读定位 / live 建图三个 launch |
| config/ | RTAB-Map 参数与 RViz 配置 |
| test/ | pytest 回归 |

## 已知限制（不得当作已验收）

* 外参只有厂商名义值，现场未标定；uncalibrated=true 是强制事实声明；
* 桌面重复扫描（233 s，最大位移约 0.66 m）不能当作 30-50 m 闭环/回环精度验收；
* 定位模式只有单桌面轨迹 smoke（用旧参数 DB 验证修复后的 ICP proximity 接受）；
* 无现场 3 位置重定位、无 groundtruth、无实机/AGX 验证；04-SLAM 总体 PARTIAL；
* RViz 只做配置与 topic 非空验证，未做人工 GUI 目视验收；live launch 本轮未在硬件上执行。
