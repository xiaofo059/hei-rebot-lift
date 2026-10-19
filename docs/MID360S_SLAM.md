# MID-360S 三维 SLAM（FAST-LIO2 + RTAB-Map 0.23.13）电脑端接入与离线验证

> 任务：HEI-LIDAR-04-SLAM-LAPTOP-BASE-20261019。分支 `feat/mid360s-lio`，基线 `1aeb8a1`。
> 本轮工作在本机 `fo`（x86_64 Ubuntu 22.04 / ROS 2 Humble / 系统 Python 3.10.12）完成，
> 没有 commit/push，没有切换分支，没有修改机器人本体。
>
> **状态：04-SLAM 软件部署与离线集成验证 PARTIAL。** 已有桌面运动包上的建图/后端优化/
> 严格导出/只读定位软件链路证据；**没有** 30–50 m 闭环精度验收、**没有**现场外参标定、
> **没有** 3 位置实机重定位、**没有** AGX/实机验证。本文件不声称地图质量或现场验收通过。

---

## 0. 结论摘要

| 项目 | 本轮结论 | 关键证据 |
| --- | --- | --- |
| RTAB-Map 版本/依赖 | 本机安装 Humble 官方 `0.23.13`（36 个固定版本 deb，无 root 本地前缀） | 依赖 manifest + `installer --verify`；Codex 独立 verify PASS 与全新外部前缀安装 PASS |
| 前端 FAST-LIO 接口 | 未修改；仍是 `/Odometry`、`/cloud_registered_body`、唯一 TF `camera_init -> body` | 复用既有 LIO 包与冻结构建 |
| 点云字段适配 | 新增最薄 XYZI 适配：丢弃 FAST-LIO 未估计的零法线字段，保留 stamp/frame/xyz/intensity | 适配前后 2331 帧逐帧 XYZI 哈希一致；原始云 normal 全零实测 |
| TF 分工 | 前端独占 `camera_init -> body`；后端独占 `map -> camera_init`；无 base_link/odom | `/tf` 记录仅两对、各唯一 publisher |
| 时间域 | 离线全链路 `use_sim_time=true` + `bag --clock 100`；历史 TF 查询成功 | now - odom last ≈ 0.01–0.07 s；历史查询 5/5（Codex）与 34/34（本机） |
| 建图 | 新参数完整 2x 回放：55 持久节点 / 55 扫描；旧配置基线 52/52 保留对照 | Codex 旧参数回放 + 本轮新参数完整回放（§6.2/§6.3） |
| 回环 | 真实自动几何闭环类型为 `Link.type=2`（local space closure）；旧配置 28 条有向 = 14 独立，新参数 50 条有向 = **25 独立** | DB SQLite 直查 + 类型枚举按本机头文件核对 |
| 纯激光 proximity 修正 | 新增 `RGBD/ProximityOdomGuess=true`（两模式），消除无图像空 guess 视觉路径；建图 WARN 33→0 | 源码 0.23.13-humble 调用路径 + 修复前后 WARN/`/info`/新库对比 |
| 定位模式 | 原建图 DB 只读加载；`/initialpose` 实测可用并触发从 map 节点 1 起的 9 次 ICP proximity 接受 | collector 原始 JSON + DB SHA256 运行前后一致 |
| 导出 | 新最终库：PLY 48,855 顶点 + 55 行 9 列有限轨迹；DB SHA 不变（旧库 48,986/52 仅作历史对照） | 严格 PLY body/有限 XYZ/pose 列数校验通过（§6.4） |
| RViz | 配置与所需 topic 已验证非空；GUI 未做人工目视验收 | `/cloud_map` 与 `/mapPath` 实测非空；配置静态测试 |
| 现场验收 | **未执行**（UNKNOWN） | 需 30–50 m 闭环 + 3 位置重定位，见 §9、§10 |

---

## 1. 范围与非目标

本轮只做**电脑端软件接入与离线集成验证**：

1. 用官方 ROS apt 包（固定版本）提供 RTAB-Map 0.23.13；无 root 时用本地前缀方案；
2. 新增 `extensions/ros2_ws/src/hei_robot_slam/`：离线建图、只读定位、live 入口、XYZI 适配、
   只读取证探针、DB 检查/严格导出、有界进程回收 harness、回归测试；
3. 用桌面运动包完整回放验证建图、后端优化与保存；
4. 用回放验证只读定位模式与 `/initialpose` 手动初始位姿入口；
5. 严格导出优化点云与轨迹，并验证产物实际非空；
6. 记录版本/SHA/依赖、TF/时间/点云契约、参数依据与全部限制。

**非目标 / 未做**：现场外参标定；现场 30–50 m 运动与闭环精度；3 位置实机重定位；
AGX 部署；机器人 odom/base_link 接入；导航；任何机器人本体/底盘/机械臂/升降动作；
commit/push；修改现有 LIO 包、TASK_BOARD、EXECUTION_LOG。

---

## 2. 版本、依赖与安装证据

### 2.1 关键版本

| 项目 | 版本 / SHA | 说明 |
| --- | --- | --- |
| 仓库 / 分支 / HEAD | `/home/robo/robotopia/hei-rebot-lift` / `feat/mid360s-lio` / `1aeb8a19952cd9011c428a2fbea063683f0cade5` | 本轮前后不变 |
| RTAB-Map（ROS Humble apt） | `0.23.13-1jammy.*`（`ros-humble-rtabmap-slam` = `0.23.13-1jammy.20261011.111917`） | 本机实际安装，非源码构建 |
| ROS | Humble | `/opt/ros/humble` |
| 系统 Python | 3.10.12（`/usr/bin/python3`） | 非 Conda |
| FAST-LIO / Livox | 沿用 03-LIO 冻结构建（见 `docs/MID360S_LIO.md`） | 本轮未重建、未改源码 |

### 2.2 安装方式与原因

本机 `dpkg` 未安装 `ros-humble-rtabmap*`；`sudo -n` 不可用（需要密码）。
因此提供 `extensions/tools/install_rtabmap_laptop.sh`：

* `--local-prefix`（默认）：`apt-get download` 官方 deb **固定版本**，逐包校验 SHA256，
  解包到 gitignored 的 `extensions/third_party/rtabmap_local/`，生成 `env.sh` 并核验；
* `--apt`：若有可交互 sudo，先 dry-run 记录再按固定版本安装；
* `--verify`：只核验已安装前缀（`ros2 pkg prefix`、Python 消息导入、`rtabmap-export`、ldd 无缺库）；
* 脚本显式拒绝非 amd64 架构（避免 AGX/aarch64 误用 x86_64 deb）；
* 解包阶段不覆盖已有前缀（保护运行中的依赖），只解包 manifest 内已校验 deb。

依赖清单（36 包，含版本/SHA256）内嵌在安装脚本中；本机实际安装产物核对：

* 解包体积约 137 MB；`ldd` 对 `rtabmap` 节点与 `librtabmap_core.so` 均无 `not found`；
* `from rtabmap_msgs.msg import MapGraph, Info` 与 `GetMap` 服务导入成功，
  并用 `rclpy.serialization` 对 `MapGraph` 做序列化/反序列化验证动态 type support；
* **已修复部署缺陷**：`env.sh` 的 `PYTHONPATH` 必须同时包含
  `prefix/opt/ros/humble/local/lib/python3.10/dist-packages`（`rtabmap_msgs` 实际位置）与
  `prefix/opt/ros/humble/lib/python3.10/site-packages`；缺前者会导致导入失败。

> 证据：`docs/agent/evidence/mid360s/20261019-slam/01-freeze/`、`06-installer/`。
> Codex 独立运行 `installer --verify` PASS，并用复制缓存在全新外部前缀默认无参安装 PASS，
> 未改动当前运行前缀。

---

## 3. 架构、数据流与契约

```text
MID-360S CustomMsg/IMU ──> FAST-LIO2 ──> /Odometry (camera_init -> body, TF 唯一)
                                │
                                └─> /cloud_registered_body (去畸变单帧, frame=body, stamp=lidar end time)
                                        │  hei_robot_slam XYZI 适配（只做字段投影）
                                        v
                                   /slam/lidar_cloud_xyzi (x,y,z,intensity; stamp/frame 不变)
                                        │
                                        v
                     RTAB-Map 0.23.13 (odom 经 odom_frame_id=camera_init 的 TF)
                        ├─> map -> camera_init (唯一后端 TF)
                        ├─> /mapData /mapGraph /info /localization_pose /cloud_map /mapPath
                        └─> rtabmap_mapping.db（唯一新目录；定位模式只读）
```

### 3.1 frame / topic 契约

| 项目 | 契约 | 违反时的行为 |
| --- | --- | --- |
| 前端 TF | `fastlio_mapping` 唯一发布 `camera_init -> body` | 本包不发布任何 TF；静态测试禁止 TF 发布器 |
| 后端 TF | `rtabmap` 唯一发布 `map -> camera_init` | 同上 |
| 禁止 frame | `base_link`、`odom`、`base_footprint` 全程不存在 | 探针记录实际 `/tf` 对；本任务实测仅两对 |
| 里程计 | 单一来源 FAST-LIO `/Odometry`；RTAB 用 `odom_frame_id=camera_init` 从 TF 取 odom | 不启动 `rtabmap_odom`/ICP odometry；`subscribe_odom=false` 契约校验 |
| 点云输入 | `/cloud_registered_body` 经 XYZI 适配后给 RTAB `scan_cloud` | 不用累计地图、不重复 deskew；适配层拒绝大端/行填充/畸形/非 float32 xyz |
| 时间 | 离线全部节点 `use_sim_time=true` + bag `--clock 100`；live 全部 false | 时间域不一致时历史 TF 查询会失败，探针会记录 |
| 6DoF | `Reg/Force3DoF=false`、`Icp/Force4DoF=false` | 契约校验拒绝 3DoF/4DoF |

### 3.2 点云字段适配的原因

FAST-LIO `publish_frame_body()` 用 `pcl::PointXYZINormal` 打包 body 云，但源码只赋值
`x/y/z/intensity`；`normal_x/normal_y/normal_z/curvature` 不是估计出来的法线。
RTAB-Map 的 3D 点云入口若把这些字段当成有效信息，会污染 PointToPlane。
本包新增 `slam_cloud_adapter`：显式投影为固定 XYZI 字段，**保留** `header.stamp`、
`header.frame_id`、逐点 xyz 与 intensity（MID-360S 反射率），**丢弃**未估计法线字段；
不 deskew、不降采样、不累计。实测：适配前后 2331 帧点云逐帧 XYZI 哈希 2331/2331 完全一致，
原始云 `nonzero_normal_points=0` 且携带 3 个 normal 字段。

### 3.3 进程/退出契约（本轮踩坑修复）

* 后台 shell（`&`/`setsid`）会把 SIGINT/SIGQUIT 继承为 `SIG_IGN`；此时对 launch 父进程发 SIGINT 是空操作；
* 正确做法：Python `Popen(preexec_fn=...)` 把 SIGINT/SIGQUIT 恢复 `SIG_DFL`，`start_new_session=True`；
* 只对 **launch 父进程**发一次 SIGINT，由 launch 转发给子节点；等待 12 s 有界升级 TERM -> KILL；
* 退出后按进程组核对残留；`clean` 判定必须同时满足父进程 0、无升级、无残留、日志无子节点死亡；
* 已实现为 `slam_run_bounded`，用真实子进程反例（忽略 INT、孤儿、日志 `process has died`）回归。

---

## 4. RTAB-Map 参数与关键修正

### 4.1 两模式共同项（节选；完整见 config）

| 参数 | 值 | 依据 |
| --- | --- | --- |
| `frame_id` / `odom_frame_id` / `map_frame_id` | `body` / `camera_init` / `map` | 本任务 frame 分工；不伪造 base_link/odom |
| `subscribe_scan_cloud` | true（其余 subscribe_* false） | 唯一输入是去畸变单帧 3D 扫描；无图像/无 IMU 直连 |
| `subscribe_odom` | false | 单一 odom 来源用 TF 读取；不订阅第二路 odom topic |
| `odom_sensor_sync` | false | 点云已在 body 帧且与 odom 同 stamp，不需要二次同步 |
| `Reg/Strategy` | 1（ICP） | 无图像，几何注册；`Icp/Strategy=1` = libpointmatcher，是否点面由 `Icp/PointToPlane` 决定 |
| `Icp/Strategy` / `Icp/PointToPlane` | 1 / true | 官方 3D 激光例；点面 ICP |
| `Reg/Force3DoF` / `Icp/Force4DoF` | false / false | 真 6DoF |
| `RGBD/ProximityMaxGraphDepth` / `PathMaxNeighbors` / `ProximityAngle` | 0 / 1 / 0 | 官方 lidar3d 例的 proximity 策略；数值含义按本机 Parameters.h 注释写清（0=忽略深度上限/不设角度上限） |
| `Optimizer/Strategy` / `GravitySigma` | 2 (GTSAM) / 0.3 | 本机默认；弱重力先验，不降 6DoF |
| **`RGBD/ProximityOdomGuess`** | **true（两模式）** | 纯激光关键适配，见 §4.2 |

### 4.2 纯激光 proximity 修正（本轮实测发现）

现象：修复前 66 s 定位回放出现 15 条
`Memory.cpp computeTransform() Missing visual features or missing raw data to compute them.`，
且定位接受依赖被视觉分支吞掉；Codex 完整建图日志同样有 33 条该 WARN。

源码依据（rtabmap 核心仓库 tag `0.23.13-humble`，本轮 blobless+sparse 取回核对）：

* `Rtabmap.cpp` 的 one-to-one proximity 分支（空间近邻、`_proximityBySpace && _rgbdSlamMode`）中，
  `guess` 仅在 `RGBD/ProximityOdomGuess=true` 时用优化位姿差构造；false（默认）时传 null；
* `Memory::computeTransform` 在无图像特征、且无可用 guess 的数据检查下走告警分支
  （视觉路径无法估计），因此纯激光场景下该分支被丢弃；
* 本机安装 `Parameters.h:447` 确认该键默认 false。

修正：两个模式都显式 `RGBD/ProximityOdomGuess: "true"`，用 odom 相对位姿作为 ICP guess；
**不**降低 `Icp/CorrespondenceRatio`、`RGBD/OptimizeMaxError`、`Rtabmap/LoopThr` 等阈值。

实测对比（同一 Codex 旧配置 DB，只读定位 66 s）：

| 指标 | 修复前 | 修复后（collector 原始数据） |
| --- | --- | --- |
| `Missing visual features` WARN | 15 | **0** |
| `/info` 条数 / 带 ref_id | 未采集 | 36 / 36 |
| `proximity_detection_id>0` | 未采集 | **9** |
| proximity 接受的 map 节点 | 日志 nearest id 150 -> 1 | `1, 29, 31, 33, 36, 38, 40, 44, 47` |
| `/initialpose` 前接受 | — | 0 条 |
| `/initialpose` 后接受 | — | 从 map 节点 1 开始连续接受 |
| WM 节点数 | 52 | 52（DB 全量加载） |
| 末次 `Loop/MapToBase` / 协方差 | — | xyz≈(0.500,0.023,-0.014) m，lin_var≈3.0e-4 |
| 建图侧 `Missing visual features` WARN | 33（旧参数完整回放） | **0**（新参数完整 2x 回放） |
| 新参数建图 DB 自动几何闭环 | — | 50 条有向 type2 = **25 条独立空间约束**；type0=108 有向 |

> 说明：`/localization_pose` 默认即使未定位也发布（`pub_loc_pose_only_when_localizing=false`），
> 因此本文件不以该 topic 计数作为定位成功证据；以上结论来自 `/info` 的
> `proximity_detection_id` 序列、stats、以及修复前后 WARN 对比。
> `Space_added_visually` 是上游遗留分支标签，不等于输入了图像；本轮输入仍只有 3D 激光。

---

## 5. 模式、数据库保护与手动初始位姿

### 5.1 离线建图（唯一新 DB）

* 入口：`slam_offline_mapping.launch.py`；强制 `uncalibrated:=true`、`ROS_DOMAIN_ID=89`、
  `ROS_LOCALHOST_ONLY=1`、已存在含 `metadata.yaml` 的 bag；
* 每次运行在 `output_root/<session>/` 新建唯一目录（已存在即拒绝），数据库固定名为
  `rtabmap_mapping.db`，且**必须不存在**（RTAB-Map 无 -d 时会打开并追加旧库）；
* 包内/launch 内**没有任何** `-d`、`--delete_db_on_start`、`delete_db_on_start` 参数；
  静态 AST 测试与启动前契约校验双重拦截；
* 进程编排：FAST-LIO + XYZI 适配 + `rtabmap_slam/rtabmap` + `ros2 bag play --clock 100`，
  可选只读探针与 RViz。

### 5.2 只读定位（原建图 DB，不回写）

* 入口：`slam_localization.launch.py`；强制已存在数据库；参数固定
  `Mem/IncrementalMemory=false`、`Mem/InitWMWithAllNodes=true`、`Mem/LocalizationReadOnly=true`；
* 实测退出日志：`rtabmap: Database is read-only, the current state of the memory is not saved.`；
* 旧库只读定位前后 DB SHA256 **一致**（`afab12b8…`）；新建图库目前验证的是导出前后 SHA 不变，新库定位尚未单独回放；测试/探针也以
  SHA256 保护原库；另在数据目录保留只读备份副本；
* 手动初始位姿：topic `/initialpose`（`geometry_msgs/msg/PoseWithCovarianceStamped`），
  `pose` 为 base(body) 在 `map` 帧中的位姿；`header.frame_id` 为空时按 map 处理；
  示例命令见 §11。实测：发布前 proximity 接受为 0，发布后从 map 节点 1 开始连续接受。

### 5.3 live 硬件入口（未实测）

* 入口：`slam_live_mapping.launch.py`；必须显式 `allow_live_hardware:=true` +
  `uncalibrated:=true` + 现场网络配置 `config_path`（IP 不进仓库）；
* 复用 `hei_robot_bringup` 的 MID-360S driver 与 `hei_robot_localization` 的 FAST-LIO 参数门控；
* 只启动雷达链路，不启动机器人底盘/机械臂/升降；
* **本轮没有硬件授权、未在真实传感器上执行**，文档不声称 live 已验证。

### 5.4 实时传感器只读定位（新增，尚未现场实测）

* 入口：`slam_live_localization.launch.py`，与离线定位不同，不需要 rosbag；启动现场 MID-360S driver、FAST-LIO2、XYZI 适配、RTAB-Map 定位和可选 RViz；
* 必须显式传 `allow_live_hardware:=true`、`uncalibrated:=true`、现场 `config_path` 和已有 `database_path`；仍要求 Domain 89 + localhost only；
* 强制使用定位参数 `Mem/IncrementalMemory=false`、`Mem/InitWMWithAllNodes=true`、`Mem/LocalizationReadOnly=true`，启动信息记录数据库 SHA256；退出后需用 `sha256sum` 确认数据库未变；
* RViz 使用 `slam_localization.rviz`，显示 `/cloud_map`、`/mapPath`、当前扫描和 TF。定位需在 `/initialpose` 提供 map 帧下的粗略 6DoF 初始位姿；起点冒烟可把雷达放回建图起点、近似设 `(0,0,0)`；
* 不启动底盘/机械臂/升降。该入口已实现并经静态检查/构建测试，但**尚未接真实雷达运行**，定位成功率与地图精度仍待现场验证。

地图数据库按用户要求可通过 `output_root` 指定到
`/home/robo/robotopia/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_slam/map/`。
每次建图使用新 `session`；该目录的 `.gitignore` 忽略地图产物，避免把大文件加入 Git。

### 5.5 进程有界回收

`slam_run_bounded`：`Popen(preexec_fn=恢复 SIG_DFL, start_new_session=True)`，到时长后
**只对 launch 父进程**发一次 SIGINT，12 s 有界等待，未退出则 TERM（5 s）再 KILL（2 s），
全过程记录升级步骤；退出后按进程组核对残留（有残留则逐个 INT/KILL 并记录），
`clean` 要求父进程 exit 0、无升级、无残留、日志无 `process has died`。

---

## 6. 实测结果

### 6.1 桌面运动包信息

`/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456`（只读）：
233.537 s，2335 CustomMsg，46708 IMU（约 10 Hz 雷达 / 200 Hz IMU）；
Codex 前期初检与完整 LIO 回放已通过（最大相邻位移 0.014 m，首尾估计差 8.5 cm / 4.5°，无 groundtruth）。
**这是桌面重复扫描，不是 30–50 m 运动/闭环数据。**

### 6.2 完整建图回放（Codex 独立执行，旧参数；作为旧配置基线）

| 指标 | 结果 |
| --- | --- |
| 输入 | 2335 lidar + 46708 IMU 全部观察 |
| LIO 输出 | 2331 `/Odometry`，有限、时间单调；2331 帧 `/cloud_registered_body`（8,680,103 点，XYZ 全有限） |
| XYZI 适配一致性 | 适配前后 2331/2331 帧逐帧哈希一致；字段 `x,y,z,intensity` |
| 后端输出 | 52 次非空 `/cloud_map`（653,197 点，frame=map）；221 次 `/mapPath`（53 位姿含临时当前点） |
| TF | 仅 `camera_init->body`、`map->camera_init` 两对，各唯一 publisher；历史 TF 查询 5/5 成功 |
| 退出 | 4 子进程 exit 0，无升级、无崩溃；13 项已安装运行文件哈希未变 |
| DB（SQLite 直查） | 52 持久节点 / 52 扫描；102 条有向 neighbor(type0) + 28 条有向 local-space(type2) = **14 条独立自动几何闭环**；integrity ok；优化位姿已保存 |
| 已知 WARN | 建图日志 33 条 `Missing visual features …`（旧配置空 guess 视觉分支；见 §4.2 修正） |
| 其它 | 数据库 SHA256 `afab12b8d5238c42dbc7887cc9e21fe5975c1acce6c08a705e6170af8e569d86`（2,809,856 B） |

> 证据：`/home/robo/hei-data/mid360s_slam_20261019/codex_full_observer/result.json` 与
> `codex_full_map/rtabmap_mapping.db`（Codex 持有；本文件只读引用，见 evidence 下的 refs）。

### 6.3 新参数完整建图回放（`ProximityOdomGuess=true`；Codex 独立接管收尾与审查）

运行：`mapping_proxguess_20261019_121802`，rate=2.0，只回放已有 bag；
harness 曾被外部 job 取消，随后由 Codex 接管该 launch 的进程监督：观察 bag player EOF 正常，
只对 launch 父进程发**一次** SIGINT，5 个子进程 clean exit，无升级、无残留、无 `process has died`；
父进程不是 Codex 子进程，因此 wait 退出码不可得，**不**把该次陈述为原 harness clean。

| 指标 | 结果 |
| --- | --- |
| 输入/前端 | 2331 `/Odometry`、2331 帧 body 云（8,680,103 点，XYZ 全有限，stamp 与 odom 匹配 2331/2331） |
| XYZI 适配 | 2331/2331 帧字段 `x,y,z,intensity`，逐帧哈希与 body 云一致（collector/Codex 双方核对） |
| 后端发布 | 55 次非空 `/cloud_map`（689,003 点，frame=map）；221 次 `/mapPath`（max 56 位姿/条）；历史 TF 查询 116/116 成功 |
| 时间域 | 与源包同时间域；TF 仅 `camera_init->body`、`map->camera_init` 两对 |
| 退出/回收 | 单次 SIGINT；5 子进程 clean；无升级/残留/child died；运行文件哈希无变化；父退出码不可得（操作例外见 §12） |
| `Missing visual features` WARN | **0**（旧参数完整回放为 33） |
| DB | 55 持久节点 / 55 扫描；type0=108 有向、type2=50 有向 = **25 条独立自动几何闭环**；integrity ok；2,949,120 B；SHA256 `b30fbaecb9d31c273808e4bff2e1a77bba396c2a1bf138207ee0b356a06356b3` |
| 用户手工/prior | type4 user closure = 0；不把 prior/gravity 计入自动回环 |

> 证据：`/home/robo/hei-data/mid360s_slam_20261019/mapping_proxguess_20261019_121802/`、
> `codex_final_mapping_supervision.json`、`codex_final_db_review.json`；本机只读 SQLite 复核结果一致。
### 6.4 优化点云与轨迹导出

* 接口：本机 0.23.13 `rtabmap-export --cloud --scan --poses --poses_format 11 --opt 2
  --voxel 0.05 --max_range 0`（完整命令写入结果 JSON）；
* **实测发现**：只传 `--cloud --scan` 时 0.23.13 输出 0 点并报 `Export failed! The cloud is empty.`；
  显式 `--voxel 0.05` 后成功；该约束已固化到 `slam_db_tools export` 与文档；
* **新最终库**（`mapping_proxguess_20261019_121802/rtabmap_mapping.db`，SHA256 `b30fbaec…`）：
  * 产物：`rtabmap_mapping_cloud.ply`（binary_little_endian，**48,855 顶点**，1,564,190 B，
    字段 x,y,z,intensity,nx,ny,nz,curvature，body 完整、非有限 XYZ 0）+
    `rtabmap_mapping_poses.txt`（**55 行**，每行 9 列，全部有限）；
  * DB SHA256 导出前后一致；严格校验（PLY 实体顶点/body 长度/有限 XYZ；pose 9 列有限）`passed=true`；
  * PLY SHA256 `60f7e5e663fbb349…`，poses `663f327482069074…`（完整哈希见 evidence）。
* **旧库历史导出保留**（`codex_full_map`：48,986 顶点 + 52 行轨迹，PLY `c093a5d4…`、poses `bd71b8d2…`），
  仅作旧配置对照，**不作为新库结果**；两库导出目录互相独立。

### 6.5 只读定位与 `/initialpose`（Codex 旧库，修复前/后对照）

* 地图加载：WM=52；`map -> camera_init`、`camera_init -> body` TF 历史查询 34/34 成功；
* 修复前（原配置）：66 s 回放，`/localization_pose` 66 条（注意该 topic 默认未定位也发），
  15 条视觉 WARN，日志 nearest id 150 -> 1；DB SHA256 不变；
* 修复后（`ProximityOdomGuess=true` 实验 A2，同一 DB 只读）：见 §4.2 表格，9 次 ICP proximity 接受、
  WARN 0、初始位姿发布前 0 次接受/发布后从节点 1 连续接受；
* 定位模式 DB SHA256 前后一致；进程干净退出（旧 harness）或无残留；
* **不能**把上述结果当作“3 位置现场重定位精度验收”：本轮只有单桌面轨迹、同一 DB、无 groundtruth、
  无现场外参标定。

### 6.6 RViz

* `config/slam_mapping.rviz` / `config/slam_localization.rviz`：Fixed Frame=`map`，
  显示 `/cloud_map`（后端优化地图）、`/mapPath`（优化轨迹）、`/slam/lidar_cloud_xyzi`（实时前端云）与 TF；
* 实测 `/cloud_map` 与 `/mapPath` 均非空（§6.2）；RViz 配置做静态解析测试；
* **未做人工 GUI 目视验收**；headless 会话下只验证 topic 非空与配置结构。

---

## 7. 证据口径（避免把 smoke 当验收）

### 7.1 回环计数口径

RTAB-Map 0.23.13 DB `Link.type` 枚举（本机安装头文件 `rtabmap/core/Link.h` 逐项核对）：

| type | 语义 | 是否计入自动几何闭环 |
| --- | --- | --- |
| 0 | neighbor（里程计邻接） | 否 |
| 1 | global closure（全局描述子，无图像时通常 0） | 是（本任务为 0） |
| 2 | **local space closure（空间近邻自动几何闭环）** | **是，本任务主口径** |
| 3 | local time closure（时间近邻） | 是（本任务为 0） |
| 4 | user closure（用户手工） | **否**（单独报告，不算自动成功） |
| 7/9 | pose prior / gravity | 否 |

本轮旧库：28 条有向 type2 = 14 条独立自动几何约束；Codex 同时独立验证 type2 与图优化结果。
**没有**为了桌面包降低 ICP/回环/图优化阈值来“造成功率”。

### 7.2 定位证据口径

* `/localization_pose` **不是**定位成功证据：`pub_loc_pose_only_when_localizing` 默认 false，未定位也发；
* 使用 `/info` 的 `proximity_detection_id`、`ref_id`、WM 状态、`stats`（`Loop/MapToBase*`、
  `Optimization/Iterations`、`RejectedHypothesis` 等）判断是否发生接受；
* 使用修复前后 `Missing visual features` WARN 数对比，确认视觉空 guess 分支不再触发；
* “地图加载与发布冒烟”（WM 加载、TF、cloud/mapPath 发布）与“实际几何定位接受”分开陈述。

### 7.3 已执行的负向/边界测试

* 预创建 run 目录 -> launch 拒绝（防覆盖）并在 evidence 归档；
* 未设置 Domain 89/localhost -> launch 拒绝；
* 建图目标 DB 已存在 -> 拒绝；定位 DB 不存在/空 -> 拒绝；
* `-d`/`delete_db_on_start` -> 静态 AST 测试与参数契约双双拒绝；
* 大端/行填充/畸形点云、截断 PLY、非有限 XYZ、坏 pose 行 -> 适配/导出层拒绝；
* 忽略 SIGINT 的子进程 / 孤儿残留 / 日志子节点死亡 -> harness 判定 not clean 并记录；
* 手工初始位姿订阅者不存在时不发布（collector 先查订阅者再发布，不固定 sleep 假定成功）。

---

## 8. 已知限制与 UNKNOWN（不得当作已验收）

1. **外参未标定**：只有 MID-360S 手册名义值 `t_IL=[-0.011,-0.02329,+0.04412] m`、`R_IL=I`；
   现场安装偏差、动态外参、IMU 轴符号/时间同步边界均 UNKNOWN；
2. **数据不是现场闭环数据**：桌面重复扫描（最大位移约 0.66 m）不能证明 30–50 m 闭环精度、
   回环正确性或长期漂移；
3. **无 groundtruth**：轨迹精度、回环精度、定位精度均未度量；
4. **定位只完成软件冒烟和有限的用户目视测试**：单桌面轨迹、同一 DB、无 3 位置重定位；
   `ProximityOdomGuess=true` 的纯 ICP 接受已在离线 `/info` 证实，但精度/鲁棒性未验收。
   用户报告雷达接近建图起始位姿时画面看似对齐，偏航约 90° 时出现明显错位/漂移；
   这只是用户目视反馈，缺少实时 `/info`、原始日志、误差测量且未由 Codex 独立复验，
   不能证明定位接受或精度，也不能据此区分初始猜测、场景重叠、外参和算法因素；
5. **live 已由用户报告运行，未独立验收**：用户称已完成实时建图并加载地图进行观察；
   Codex 未采集本次实时 topic/日志、数据库运行前后哈希或定位统计；雷达仍未固定到机器人；
6. **RViz 仅有用户目视反馈**：点云与地图对齐情况没有可复核的同步记录或量化指标；
7. **上游许可状态**沿用 03-LIO 的 UNKNOWN（FAST-LIO 元数据/许可证文件矛盾、ikd-Tree 无 LICENSE），
   分发前需人工确认；
8. **AGX/实机未验证**：本机 x86_64；ARM64 构建与 AGX 部署未做；
9. **时间同步**：NoSync 主机 epoch 锚点 + steady；没有 PPS/硬件同步边界实验。

---

## 9. 现场 30–50 m 闭环 + 3 位置重定位验收步骤（后续任务）

前置：完成现场外参标定并替换参数；获得运动授权与现场人员；固定雷达安装与线束。

1. **标定**：多姿态动标（充分旋转+平移激励）得到 `R_IL/t_IL`；六面静态只验证 acc 轴/单位/bias；
2. **传感器/时间**：复核 IMU 轴符号、NoSync 误差边界（PPS 或双流对齐实验）；
3. **采集**：固定小推车/移动平台采集直线、横移、转弯、静止、**30–50 m 闭环**数据；
   原包保存到独立唯一目录，禁止覆盖；
4. **建图**：用本包 `slam_offline_mapping.launch.py` 回放，检查：
   * 退出 clean、DB integrity ok、持久节点/扫描非空；
   * `Link.type=2` 自动几何闭环数 > 0 且与真实闭环位置对应（用 `/mapGraph` 与 DB 逐条核对）；
   * 不把 neighbor/prior/user link 计为回环；
5. **精度**：与卷尺/全站仪/动捕等独立测量比较闭环误差与轨迹误差（需 groundtruth）；
6. **3 位置重定位**：在同一 DB 上从 3 个不同初始位置运行只读定位，验证：
   * `/initialpose` 设置初始位姿后 `/info` 有 proximity 接受，且接受节点与真实位置一致；
   * DB SHA256 不变；重复定位结果可复现；
7. **回归**：本包 pytest 全绿、`slam_db_tools inspect/export` 严格校验通过；
8. **输出**：DB、导出 PLY/PCD、轨迹、原始日志、SHA256 清单写入数据目录唯一运行目录；
   不提交大地图或私密配置。

---

## 10. 复现命令（本机 x86_64）

**必须使用清除 Conda 变量影响的新 shell**：Conda 的 `PYTHONPATH`/`LD_LIBRARY_PATH`/
`AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH`/`PATH` 会污染 ROS 2 与本地 rtabmap 前缀。
测试与节点命令统一用 `/usr/bin/python3`。

```bash
# 0) 干净 shell（每次新终端；不要 source Conda）
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    PATH=/usr/bin:/bin:/usr/local/bin bash --noprofile --norc

cd /home/robo/robotopia/hei-rebot-lift

# 1) 依赖（无 root 本地前缀；仅首次）
extensions/tools/install_rtabmap_laptop.sh          # 或 --verify

# 2) 环境（同一干净 shell 内按顺序 source）
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
source extensions/third_party/fast_lio_ws/install/local_setup.bash
source extensions/third_party/rtabmap_local/env.sh
source extensions/ros2_ws/install/local_setup.bash

# 3) 构建 + 测试（测试用 /usr/bin/python3）
cd extensions/ros2_ws
colcon build --packages-select hei_robot_slam
cd src/hei_robot_slam && /usr/bin/python3 -m pytest test/ -q && cd ../../..

# 4) 离线建图
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ros2 launch hei_robot_slam slam_offline_mapping.launch.py \
    bag_path:=/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456 \
    uncalibrated:=true probe:=true

# 5) 只读定位 + 手动初始位姿
ros2 launch hei_robot_slam slam_localization.launch.py \
    bag_path:=/home/robo/hei-data/mid360s_lio_tests/table_motion_20261019_112456 \
    database_path:=<已有DB> uncalibrated:=true probe:=true

# 初始位姿命令放在**另开的同环境终端**，用 timeout 防止无订阅者时挂死：
timeout -s INT 15s ros2 topic pub --once --qos-reliability reliable /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 5a) 真实 MID-360S 实时建图，地图直接写入包内 map/ 子目录
map_root=/home/robo/robotopia/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_slam/map
map_session="mid360s_live_$(date +%Y%m%d_%H%M%S)"
ros2 launch hei_robot_slam slam_live_mapping.launch.py \
    config_path:=/home/robo/robotopia/hei-rebot-lift/extensions/config/local/mid360s_config.json \
    output_root:="$map_root" session:="$map_session" \
    allow_live_hardware:=true uncalibrated:=true probe:=true rviz:=true

# 建图正常 Ctrl+C 退出后，用同一数据库进行真实雷达只读定位
database_path="$map_root/$map_session/rtabmap_mapping.db"
sha256sum "$database_path"  # 记录定位前哈希
ros2 launch hei_robot_slam slam_live_localization.launch.py \
    config_path:=/home/robo/robotopia/hei-rebot-lift/extensions/config/local/mid360s_config.json \
    database_path:="$database_path" allow_live_hardware:=true \
    uncalibrated:=true probe:=true rviz:=true
# 在另一个 source 了相同环境的终端发布 /initialpose；定位退出后再次 sha256sum 核对。

# 6) 检查 DB / 导出（严格校验；不要 -d）
ros2 run hei_robot_slam slam_db_tools inspect <DB> --json inspect.json
ros2 run hei_robot_slam slam_db_tools export <DB> --output-dir <全新目录> --json export.json
```

## 11. 交付清单与证据索引

| 路径 | 说明 |
| --- | --- |
| `extensions/ros2_ws/src/hei_robot_slam/` | 新增 SLAM 接入包（launch/config/rviz/适配/探针/DB 工具/harness/测试/README） |
| `extensions/tools/install_rtabmap_laptop.sh` | RTAB-Map 固定版本依赖安装/核验脚本（amd64；无 root 本地前缀） |
| `docs/MID360S_SLAM.md` | 本文件（任务级文档） |
| `docs/agent/evidence/mid360s/20261019-slam/` | 本轮证据索引：预状态哈希、依赖/安装、环境修复、构建测试、旧参数完整回放引用、**新参数完整回放（`02b-new-config-run/`）**、导出（新旧库分开）、定位、安装器、Git 范围 |
| `/home/robo/hei-data/mid360s_slam_20261019/` | 数据输出：`mapping_proxguess_20261019_121802/`（新最终库）、`export_optimized_final_proxguess_*`（新最终导出）、旧参数 `codex_full_map`/`codex_full_observer`（只读引用）、`export_optimized_20261019_120641.final`（旧库历史导出）、定位运行目录、`_db_backups/` 只读备份 |
| `extensions/ros2_ws/src/hei_robot_slam/map/` | 用户指定的本机实时建图数据库保存目录；地图/导出数据由 `.gitignore` 排除 |

**冻结**：核心运行文件（包 `hei_robot_slam` 的 Python/launch/config 与 entry points、安装脚本）
在本轮最终 build 后冻结；冻结哈希见 evidence `03-p2-build-test/final_runtime_hashes_proxguess.json`（含 ProximityOdomGuess 修正后的最终 build）。
文档/edit 不改变运行文件；后续若改配置/算法，必须重新 build + 测试 + 重新跑对应回放，不得复用旧结论。

---

## 12. 本轮变更范围与操作例外记录

* 新增：`hei_robot_slam` 包、`install_rtabmap_laptop.sh`、本文件、evidence 目录；
* 未修改：现有 LIO 包、Livox/FAST-LIO 第三方源码、TASK_BOARD/EXECUTION_LOG（Codex 单写）、
  机器人本体代码；未 commit/push/切分支；
* 第三方运行依赖：`extensions/third_party/rtabmap_local/`（gitignored，不入库）；
* 操作例外 1：早期临时探针在 `/tmp` 一次性临时 DB 上使用过 `delete_db_on_start=true`；
  该参数随后移除，最终交付的 launch/config/测试入口**零** `-d`/`delete_db_on_start`，静态测试固定；
* 操作例外 2（2026-10-19，新参数完整建图）：DSH harness 被外部 job 取消后，launch 及其子进程由 Codex
  接管监督：观察 bag player EOF 正常，只对 launch 父进程发一次 SIGINT，5 个子进程 clean exit，
  无升级/残留/`process has died`；父进程非 Codex 子进程，wait 退出码不可得。
  因此**不**把该次陈述为原 harness clean；DB/日志/哈希由双方独立复核（§6.3）；
* 操作例外 3：定位实验曾因外层预创建 run 目录被 launch 正确拒绝；该轮记为防覆盖护栏证据，
  不记为定位失败；
* 操作例外 4：`/info` 旁观需在算法运行且 collector 存活时采集；一次性 6 s 事后旁观 0 条不作为证据。

---

## 13. 本轮最终状态（供 Codex 终审）

| 项 | 状态 |
| --- | --- |
| 运行文件 | 已冻结；源码↔install 19 项逐文件 SHA256 全部一致（含 ProximityOdomGuess 配置与最终 build） |
| 测试 | 87 passed（纯单测 + 2 个真实节点启动，覆盖契约/DB 保护/导出实体校验/有界回收/静态护栏） |
| 新最终建图 | 完整 2x 回放：55 节点/55 扫描，25 条独立 type2 自动几何闭环，视觉 WARN 0，DB `b30fbaec…` |
| 新最终导出 | PLY 48,855 顶点（body 完整/有限 XYZ 0）+ 55 行 9 列有限 poses；DB SHA 前后一致 |
| 旧库/旧测试 | 保留；旧库导出仅作历史对照，不作为新库结果 |
| 未独立验收 | 现场 30–50 m 闭环精度、3 位置实机重定位、现场外参标定、实时 `/info`/日志与定位误差、AGX/机器人固定安装；用户报告的实时建图/目视定位现象见 §8，不等同通过 |
| 结论 | 04-SLAM 软件链路 PARTIAL；不声称地图质量/定位精度/现场验收通过；不自动提交 |
