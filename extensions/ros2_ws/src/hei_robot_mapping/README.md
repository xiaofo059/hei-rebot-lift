# HEI robot mapping

AGX 上使用 FAST-LIO2 里程计和 MID-360S 点云进行 RTAB-Map 在线三维建图。
本包负责建图集成、按需保存地图及加载地图定位；必要的上游改动通过独立补丁记录。

## 节点与数据流

```text
FAST-LIO2 /cloud_registered_body (body, PointXYZINormal)
  -> /hei_robot_mapping/cloud_adapter (保留 XYZ/intensity，移除未计算的 normals)
  -> /hei_robot_mapping/scan_cloud (body, PointXYZI)
  -> /rtabmap/rtabmap <- FAST-LIO2 /Odometry (odom -> body)
```

适配器保持点坐标、frame_id 和时间戳，剔除非有限 XYZ；支持填充行和大小端输入。
输入必须是 body 坐标系，误接 /cloud_registered（odom）会报错而不篡改坐标系。
FAST-LIO2 已去畸变，不再次去畸变；不重复启动里程计或订阅原始 IMU。
RTAB-Map 使用现有 PCL ICP 后端与 g2o；以 2 Hz 处理建图帧。
点云与里程计使用精确时间同步，依赖 FAST-LIO2 当前实现的同一扫描末尾时间戳。

TF 职责：FAST-LIO2 发布 odom -> body；RTAB-Map 发布 map -> odom。
不发布重复静态 TF，不启动雷达或 FAST-LIO2；建图和定位 launch 默认启动 RViz2，不涉及底盘控制和 Nav2。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/user/hei-rebot-lift/extensions/ros2_ws
source install/local_setup.bash
colcon build --packages-select hei_robot_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

## 手动运行

先按既有流程启动雷达驱动（xfer_format:=1），等待数据稳定，再启动 FAST-LIO2。
FAST-LIO2 的 IMU 订阅采用 BEST_EFFORT（深度 10），驱动的 IMU 发布保持 RELIABLE。
确认 /Odometry 和 /cloud_registered_body 正常后，在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=0
ros2 launch hei_robot_mapping mapping.launch.py
```

所有终端须使用相同 ROS_DOMAIN_ID。不能同时运行其他发布 map -> odom 的节点。
可通过 cloud_topic、odom_topic、config_file、use_sim_time 参数覆盖相应输入。
建图 launch 默认在 AGX 本机同时打开 RViz2，Fixed Frame=map，灰色为地图点云、
青色为实时点云，并显示 TF。请在 AGX 图形桌面的终端执行。
无需图形界面时加 rviz:=false；自定义显示配置使用 rviz_config:=/绝对路径/配置.rviz。
默认 frame_id=body，与 FAST-LIO2 当前 TF 契约一致；改坐标系时须同时调整适配器。

## 输出与查看

- /rtabmap/cloud_map：地图点云（PointCloud2，通常需要 RViz 订阅后才组装发布）
- /rtabmap/mapData：地图节点和图数据
- /rtabmap/mapGraph：优化后的位姿图
- /rtabmap/info：建图及回环状态
- TF map -> odom：地图对里程计的修正

RViz Fixed Frame 使用 map，PointCloud2 选择 /rtabmap/cloud_map。
查看中间点云时可选 /hei_robot_mapping/scan_cloud，需有相应 TF。

## 保存地图

mapping.yaml 显式 database_path=''，RTAB-Map 使用内存数据库。
**先保存成功，再关闭建图。** 建图退出后内存中的地图无法补存。

在 AGX 另开终端，加载环境，保持与建图终端相同的 ROS_DOMAIN_ID 和
ROS_LOCALHOST_ONLY，然后运行（将 warehouse_01 换成你的地图名）：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_robot_mapping save_map.launch.py map_name:=warehouse_01
```

默认生成：

```text
/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map/warehouse_01.db
```

看到 `Saved ... (N nodes, N scans, ... directed constraints)` 表示保存并回读校验成功，
保存节点随后自动退出；原建图节点继续运行，可换一个名字再次保存。
同名文件会拒绝覆盖，中文名字及可选 `.db` 后缀均可使用。地图文件不纳入 Git。
建议停止移动雷达后保存；大地图的数据获取、写入和校验需要时间。

也可直接运行节点：

```bash
ros2 run hei_robot_mapping map_saver --ros-args -p map_name:=warehouse_01
```

保存节点参数（launch 参数同名）：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| map_name | 必填 | 单个文件名，不能包含目录路径 |
| output_dir | 上述源码目录中的 map 文件夹 | 可自定义绝对保存目录 |
| rtabmap_node | /rtabmap/rtabmap | 读取当前算法参数的节点 |
| map_service | /rtabmap/rtabmap/get_map_data | 读取地图数据的服务，默认由 rtabmap_node 拼接 |
| wait_timeout_sec | 10.0 | 等待服务的最长时间（秒） |
| request_timeout_sec | 120.0 | 单次服务请求的最长时间（秒） |

例如：

```bash
ros2 launch hei_robot_mapping save_map.launch.py \
  map_name:=floor_02 output_dir:=/home/user/maps
```

`.db` 保存当前纯激光地图快照，包括逐节点激光点云、占据网格数据、原始里程计位姿、
图约束、优化后的位姿及 RTAB-Map 算法参数，可供 RTAB-Map 后续重新加载。
它不是 PCD/PLY 点云导出，也不生成 Nav2 的 PGM/YAML 二维地图。
保存节点只读取地图服务，不调用 backup、load_database、reset 或关闭建图服务，
适用于当前的内存建图。它不复制原运行数据库中的统计历史、预览图或预生成网格缓存。
当前专用于 Reg/Strategy=1 的纯激光配置，含视觉词典的地图会明确报错。

无服务、空地图、缺少激光数据、同名文件或保存失败均返回错误。
写入临时文件并回读节点/点云/图约束后，才发布最终 `.db`，避免留下看似成功的空地图。
长期运行仍会增长内存，应按实际地图规模评估；保存快照不会清空运行中的地图。

## 加载地图定位与重定位测试

先保存地图，停止建图 launch，再单独启动雷达（xfer_format:=1）和 FAST-LIO2。
两者启动稳定后，在 AGX 另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_robot_mapping localization.launch.py map_name:=warehouse_01
```

默认读取 map/warehouse_01.db，支持 map_name:=warehouse_01.db。
自定义地图目录使用 map_dir:=/home/user/maps；不创建缺失的数据库。
地图按只读方式打开，加载全部地图节点，关闭定位时不会写回原数据库。
本 launch 启动点云适配器、RTAB-Map 和 RViz2；雷达、FAST-LIO2 单独启动。
默认 RViz2 的 Fixed Frame=map，灰色显示已保存地图，青色显示实时去畸变点云，
同时显示 TF；2D Pose Estimate 发布 /initialpose。请在 AGX 图形桌面终端运行。
无图形界面或已有 RViz 时加 rviz:=false；自定义配置使用 rviz_config:=/绝对路径/配置.rviz。
**不能与 mapping.launch.py 同时运行**，否则会重复启动适配器和发布 map→odom。
各终端的 ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY 必须一致。

默认初始位姿为 `0 0 0 0 0 0`：假设 body 在原建图起点、朝向与原建图开始时相同。
从其他位置启动时，提供 body 在 map 中的大致位置和朝向：

```bash
ros2 launch hei_robot_mapping localization.launch.py map_name:=warehouse_01 \
  initial_pose:="1.0 2.0 0.0 0.0 0.0 1.5708"
```

六个数依次为 **x、y、z、roll、pitch、yaw**，位置单位米、角度单位弧度；
上例表示约 (1, 2, 0) 米、偏航约90°。这是初值，不是定位成功的证明。
`cloud_topic`、`odom_topic`、`use_sim_time` 参数与建图 launch 相同。

在 AGX 本机 RViz 中设置 Fixed Frame=map，显示 /rtabmap/cloud_map、
/cloud_registered 和 TF。若实际起点与初值不同，也可以在 RViz 使用
**2D Pose Estimate** 给出大致位置及朝向（默认发布 /initialpose，本 launch 已对接）。
该工具提供平面初值；有明显高度/倾斜差异时应使用六自由度 initial_pose，
或自行发布 map 坐标系下的 PoseWithCovarianceStamped 到 /initialpose。

观察实时点云是否与地图重合、body位置是否符合实际，并检查 /rtabmap/info 的匹配结果
及 map→odom 修正是否合理。仅看到地图或TF不代表匹配成功。
定位模式下不向地图添加节点，静止时也继续尝试配准。
本模式是**给定大致初值后的纯激光定位**；ICP对初值、重叠区域和场景几何有要求，
不保证从任意未知位置或雷达突然搬动后自动全局重定位。
定位专用配置将 Icp/MaxTranslation 设为1米，以允许修正近似初值，
保留点云对应关系筛选和图优化一致性检查；这不保证1米内的任意初值都能收敛。

## 导航前启动定位与 relocalize 服务

导航前使用独立入口 `relocalize.launch.py`。它复用上面的只读定位 launch，
启动点云适配器、RTAB-Map 和一个轻量重定位服务节点，默认不启动 RViz2；
雷达驱动、FAST-LIO2 需要先启动稳定，本入口不启动 Nav2 或底盘。
不能与 `mapping.launch.py` 或 `localization.launch.py` 同时运行。

编辑 `config/relocalize.yaml`：

```yaml
/**:
  ros__parameters:
    database_path: /home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map/warehouse_01.db
    initial_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    auto_relocalize: true
    service_timeout_sec: 15.0
    localization_timeout_sec: 60.0
    required_matches: 3
    data_timeout_sec: 3.0
```

以上是主要参数，完整文件还包含固定服务端、坐标系和话题名称。
`database_path` 是已存在的 `.db` 绝对路径，不创建缺失地图；
`initial_pose` 是 body 在已保存 map 中的大致六自由度位姿（米、弧度）。
零初值假设回到原建图起点和朝向；从其他地方启动应修改初值，不能保证未知位置全局搜索。

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_robot_mapping relocalize.launch.py \
  config_file:=/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/config/relocalize.yaml
```

默认不启动 RViz2，需要检查定位时可加 `rviz:=true`。后续导航启动入口负责加载
二维 PGM/YAML 的 map_server 和导航所需的 RViz2。显式传入源码 YAML 后，
修改配置无需重新编译，重启此 launch 即生效；运行中不会自动热切换地图或参数。
省略 `config_file` 时使用 install 副本，修改源码 YAML 后需重新编译本包更新副本。
原来的 `localization.launch.py map_name:=...` 用法保持可用。

启动后 `auto_relocalize=true` 会自动调用一次封装服务。再次请求：

```bash
ros2 service call /hei_robot_mapping/relocalize std_srvs/srv/Trigger '{}'
```

**当前 RTAB-Map 没有原生 `relocalize` 服务。** `/hei_robot_mapping/relocalize`
是本包提供的封装：先检查目标 RTAB-Map 的数据库路径、坐标系、只读定位参数，
再调用原生 `/rtabmap/rtabmap/set_mode_localization`，发布配置初值到 `/initialpose`。
地图在 launch 启动时加载，不调用具有切换/清空地图语义的 `load_database` 或 `reset`。
修改地图路径后应重启 launch，再次调用服务会重新使用本次启动配置中的初值。
请求等待匹配期间，也可在 RViz 用 2D Pose Estimate 修正平面初值；
下一次服务调用仍会使用 YAML 中的初值，需要持久修正时应修改 YAML 后重启。

服务不会在调用成功或 TF 出现后立即报告定位成功：它要求同一新时间戳的
`/rtabmap/info` 已接受激光回环/邻近匹配和 `/rtabmap/localization_pose` 有限有效位姿，
同时 `map -> odom -> body` 在该时间戳有可用变换。默认连续 3 帧才返回 `success=true`。
拒绝的匹配、旧消息、无效位姿/协方差、缺少 TF 均不能满足条件；
服务不可用或超时返回 `success=false`，并发请求明确返回“正在重定位”。
服务响应中的 success 表示满足上述软件条件，不是实际定位精度的保证。

`/hei_robot_mapping/localization_ready` 是持久发布的 Bool：成功后 true，
新请求、后续匹配失败或超过 `data_timeout_sec` 没有新成功匹配时恢复 false。
订阅时如需立即获取当前值，应使用 transient_local durability。

```bash
ros2 topic echo /hei_robot_mapping/localization_ready \
  --qos-durability transient_local --once
```

确认 ready=true，且 RViz 中实时点云与地图重合、位置朝向符合实际后，再启动 Nav2。
此节点不自动启动导航或控制电机；导航期间如何处理 ready 变化需由后续导航集成决定。
RTAB-Map 的 ICP 仍依赖合理初值和场景重叠，不能把该封装当成任意位置的全局重定位算法。
服务节点仅订阅低频 Info/位姿和 TF，不订阅点云，不修改 FAST-LIO2/RTAB-Map 主节点源码。

## 验证

```bash
colcon test --packages-select hei_robot_mapping --event-handlers console_direct+
colcon test-result --test-result-base build/hei_robot_mapping --verbose
ros2 launch hei_robot_mapping mapping.launch.py --show-args
```

单元测试覆盖法向量移除、时间戳/坐标保持、非有限点过滤、行填充、字节序和非法输入。
保存测试覆盖点云与占据数据、双向图约束、优化位姿和算法参数的数据库往返，
RTAB-Map 核心重新加载、空地图拒绝、文件名校验与同名文件保护。

2026-10-26：AGX 原生编译通过，点云转换测试全部通过，launch 参数加载通过；
算法参数已与本机 RTAB-Map 参数清单核对，安装文件一致性与动态依赖检查通过。
验证日志位于 ros2_ws/log/hei-mapping-*.log。未启动建图节点、未修改或重启现有驱动。

2026-10-27：保存节点在 AGX 原生编译通过；原有 4 项转换测试及新增 4 项保存测试全部通过。
使用软件点云生成真实 RTAB-Map 内存地图，保存 10 个节点后重新启动 RTAB-Map，
成功读取 10 个节点、10 份激光数据及 10 个优化位姿；保存前后原节点地图保持不变。
完整 launch 调用、空地图、服务不可用、非法名称及同名拒绝的退出状态均已验证。
日志位于 ros2_ws/log/save_map_validation_20261027/。没有启动实机驱动；
用户实际建图数据的保存与后续定位仍需单独验收。

2026-10-27：独立 localization.launch.py 在 AGX 编译安装通过。隔离 ROS 域中加载
10 节点软件测试地图，处理 14 帧、接受 9 帧匹配，验证初始位姿参数和 /initialpose 对接；
退出后原数据库 SHA-256 不变。缺失地图、非法名称和非有限/格式错误位姿均在启动前拒绝。
日志位于 ros2_ws/log/localization_validation_20261027/。此验证未启动实机驱动，
不能替代实际环境中的定位精度和重定位验收。

### g2o 信息矩阵不对称错误

2026-10-27 实际 FAST-LIO2 数据回放复现 `Information Matrix for an edge is not symmetric`，
继而出现 `OptimizerG2O.cpp` 错误和 `Optimization failed, rejecting localization`。
原地图 218 条约束的矩阵有效；实时协方差及其求逆存在浮点舍入造成的微小不对称。
仅在输入端对称化不足以消除求逆后误差。

已对 RTAB-Map 6D 图边的信息矩阵加入有限且近似对称（相对容差 1e-10）的舍入修正，
保留权重、正定性验证和图优化一致性检查，不忽略协方差、不放宽回环拒绝阈值。
补丁和上游基线见 `patches/README.md`；RTAB-Map 源码属于独立仓库，补丁在本包保留。
AGX 原生编译安装通过。同一批 69 对真实输入回放，修正前 22 次优化失败、
修正后 0 次不对称错误/优化失败且 22 次定位匹配被接受，数据库 SHA-256 不变。
日志位于 ros2_ws/log/localization_error_20261027/。
已运行节点仍使用旧库，必须停止并重新启动定位 launch 才生效；雷达和 FAST-LIO2 可继续运行。
这是有限时段回放验证，长时间移动定位效果仍需实机验收。

## FAST-LIO2 世界系重力对齐（2026-10-28）

AGX FAST-LIO2 已改为在 IMU 初始化时将平均比力方向旋转到世界 +Z，
并将初始世界重力设为 -Z。该行为现在由 FAST-LIO2 YAML 的 `gravity_align: true` 控制，
`imu_init_num: 100` 控制初始化样本数。odom 因而在初始化时水平，body 保留实际倾角。
重力不能确定绝对航向；算法使用最短倾斜旋转，不提供指北航向。
初始化后重力仍是滤波器估计状态，不是每帧强制重置的姿态约束。

启动时仍需让雷达静止，运动加速度会干扰重力估计。原点仍是启动位置而非地面，
因此生成二维地图前仍要确定地面高度。此改动不会旋转已有 .db/PCD，
RTAB-Map 当前配置也不强制 map 的滚转/俯仰与重力一致。
若正在建图，应先保存地图并停止建图/定位，再重启 FAST-LIO2 和上层节点，
避免运行中改变 odom 定义。已有进程继续使用旧程序，重启后才生效。
完整可重放补丁及测试说明见 extensions/patches/README.md。

重力对齐验证：AGX 原生编译安装通过，实际 IMU 初始化路径的 127 个初始化场景通过。
隔离域软件点云对比中，倾斜雷达首帧地面 Z 标准差由约 0.99 米降到 1.28 毫米。
连续 8 秒模拟的末帧仍有姿态/高度变化（地面 Z 标准差约 3.49 厘米），
因此只确认初始化及初始输出的重力对齐，不宣称持续运行始终严格水平。
未做实机静止/移动验收。详见 extensions/patches/README.md 和
ros2_ws/log/gravity_alignment_20261028/ 中的完整记录。

AGX 的默认、test、fine 和固定外参配置均使用 `world_frame: odom`、`body_frame: body`。
RTAB-Map 的 `odom_frame_id` 留空，会从同步的 `/Odometry` 消息中读取父坐标系。
修改 YAML 不会改变已启动的 FAST-LIO2；保存需要的地图后重启 FAST-LIO2 和建图/定位
节点，再重新打开 RViz，新的 TF 链为 `map -> odom -> body`。RViz Fixed Frame 保持 `map`。

RViz 地图点云 `/rtabmap/cloud_map` 使用 Reliable + Transient Local，以接收静止时
不再重复发布的缓存地图。Fixed Frame 使用 map，避免对旧地图消息请求已过期的 TF。
实时点云仍使用 Best Effort；默认视距 15 m，过度放大会看不到周围墙面和地面。


## 离线地图导出：DB 转三维 PCD 和 Nav2 二维地图

使用独立的 `map_projector` 工具读取已保存 `.db` 中的优化位姿和原始激光扫描，
将点变换到原地图坐标系，一次导出两种地图：

- `.pcd`：完整三维全局点云，可供后续 ICP 定位模块加载。
- `.pgm` + `.yaml`：筛选 **map 坐标系 Z ∈ [-0.20, 0.80] m** 的点，沿 Z 投影到 XY 栅格。

不依赖正在运行的雷达、FAST-LIO2 或 RTAB-Map，
不会修改数据库，也不会修改建图、定位或里程计节点。
请先用保存节点完成快照，输入应为已保存完成、没有其他程序正在写入的数据库。

在 AGX 终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_robot_mapping project_map.launch.py \
  db_path:=/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map/warehouse_01.db \
  output_directory:=/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map \
  config_file:=/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/config/map_projection.yaml
```

输入数据库路径用 `db_path` 指定，输出目录用 `output_directory` 指定（不存在时创建）。
三个输出始终采用输入数据库的原文件名，不再附加 `_2d`：

```text
warehouse_01.db -> warehouse_01.pcd
                   warehouse_01.pgm
                   warehouse_01.yaml
```

看到 `Saved ...` 后工具自行退出。省略 `output_directory` 且配置中也为空时，输出到输入 DB 所在目录。
任一同名文件（包括符号链接）存在时，整次导出失败，不覆盖或生成部分结果；重新转换请换一个输出目录。
仍兼容旧的 `map_name:=warehouse_01` 调用，输入从 `map_directory` 查找；`map_name` 与 `db_path` 不可同时指定。
原 `output_name` 参数已移除，输出名统一从 DB 文件名推导。

PCD 使用 binary XYZI（float32）格式，坐标单位为米，坐标系为原 `map`，
包含数据库中所有具有优化位姿的三维扫描点。每帧均应用“优化位姿 × 扫描外参”，
不会使用未经优化的原始里程计轨迹拼接。PCD 不受高度区间影响，也不额外降采样或去重；
非有限 XYZ 点被跳过，强度字段原样保留（数据库无强度或强度非有限时填 0）。
PCD 中的 VIEWPOINT 为单位变换，点已经变换到全局 map 坐标系。
工具按扫描逐帧读取并写入 PCD，不将整张三维地图积累到内存。
本工具只导出文件；后续 ICP 定位需要另外配置定位模块的预处理及初始位姿。

转换参数在 `config/map_projection.yaml`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `db_path` | 空 | 输入 DB 的路径，通常用同名 launch 参数覆盖 |
| `map_directory` | 本包源码下的绝对 `map` 路径 | 仅兼容旧 `map_name` 调用时使用的输入目录 |
| `output_directory` | 空 | 空时输出到输入 DB 的目录，也可用同名 launch 参数覆盖 |
| `z_min` / `z_max` | `-0.20` / `0.80` | 仅二维投影：map 高度上下界，包含边界，浮点容差 1 微米 |
| `resolution` | `0.05` | 栅格分辨率，米/格 |
| `padding` | `0.50` | 地图边界外扩距离，米；外扩区域保持未知 |
| `raytrace_free_space` | `true` | 将实际观测射线与高度区间相交的部分标为空闲 |
| `max_cells` | `25000000` | 最大栅格数，防止异常坐标或过细分辨率耗尽内存 |

通过上面命令显式指定源码中的 `config_file` 后，修改该 YAML 无需重新编译，
再次运行转换即可。省略 `config_file` 则使用 `install` 下的配置副本，
源码配置变更后需重新 `colcon build --packages-select hei_robot_mapping` 才会更新副本。

输出 `.yaml` 包含 Nav2 的 `image`、`mode`、`resolution`、`origin`、`negate`、
`occupied_thresh`、`free_thresh`，并附带 `conversion` 配置快照和 PCD 文件名、点数，方便追溯本次转换。
`image` 使用相对文件名，请将输出的 PGM/YAML 放在一起。
栅格原点对齐原 map 的 XY 网格，偏航为 0，PGM 行方向按 ROS 地图约定翻转。
黑色（0）是高度区间内的障碍，白色（254）是观测射线推断的空闲，灰色（205）是未知。
障碍优先，不会被其他射线清除；未观测区域不会填成空闲。

**高度是相对于原 map 原点，不是距地面高度。** 工具不会自动旋转旧地图、
修改地面高度或删除地面；如果地面落在这个高度区间内，也会按要求投影为障碍。
若实际需要“地面以上 -20 cm 到 80 cm”，需先确定地面在 map 中的 Z，再相应调整上下界。
射线推断空闲只表示该条激光路径观测为空，不保证整个垂直体积都已观测；
二维地图必须先检查墙体、窄道和可通行区域，再用于实际导航。
此工具不做障碍膨胀或形态学填充，导航的机器人 footprint 和 inflation 应在 Nav2 中配置。

单元测试覆盖优化位姿/激光外参应用、全局高度裁剪及边界、PGM 行方向、
空闲射线与未知区域、障碍优先、数据库只读、无数据/无优化位姿、栅格上限及同名保护。
PCD 测试通过 PCL 重新读取，检查多帧优化变换、旋转、强度、完整高度范围、NaN 过滤及同名输出。

2026-11-05：AGX 原生编译通过，新增 7 项转换测试及原有点云/保存测试全部通过。
使用实际 warehouse_01.db（62 份扫描）完成默认范围投影，输出 220×475 格、0.05 m/格；
PGM/YAML 解析、三态阈值、YAML 参数覆盖、同名保护、缺失输入和空高度区间退出均已验证，
原数据库 SHA-256 保持不变。记录在 ros2_ws/log/map_projection_validation_20261105/。
本次仅做离线转换，没有启动实机驱动或规划器；未进行 Nav2 实车通行验收。

2026-11-06：新增同名三维 PCD 导出及 `db_path` 输入，AGX 编译与全包 17 项测试通过。
warehouse_01.db 实图导出 290840 个 XYZI 点；PCD 的 Z 范围约为 [-1.165, 8.216] m，
没有被二维高度区间裁剪。默认 PGM 与上一版导出逐字节相同，DB SHA-256 保持不变。
已验证同名拒绝、缺失 DB 的非零退出、中文/空格路径及 YAML 覆盖；改变二维投影高度和
分辨率后，PCD 仍逐字节一致。记录在 ros2_ws/log/map_export_validation_20261106_4mv3xoi3/。
