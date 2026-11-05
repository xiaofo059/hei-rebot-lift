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
