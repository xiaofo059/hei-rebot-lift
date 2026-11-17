# HEI Nav2 导航框架（ROS 2 Humble）

本包提供独立的 Nav2 启动与配置入口，Nav2 主程序使用 AGX 上的系统安装包。
当前阶段完成服务框架，暂不承担任务调度、精确停靠、自动重定位恢复或实车导航验收。

## 与现有节点的连接

```text
RTAB-Map 定位：map -> odom
FAST-LIO2：   odom -> body，/Odometry，/cloud_registered_body
导航安装偏移：body -> base_link（50×50 cm 模型，雷达位于前边缘中点）
底盘轮速反馈：/chassis/odom（只取速度，不使用其 wheel_odom 位姿或 TF）

二维地图 YAML + PGM -> map_server -> /map -> 全局代价地图 -> planner_server
实时三维点云 --------------------------------> 局部代价地图 -> controller_server
NavigateToPose / NavigateThroughPoses -> bt_navigator -> 规划、跟踪和恢复行为
controller_server / behavior_server -> /hei_robot_navigation/cmd_vel_raw
                                   -> velocity_smoother
                                   -> /cmd_vel（默认底盘速度输入）
```

Nav2 使用底盘中心的 `base_link`，与底盘速度指令和 `/chassis/odom` 的速度参考点一致。
按 50×50 cm 导航模型、雷达 `body` 位于前边缘中点的约定，由本 launch 发布
`body -> base_link` 的静态变换 `(-0.25, 0, 0)` 米。`base_link` 位于矩形中心，
`body` 在 `base_link` 的前方 25 cm，恰好位于轮廓前边缘中点。
安装偏移仍是独立物理参数；后续以实际雷达安装测量值为准。
这里的 `z=0` 表示与雷达同高，并非已经测量了雷达离地高度，也不是地面坐标系。
雷达 `body` 与真实 IMU/激光测量原点的差异、实际安装位置仍需后续实测校正。
`config/nav2_params.yaml` 的 `body_to_base_link.ros__parameters` 可修改此变换；
若后续 URDF/robot_state_publisher 已发布该连接，将其中 `publish_tf` 设为 `false`，
避免两个节点同时发布同一 TF。该配置由 launch 读取，修改后需要重启 launch。
底盘控制器自己的轮式里程计 TF 继续关闭。
本包不启动雷达、FAST-LIO2、RTAB-Map、底盘控制器或 AMCL，不发布 `map -> odom` 定位 TF。
RTAB-Map 使用的 `.db` 与这里加载的二维地图必须来自同一次建图和同一 `map` 坐标系。
当前默认搭配为 `lab_01.db` 与其导航修正版 `lab_01_nav.yaml`，地图原点和分辨率未变。

源码核对发现，当前 FAST-LIO2 的 `/Odometry` 只填充位姿，`twist` 保持零值，不能直接
当作 DWB 的实际运动速度。此配置的 `odom_topic` 因而预留为已有底盘的 `/chassis/odom`；
Nav2 的机器人位置仍通过 FAST-LIO2/RTAB-Map 的 TF 获得，再经安装偏移变换到底盘中心。
后续联动需实测确认雷达与底盘的安装位置和方向。

## 安装与编译

系统依赖在 **AGX 终端** 安装：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-spatio-temporal-voxel-layer
```

编译本包即可，不需要重新编译已经部署的 SLAM 或底盘源码：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/hei-rebot-lift/extensions/ros2_ws
colcon build --packages-select hei_robot_navigation --symlink-install
source install/setup.bash
```

## 当前阶段的启动方式

先按原有流程启动雷达、FAST-LIO2 和定位：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_robot_mapping relocalize.launch.py
```

检查 `/hei_robot_mapping/localization_ready` 为 `true`，并确认实时点云与地图对齐。
此框架尚未把该状态接成自动激活/停车联锁。

另一终端启动框架：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_robot_navigation navigation.launch.py
```

默认行为：

- `map_server` 自动激活，发布 `/map`；默认地图由所选参数文件的
  `map_server.ros__parameters.yaml_filename` 指定。
- 打开导航 RViz，显示二维地图、TF、路径、代价地图和轮廓；实时点云可手动勾选。
- `autostart=true`：同一个生命周期管理器先配置地图，再配置七个导航服务；
  激活时先激活地图并发布 `/map`，然后激活导航服务。地图启动失败会阻止后续激活。
  `autostart:=false` 时，地图和导航服务均等待手动配置、激活。
- 速度默认经平滑器输出至底盘的 `/cmd_vel`；恢复行为也经过同一个速度平滑器。
- 启动不会自动创建导航目标；在 RViz 中发送 Nav2 Goal 后开始执行导航任务。

启动导航但不打开 RViz 窗口：

```bash
ros2 launch hei_robot_navigation navigation.launch.py rviz:=false
```

### RViz 启动与 DDS 传输

RViz 默认延后 5 秒启动，避开 Nav2 多进程集中初始化。AGX 对照中，同步启动时曾出现
已收到地图但视窗全黑；导航就绪后单独启动连续三次显示正常。可通过
`rviz_start_delay:=0.0` 恢复同步启动，或按机器负载调整间隔。该间隔不是导航就绪判断。

本 launch 的地图服务、Nav2 服务、生命周期管理器、安装偏移 TF 和 RViz 默认设置
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。原默认传输复测中曾出现某个导航节点收不到
`odom` TF，而独立观察节点能收到的情况；UDPv4 对照连续五次全部服务激活、地图正确接收，
RViz 均正常显示。此前也捕获过 RViz 进程存在但窗口不出现的 DDS 初始化挂起。
这是基于本机对照的传输绕过措施，不代表已证明 Fast DDS 底层故障的具体位置，
也不能保证修复所有 OpenGL 黑屏问题。
设置仅传给本 launch 创建的进程，不切换 RMW，不改变雷达、FAST-LIO2、定位或底盘
节点的环境，也不改变 IMU 的 BEST_EFFORT 配置。

如需对照 Fast DDS 默认的 UDP + 共享内存传输：

```bash
ros2 launch hei_robot_navigation navigation.launch.py \
  nav_dds_transport:=DEFAULT rviz_dds_transport:=DEFAULT
```

如果已有导航服务正常运行，仅需重开显示窗口，可关闭原 RViz 后在 AGX 本机终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 run rviz2 rviz2 -d \
  /home/user/hei-rebot-lift/extensions/ros2_ws/install/hei_robot_navigation/share/hei_robot_navigation/config/navigation.rviz
```

若终端没有沿用现有节点的 `ROS_DOMAIN_ID`，需先设置为相同的值。
不要为重开 RViz 重复启动一套 Nav2。窗口出现但黑屏时，还需核对 RViz 图形日志、
地图订阅和 TF；`Managed nodes are active` 只代表 Nav2 生命周期服务已激活。

### 地图服务器生命周期启动

AGX 上曾捕获 `map_server` 已成功读入 PGM，但配置服务报
`failed to send response ... client will not receive response`，地图服务器停在
`inactive`、没有发布 `/map`。原来独立的导航生命周期管理器仍会激活导航，
导致全局代价地图持续报告 `no map received` 和 `Robot is out of bounds`。

本 launch 给地图服务器、七个导航服务和生命周期管理器单独加载
`config/fastdds_services.xml`。它将 Fast DDS 的 `service` 应答 writer 的
`reliability.max_blocking_time` 从默认 100 ms 延长至 3 s，为启动期间客户端发现留出时间；
不设置默认话题 writer，不切换 DDS，不改变传感器或底盘节点、IMU BEST_EFFORT。
这不是固定等待 3 秒，发现完成后会立即返回。
地图与导航同时交给 `lifecycle_manager_navigation` 顺序管理，地图先激活，
避免地图启动失败时导航仍显示激活成功。旧的 `lifecycle_manager_map` 不再启动。
`autostart:=false` 现在同时控制地图与导航，软件联调时需留意此区别。

临时换二维地图：

```bash
ros2 launch hei_robot_navigation navigation.launch.py \
  map:=/绝对路径/地图名.yaml
```

换整套参数（可直接读取源码 YAML）：

```bash
ros2 launch hei_robot_navigation navigation.launch.py \
  params_file:=/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_navigation/config/nav2_params.yaml
```

源码配置的修改需重启 launch；使用这里的 `--symlink-install` 构建后，配置和 launch
文件通过符号链接安装，通常无需因 YAML 修改重新构建。修改 `package.xml` 或 CMake
安装规则仍需要重新构建。

## 文件与基础参数

- `launch/navigation.launch.py`：地图服务、七个导航服务、一个有序生命周期管理器、
  可选安装偏移静态 TF 和可选 RViz。
- `config/fastdds_services.xml`：仅 Nav2 服务应答的 Fast DDS writer 配置。
- `config/nav2_params.yaml`：地图路径、坐标、话题、规划/控制插件、局部/全局代价地图。
- `config/navigation.rviz`：导航显示配置，Fixed Frame 为 `map`，默认使用 Orbit 视角；
  左键拖动旋转，中键或 Shift+左键拖动平移，滚轮缩放。右侧 Views 的 Type 可切回
  TopDownOrtho 使用纯二维俯视。默认观察距离 15 m、俯视角 1.2 rad，焦点为地图原点。

当前全局规划使用 NavFn（A*），局部控制使用允许横移的 DWB；它们作为可替换的
框架基线，并不是已针对实验室调好的最终算法选择。全局代价地图为二维静态地图加膨胀，
局部代价地图为 `/cloud_registered_body` 三维点云的 STVL 时间衰减障碍层加膨胀。

当前全局和局部导航轮廓均为以底盘中心为原点的 **50×50 cm 矩形**，顶点为
`(±0.25, ±0.25)` 米，`footprint_padding=0.02` 米，带 padding 的内切半径为 0.27 米。
这是用户指定的导航轮廓，需确认能覆盖真实车身与伸出部件；较小轮廓不代表实际车身缩小。
DWB 使用 `ObstacleFootprint` 检查矩形边界。
本次轮廓调整保留用户当前设置：全局 `inflation_radius=0.08` 米、
`cost_scaling_factor=5.0`，局部 `inflation_radius=0.15` 米、`cost_scaling_factor=2.0`。
`inflation_radius` 从障碍栅格起算，不是车身外的额外间隙；当前全局 8 cm、局部 15 cm 均小于带 padding
的 27 cm 内切半径，不能覆盖完整车身所需的内切膨胀范围。NavFn 仍可能生成贴近障碍、
被局部完整轮廓碰撞检查拒绝的路径。后续需结合真实尺寸和现场通行情况单独调整膨胀配置。
实际栅格边界受 5 cm 分辨率影响，软代价带本身不保证固定的机身净空。
调低膨胀不能消除静态地图中的实际障碍或轮廓碰撞。
RViz 全局/局部代价地图 Alpha 分别为 0.12/0.25，降低覆盖地图和绿色轮廓的颜色强度；
全局代价地图默认不勾选，需要检查膨胀时在 Displays 中勾选 `Global Costmap`。
这只调整显示，不改变规划代价。青色对应内切碰撞代价，不能当成可通行的低代价区域。
NavFn 仍按参考点的膨胀代价规划，不做矩形在每个朝向的完整碰撞搜索；
矩形角部由局部 DWB 检查，因此狭窄通道的可达性仍需实测。
速度框架上限低于现有底盘限制。
局部障碍物初始高度范围为 `[-0.2, 0.8] m`，数值相对于 `odom`，不是距地面高度；
它沿用现有地图投影的初始高度带，雷达安装高度和机身遮挡尚需实测。

默认速度输出已接入 `/cmd_vel`，不要与键盘控制同时发送命令。
完整机器人轮廓（包含机械臂状态）、雷达安装偏移、点云高度、定位丢失处理及
规划/控制参数仍需结合实车验证。

## 动态障碍离开后的清除

`config/nav2_params.yaml` 的 `local_costmap.local_costmap.ros__parameters.stvl_layer`
使用 Humble 的 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`，保留 5 cm
二维地图分辨率、10 Hz 更新、5 Hz 发布、现有轮廓和局部 15 cm 膨胀半径。
全局地图仍为静态地图加膨胀。

原 `nav2_costmap_2d::VoxelLayer` 没有障碍时间过期机制，需要后续射线穿过同一旧体素；
人员离开后，稀疏点云或遮挡可能使旧占用长期残留。`observation_persistence=0` 只控制
观测缓冲，不表示已写入地图的障碍立即消失。缩小膨胀半径或提高发布频率不会清除旧占用。

现在使用以下 STVL 官方参数：

- `voxel_decay: 2.0`、`decay_model: 0`：最后实际观测后约 2 秒过期；持续观测会续期。
- `voxel_size: 0.05`：三维体素边长 5 cm。
- `cloud.observation_persistence: 0.0`、`cloud.clear_after_reading: true`：只消费新观测，
  不用同一帧旧点云反复续期。
- `cloud.expected_update_rate: 0.5`：输入间隔超过 0.5 秒时，该层报告数据不新鲜；单位是秒。
- `cloud.marking: true`、`cloud.clearing: false`：通过新观测标记、通过时间过期清除。
  STVL 的 `clearing` 指视锥加速清除，关闭它仍会执行上述时间衰减；此处不假设雷达视场形状。
- `cloud.filter: passthrough`：保留当前 `odom` Z 的 `[-0.2, 0.8] m` 高度范围。
  STVL 没有标准 VoxelLayer 的 `obstacle_min_range` 参数；当前 FAST-LIO2 的
  `preprocess.blind=0.35 m` 在上游过滤近距离点，`cloud.obstacle_range=5.0 m` 限制最远标记。

这表示“最近 2 秒未再观测到”，不能独立区分障碍确实离开和暂时被遮挡。此时间针对当前
低速、10 Hz 点云的室内测试起步配置，需结合现场遮挡情况验收。若稀疏固定障碍闪烁，
先增大 `voxel_decay`（例如 3 秒），并检查当前帧点云是否持续覆盖它。持续被误识别的地面、
自体回波不会靠衰减消失，需要单独校准安装高度或过滤点云。

修改此配置后需重启导航 launch 重新加载插件；不用重启雷达、FAST-LIO2 或定位。
使用 `--symlink-install` 时 YAML 通过符号链接安装；新增依赖后仍应编译一次本包。
现场验证时保持底盘静止、不要发送导航目标：人在雷达前停留，应持续有障碍；离开后，
在点云中确认该处已无回波，检查局部障碍及膨胀是否约 2～3 秒消退，固定障碍是否仍保留。

参考：[STVL Humble 源码](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/humble/spatio_temporal_voxel_layer)。

## 软件联调入口

默认自动激活导航并向 `/cmd_vel` 发布速度。仅加载框架、不自动激活导航时，可显式设置
`autostart:=false`；隔离速度输出用于软件联调时，可显式选择预留话题：

```bash
ros2 launch hei_robot_navigation navigation.launch.py \
  autostart:=false cmd_vel_topic:=/hei_robot_navigation/cmd_vel
```

激活导航需要连续的 `map -> odom -> body -> base_link`、`/chassis/odom` 速度反馈和点云。

标准接口包括 `/navigate_to_pose`、`/navigate_through_poses`、`/compute_path_to_pose`、
`/follow_path`、`/follow_waypoints`。RViz 使用 Nav2 Goal 工具和 Navigation 2 面板。
本阶段不新增任务点导航业务节点，等待后续需求。

## Nav2 Goal 规划失败与路径显示

单目标导航使用 `config/navigate_with_replanning.xml`：保持 1 Hz 全局重规划，规划或
跟踪失败后清理对应代价地图并重试一次，仍失败就终止目标，不执行自动旋转/倒车恢复。
正常跟踪有效路径、到达目标时对齐朝向仍可能旋转。
`bt_navigator.default_nav_to_pose_bt_xml` 可指定其他 XML；相对路径相对于参数 YAML。
此设置针对 `NavigateToPose`（含 RViz Nav2 Goal）；`NavigateThroughPoses` 仍使用上游默认树。

RViz 的 `Global Path` 订阅 `/plan`，以 6 cm 宽的橙色线显示。只有全局规划成功后才有路径；
`failed to create plan` 时不能靠显示配置生成路径。Nav2 面板的 `Localization: inactive`
主要反映它管理的定位生命周期节点，不能据此判断外部 RTAB-Map；应检查
`/hei_robot_mapping/localization_ready`、连续 TF 和实时点云对齐情况。

以下诊断记录对应改为 30×30 cm 轮廓之前的 60×60 cm 配置。

2026-11-09 日志中的起点约 `(-0.48, 0.14)`、目标约 `(-0.01, 0.19)`。
`lab_01.pgm` 在该起点附近已有占据栅格，最近栅格中心距起点约 0.24 m，小于
60 cm 车身的半宽；该起点不是有效的无碰撞起点。缩小软膨胀带或隐藏代价地图不能
修复这个问题。应先确认实时扫描与静态地图的位置是否一致：若不一致，先修正定位；
若一致且现场确实有障碍，则应更换有足够净空的起点；若地图将机身回波或噪点记录成
障碍，应在核实投影高度、机身过滤和地图质量后重新导出地图，不应未经现场确认擦除障碍。

同日静止复测：RTAB-Map 确认定位，实时点云与原始 PCD 整体吻合。按临时安装偏移算得
底盘中心约 `(-0.489, 0.162)` m，60×60 cm 轮廓加 2 cm padding 内有 7 个占据栅格。
用户确认当前机身周围空旷后，生成独立 `lab_01_nav.pgm` / `lab_01_nav.yaml`，仅将这
7 个栅格置为空闲，原 `lab_01.db` / `.pcd` / `.pgm` / `.yaml` 全部保留。导航 YAML 的
`navigation_correction` 记录源图哈希、修正位置与栅格中心；此修正不会改变三维定位地图。
这是一处经现场确认的局部地图修正，不能替代后续准确测量轮廓、雷达安装高度和排查自体回波。

隔离域软件验证中，原图起点规划失败且没有非零速度；修正版从本次测得的位姿规划成功，
`/plan` 发布 19 个位姿点。回放本次采集的真实点云时，DWB 能生成平移指令，目标随后
取消。测试使用预留速度话题，未驱动真实底盘，不能视为实车路径跟踪和避障验收。

参考：[Nav2 Humble 官方启动框架](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bringup)。
