# AGX 本机三维建图与重定位部署流程

## 1. 部署目标

在 Jetson AGX Orin（Ubuntu 22.04 / ARM64 / ROS 2 Humble）本机运行正式算法链路：

```text
MID-360S → Livox-SDK2 / livox_ros_driver2 → FAST-LIO2
  → XYZI 点云适配 → RTAB-Map → 地图数据库 / 本机 RViz
```

仅保留驱动、FAST-LIO2、必要的 XYZI 适配和 RTAB-Map。适配器负责剔除前端未估计的法线字段，属于现有算法接口的一部分。
默认关闭探针、定位事件采集和 rosbag 录制；不添加持续读取算法输出的监测节点。
RViz 按需在 AGX 桌面打开。暂不开展电脑—AGX ROS 分布通信，也不接入机器人运动控制或 Nav2。

## 2. 一次性环境部署

1. 核对 `/home/user/hei-rebot-lift` 的代码版本、系统 ROS Humble、系统 Python 和 `/usr/bin/cmake`，使用独立于 Conda 的 ROS shell。
2. 按 `extensions/dependencies/*.repos` 锁定的版本获取 Livox-SDK2、livox_ros_driver2、FAST-LIO2 和 ikd-Tree，应用仓库中对应补丁，在 AGX 本机构建。
3. 建立 ARM64 RTAB-Map 部署方案：保持已验证的 0.23.13 基线，核验 libpointmatcher 点面 ICP、GTSAM 及 ROS 消息依赖。当前 `install_rtabmap_laptop.sh` 只适用 amd64；当前分支尚无可直接使用的 AGX 安装脚本。
4. 将第三方依赖放在 `extensions/third_party/` 下，分别构建 Livox、FAST-LIO underlay，最后构建 `extensions/ros2_ws` 中三个自研包。x86 的二进制和构建目录不用于 AGX。
5. 记录最终版本和环境加载顺序：ROS Humble → Livox underlay → FAST-LIO underlay → ARM64 RTAB-Map 环境 → 自研 ROS overlay。RTAB-Map 的实际安装路径确定后，统一成一个环境入口。

部署门槛：目标可执行文件为 ARM64，动态库无缺失，ROS 包及消息可加载。只做必要的一次性检查，不部署常驻诊断服务。

## 3. 本机配置

- 工作目录：`/home/user/hei-rebot-lift`。
- 雷达配置：`extensions/config/local/mid360s_config.json`。2026-10-22 检查值为 USB 网口 `enx9c69d3cd61a9`、AGX `192.168.1.5`、雷达 `192.168.1.135`；启动前核对实际接线与路由。
- 保持 `ROS_DOMAIN_ID=89`、`ROS_LOCALHOST_ONLY=1`，满足当前启动器约束。
- 每次明确指定地图输出目录，避免沿用代码中电脑端 `/home/robo/` 默认路径。
- 初期沿用已验证的参数及厂商名义外参，使用 `uncalibrated:=true`；安装外参测量另行进行。
- TF 保持 `map → camera_init → body`，由 RTAB-Map 和 FAST-LIO 分别发布，禁止重复发布。

## 4. 实时建图 → 保存 → 重定位

以下命令在一次性环境部署完成、已按上述顺序加载环境后，于 AGX 本机终端执行。
`rviz:=true` 需要 AGX 桌面图形会话；无图形会话时设为 `false`。

```bash
cd /home/user/hei-rebot-lift
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
config_path="$PWD/extensions/config/local/mid360s_config.json"
map_root="$PWD/extensions/ros2_ws/src/hei_robot_slam/map"
map_session="agx_mapping_$(date +%Y%m%d_%H%M%S)"

ros2 launch hei_robot_slam slam_live_mapping.launch.py \
  config_path:="$config_path" output_root:="$map_root" session:="$map_session" \
  allow_live_hardware:=true uncalibrated:=true probe:=false rviz:=true
```

完成采集后按一次 Ctrl+C，等待全部节点正常退出。地图数据库位于：
`$map_root/$map_session/rtabmap_mapping.db`。每次建图使用新 session，保留正常 ROS 日志及启动配置记录。

在同一终端继续执行以下命令；若换终端，先重新加载环境并填写刚才的地图路径：

```bash
database_path="$map_root/$map_session/rtabmap_mapping.db"
test -s "$database_path" || { echo "地图数据库不存在"; exit 1; }
sha256sum "$database_path"

ros2 launch hei_robot_slam slam_live_localization.launch.py \
  config_path:="$config_path" database_path:="$database_path" \
  output_root:="$map_root" session:="agx_localization_$(date +%Y%m%d_%H%M%S)" \
  allow_live_hardware:=true uncalibrated:=true probe:=false rviz:=true
```

定位时在 AGX 本机 RViz 使用 `2D Pose Estimate` 提供地图中的大致位置和朝向，确认工具发布 `/initialpose`。
定位视图显示保存地图与当前扫描，隐藏历史建图轨迹。定位结束后再次检查数据库 SHA256，应保持一致。

## 5. 最小验收与后续边界

依次确认：雷达持续供数 → FAST-LIO 静止无持续明显漂移 → 实时建图正常保存 → 重启加载地图 → 粗初值后扫描对齐并随移动持续跟踪。
通过现有节点日志核对后端定位接受信息，并检查正常退出和数据库完整性；不把单张 RViz 截图当作精度证明。
无初值、任意位置及大角度自动重定位不属于首版保证能力。

出现问题时先检查现有日志、配置和对应模块，必要时短时执行针对性诊断命令；录包或增加探针另行决定，不默认常驻。
AGX 本机建图和重定位跑通后，再安排电脑端可视化通信、底盘 TF、速度桥接与 Nav2。

状态：本文是部署流程，ARM64 环境搭建及 AGX 全链路运行尚未因此完成。
