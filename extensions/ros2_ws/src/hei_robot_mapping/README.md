# HEI robot mapping

AGX 上使用 FAST-LIO2 里程计和 MID-360S 点云进行 RTAB-Map 在线三维建图。
本包负责集成，不修改第三方算法，不包含保存地图或定位模式的自研节点。

## 节点与数据流

```text
FAST-LIO2 /cloud_registered_body (body, PointXYZINormal)
  -> /hei_robot_mapping/cloud_adapter (保留 XYZ/intensity，移除未计算的 normals)
  -> /hei_robot_mapping/scan_cloud (body, PointXYZI)
  -> /rtabmap/rtabmap <- FAST-LIO2 /Odometry (camera_init -> body)
```

适配器保持点坐标、frame_id 和时间戳，剔除非有限 XYZ；支持填充行和大小端输入。
输入必须是 body 坐标系，误接 /cloud_registered（camera_init）会报错而不篡改坐标系。
FAST-LIO2 已去畸变，不再次去畸变；不重复启动里程计或订阅原始 IMU。
RTAB-Map 使用现有 PCL ICP 后端与 g2o；以 2 Hz 处理建图帧。
点云与里程计使用精确时间同步，依赖 FAST-LIO2 当前实现的同一扫描末尾时间戳。

TF 职责：FAST-LIO2 发布 camera_init -> body；RTAB-Map 发布 map -> camera_init。
不发布重复静态 TF，不启动雷达、FAST-LIO2 或 RViz，不涉及底盘控制和 Nav2。

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
确认 /Odometry 和 /cloud_registered_body 正常后，在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=0
ros2 launch hei_robot_mapping mapping.launch.py
```

所有终端须使用相同 ROS_DOMAIN_ID。不能同时运行其他发布 map -> camera_init 的节点。
可通过 cloud_topic、odom_topic、config_file、use_sim_time 参数覆盖相应输入。
默认 frame_id=body，与 FAST-LIO2 当前 TF 契约一致；改坐标系时须同时调整适配器。

## 输出与查看

- /rtabmap/cloud_map：地图点云（PointCloud2，通常需要 RViz 订阅后才组装发布）
- /rtabmap/mapData：地图节点和图数据
- /rtabmap/mapGraph：优化后的位姿图
- /rtabmap/info：建图及回环状态
- TF map -> camera_init：地图对里程计的修正

RViz Fixed Frame 使用 map，PointCloud2 选择 /rtabmap/cloud_map。
查看中间点云时可选 /hei_robot_mapping/scan_cloud，需有相应 TF。

## 本轮存储边界

mapping.yaml 显式 database_path=''，RTAB-Map 使用内存数据库。
退出建图进程后本轮地图不会保留；不会覆盖 ~/.ros/rtabmap.db。
未添加保存、导出、清库命令或自研保存节点，后续再单独实现持久化流程。
长期运行仍会增长内存，应按实际地图规模评估。

## 验证

```bash
colcon test --packages-select hei_robot_mapping --event-handlers console_direct+
colcon test-result --test-result-base build/hei_robot_mapping --verbose
ros2 launch hei_robot_mapping mapping.launch.py --show-args
```

单元测试覆盖法向量移除、时间戳/坐标保持、非有限点过滤、行填充、字节序和非法输入。
本轮只进行编译及离线检查，尚未进行机器人端实际建图与回环验收。

2026-10-26：AGX 原生编译通过，点云转换测试全部通过，launch 参数加载通过；
算法参数已与本机 RTAB-Map 参数清单核对，安装文件一致性与动态依赖检查通过。
验证日志位于 ros2_ws/log/hei-mapping-*.log。未启动建图节点、未修改或重启现有驱动。
