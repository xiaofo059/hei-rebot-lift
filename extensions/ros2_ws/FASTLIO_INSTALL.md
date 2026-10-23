# AGX FAST-LIO2 / ROS 2 Humble

2026-10-23 安装。源码：`src/fast_lio`。

- 上游：https://github.com/hku-mars/FAST_LIO ，分支 `ROS2`
- 固定提交：`a4743b095409588842a5b30ddfa27e29d2f99164`
- ikd-Tree 子模块：`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- AGX 原生 ARM64 构建，使用系统 CMake / Python 3.10。
- 已安装系统依赖 `ros-humble-pcl-ros`（用户执行 sudo 安装）。
- 主仓库使用稀疏检出，包含构建所需根文件及 include/src/config/launch/msg/rviz，跳过大型演示文档。
- ikd-Tree 使用同版本本机 Git bundle 经 SSH 传输，origin 恢复为官方地址。
- 未修改 FAST-LIO2 算法源码和上游参数，未运行里程计节点。

## 重建

```bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export CMAKE_BUILD_PARALLEL_LEVEL=3
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/local_setup.bash
cd /home/user/hei-rebot-lift/extensions/ros2_ws
colcon build --packages-select fast_lio --parallel-workers 1 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
```

## 使用边界

加载 `install/setup.bash` 后包名为 `fast_lio`，可执行文件为 `fastlio_mapping`，
上游启动文件为 `mapping.launch.py`，默认配置为 `mid360.yaml`。
该配置订阅 `/livox/lidar`（Livox CustomMsg）和 `/livox/imu`。
因此接入 FAST-LIO2 时雷达驱动应使用 `xfer_format:=1`，RViz 标准 PointCloud2
显示用的 `xfer_format:=0` 不适用于这份默认配置。

当前 mid360.yaml 中的外参是上游 MID-360 默认值；2026-10-23 已核对 MID-360S 官方标称外参相同；
标称外参核对不代表逐台设备标定或实机里程计精度验收。
上游默认启用地图点云及 PCD 保存，后续按照“FAST-LIO2 负责里程计、RTAB-Map
负责建图与定位”的分工单独配置，不在本次编译部署中运行这些功能。

## 编译与离线检查结果

Release 编译成功（1 package finished）；可执行文件为 ARM aarch64。
ROS 2 可发现 fast_lio 包及 fastlio_mapping，mapping.launch.py --show-args
加载通过，可执行文件动态库依赖无缺失。日志保存在 `log/fastlio-build.log`
与 `log/fastlio-install-verification.log`。
未启动 FAST-LIO2 节点，未进行实时里程计性能或精度验收。

## AGX MID-360S 里程计启动文件

新增 `launch/fastlio_mid360s_agx.launch.py` 和 `config/mid360s_agx.yaml`。
初始 LiDAR-IMU 外参沿用上游 mid360.yaml，已核对与 MID-360S 官方标称值一致；未做逐台设备标定。
保留在线外参估计；关闭全局地图发布、PCD 保存，保留里程计内部局部地图。
配准点云使用完整扫描。雷达驱动须单独运行，且使用 xfer_format:=1；
如当前以 xfer_format:=0 运行，先停止旧驱动再切换，不要重复启动。

加载 ROS 和工作空间环境后：

```bash
ros2 launch livox_ros_driver2 mid360s_agx.launch.py xfer_format:=1
# 另一个加载相同环境和 ROS_DOMAIN_ID 的终端：
ros2 launch fast_lio fastlio_mid360s_agx.launch.py
```

输入：/livox/lidar（Livox CustomMsg）、/livox/imu（sensor_msgs/Imu）。
输出：/Odometry、/cloud_registered（camera_init）、/cloud_registered_body（body）、
/path、TF camera_init -> body。此 launch 不启动雷达或 RViz。
RViz 查看 /cloud_registered 时 Fixed Frame 使用 camera_init。

重新编译、launch --show-args、ROS 参数加载及安装副本检查通过。
未启动 FAST-LIO2 节点，未验收实际位姿或配准点云。


## MID-360 / MID-360S 外参核对（2026-10-23）

官方 MID-360S 手册第 22 页、MID-360 手册印刷页第 15 页（PDF 第 17 页）
均给出 IMU 在点云坐标系中的位置 [11.0, 23.29, -44.12] mm，且轴方向一致。
FAST-LIO2 对应 p_imu = R * p_lidar + T，因此 R = I，
T = [-0.011, -0.02329, 0.04412] m。两型号标称几何相同，保留现有外参数值，
在线估计开关也保持原值。此次只更新说明，没有修改数值或重启运行中节点。

来源：
- MID-360S: https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf
- MID-360: https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf
