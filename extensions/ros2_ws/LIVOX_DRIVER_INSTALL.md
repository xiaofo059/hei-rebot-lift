# AGX MID-360S 官方 ROS 2 驱动

安装日期：2026-10-23。目标：AGX ARM64、ROS 2 Humble。

- 驱动来源：https://github.com/Livox-SDK/livox_ros_driver2
- 驱动提交：21445540f0d100dc86a7e6df312dd70bbdb4afdf
- SDK2 来源：https://github.com/Livox-SDK/Livox-SDK2
- SDK2 提交：c0796f04c143143899c87a773d9f6b7136453c0b
- 驱动源码：`src/livox_ros_driver2`
- SDK2 源码：`../third_party/Livox-SDK2`
- SDK2 安装：`../third_party/install/livox_sdk2`

依赖安装在用户工作空间内，无需 sudo。驱动的 `package.xml` 从官方 `package_ROS2.xml` 复制。
为保留工作空间其他包的构建结果，不直接使用会清理整个 build/install 的官方 build.sh。

## 重新构建

在干净 Bash 环境中执行，使用系统 Python 和 CMake：

```bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
cd /home/user/hei-rebot-lift
cmake -S extensions/third_party/Livox-SDK2 \
  -B extensions/third_party/Livox-SDK2/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/user/hei-rebot-lift/extensions/third_party/install/livox_sdk2
cmake --build extensions/third_party/Livox-SDK2/build -j4
cmake --install extensions/third_party/Livox-SDK2/build
cp extensions/ros2_ws/src/livox_ros_driver2/package_ROS2.xml \
  extensions/ros2_ws/src/livox_ros_driver2/package.xml
source /opt/ros/humble/setup.bash
cd extensions/ros2_ws
colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/user/hei-rebot-lift/extensions/third_party/install/livox_sdk2 \
  -DCMAKE_INSTALL_RPATH=/home/user/hei-rebot-lift/extensions/third_party/install/livox_sdk2/lib \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
```

## 加载与只读检查

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg executables livox_ros_driver2
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

MID-360S 专用配置为 `config/MID360s_config.json`，启动文件为
`launch_ROS2/msg_MID360s_launch.py` 和 `launch_ROS2/rviz_MID360s_launch.py`。
本次保留官方默认网络配置；启动前须按实际雷达和 AGX 网络核对配置。
安装、编译和离线检查不代表雷达数据或 SLAM 实机验收。

## 本次检查结果

ARM64 原生编译成功；ROS 包与可执行文件可发现；CustomMsg 消息和 Python
类型支持加载成功；驱动库与可执行文件的动态依赖无缺失。
SDK2 路径通过 CMAKE_INSTALL_RPATH 写入构建结果。
构建和检查日志保存在本工作空间 `log/livox-*.log`。
未启动驱动、修改网络配置或验证实机点云。

## AGX 本机启动配置

新增 `src/livox_ros_driver2/config/MID360s_agx.json`：雷达 IP
`192.168.1.135`，AGX IP `192.168.1.5`。保留官方 MID-360S 端口与默认外参；
零外参不代表已完成雷达与机器人标定。

已安装自定义 `launch/mid360s_agx.launch.py`，加载上文环境后可使用：

```bash
ros2 launch livox_ros_driver2 mid360s_agx.launch.py
# 若需要标准 PointCloud2：
ros2 launch livox_ros_driver2 mid360s_agx.launch.py xfer_format:=0
```

默认 `/livox/lidar` 使用 Livox CustomMsg、10 Hz；IMU 数据使用
`/livox/imu`（sensor_msgs/Imu）。驱动自动启用 IMU，无须独立 IMU 节点。
保持官方 `livox_frame`，不添加机器人 TF。默认不启动 RViz。

注意：官方 `build.sh` 会覆盖并删除 `launch/`，请使用本文的 colcon 构建方法。
本次已重新编译、检查 JSON/Python 与安装副本，并通过 `--show-args` 验证 launch
可发现和加载；未启动雷达，未验证实际数据频率或网络连通性。
