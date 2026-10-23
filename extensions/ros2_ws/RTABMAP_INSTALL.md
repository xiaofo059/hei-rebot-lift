# AGX RTAB-Map / ROS 2 Humble

2026-10-23 部署，AGX ARM64 原生编译。

源码位于 `src/rtabmap`（核心库）与 `src/rtabmap_ros`（ROS 2 包），
均固定官方 `0.23.13-humble` 标签，完整提交见 `rtabmap.repos`。
核心库使用稀疏检出 corelib/utilite/guilib/tools/app/cmake_modules、data/presets 和根文件，
不下载大型示例数据；BUILD_EXAMPLES、BUILD_TESTING、BUILD_PERF_TESTS 关闭。

核心功能启用 Qt、g2o、OctoMap；使用 PCL ICP 后端。
未安装可选 GTSAM、libpointmatcher，不影响基础建图/定位构建。
ROS 包构建选择：rtabmap_msgs、rtabmap_conversions、rtabmap_sync、rtabmap_util、
rtabmap_odom、rtabmap_slam、rtabmap_viz、rtabmap_rviz_plugins、rtabmap_python、rtabmap_launch。
不构建机器人/传感器演示、Nav2 costmap 扩展及依赖它们的汇总包 rtabmap_ros。

## 本机 OpenCV 兼容处理

系统 NVIDIA libopencv-dev 4.8 的 CMake 配置引用缺失的 4.8 运行库；
系统实际存在 Ubuntu 4.5.4 运行库，ROS cv_bridge 也链接 4.5。
为避免更改系统 OpenCV，在 `extensions/third_party/opencv_4_5/` 下载并解包
Ubuntu Jammy ARM64 的 OpenCV 4.5.4 开发 deb，存放于 prefix，不使用 sudo 安装。
其中运行库链接指向系统 4.5.4；额外缺失的 videostab/superres 运行库也解包在 prefix。
CMake 必须显式指定此 OpenCV_DIR，不能删除该目录。
系统包未做降级、替换或软件源修改。

## 重建

在干净 Bash 环境中执行：

```bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export CMAKE_BUILD_PARALLEL_LEVEL=6
export MAKEFLAGS=-j6
source /opt/ros/humble/setup.bash
cd /home/user/hei-rebot-lift/extensions/ros2_ws
colcon build --packages-select rtabmap --parallel-workers 1 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF -DWITH_QT=ON -DWITH_G2O=ON -DWITH_OCTOMAP=ON \
  -DOpenCV_DIR=/home/user/hei-rebot-lift/extensions/third_party/opencv_4_5/prefix/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
colcon build --packages-select \
  rtabmap_msgs rtabmap_conversions rtabmap_sync rtabmap_util rtabmap_odom \
  rtabmap_slam rtabmap_viz rtabmap_rviz_plugins rtabmap_python rtabmap_launch \
  --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DOpenCV_DIR=/home/user/hei-rebot-lift/extensions/third_party/opencv_4_5/prefix/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

当前任务仅源码部署与编译，不启动建图、定位、雷达或机器人控制。
FAST-LIO2 → RTAB-Map 的输入适配和 TF/参数配置属于后续集成工作。

## 编译与检查结果

RTAB-Map 核心库及选定的 10 个 ROS 2 包全部编译成功。
核心库属于普通 CMake 项目；ROS 的入口包名为 rtabmap_slam、rtabmap_launch 等，
不是核心库目录名 rtabmap。

验证通过：10 个 ROS 包均来自本工作空间；rtabmap_msgs 类型支持可加载；
SLAM、GUI、ICP odometry、point_cloud_assembler 为 ARM64 可执行文件，动态库无缺失；
官方 rtabmap.launch.py --show-args 能加载；rtabmap_slam/rtabmap --params 能列出默认参数。
OpenCV 使用 4.5，与 ROS cv_bridge 一致。未运行算法测试、建图或定位，
未创建地图数据库或连接机器人控制接口。部署与编译通过不等于 SLAM 实机验收。

日志：
- log/rtabmap-core-build.log
- log/rtabmap-ros-build.log
- log/rtabmap-core-ldd.log
- log/rtabmap-install-verification.log
- log/rtabmap-launch-args.log
- log/rtabmap-default-params.log

使用前加载：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 pkg prefix rtabmap_slam
```
