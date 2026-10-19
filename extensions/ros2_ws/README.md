# ROS 2 功能工作空间

本工作空间统一管理自研 ROS 2 Humble 功能包，供 Jetson AGX 和笔记本共同开发。
2026-10-17：笔记本ROS2 Humble下已有hei_robot_bringup（MID-360S真实数据验收通过）与hei_robot_localization（FAST-LIO2静止包离线接入通过）。机器人桥接、导航和AGX部署尚未验收。

## 目录布局

```text
ros2_ws/
├── README.md
├── src/           # 功能包源码，提交 Git
├── build/         # colcon 生成，不提交
├── install/       # colcon 生成，不提交
└── log/           # colcon 生成，不提交
```

`build/`、`install/`、`log/` 会在构建时生成，已经在仓库根 `.gitignore` 中忽略。
两台机器各自构建，不相互复制这些目录。

## 功能包如何组织

以下按职责列包名；实际已有包以src目录及各包README为准，其余仍为规划：

| 包名 | 职责 |
| --- | --- |
| `hei_robot_bringup` | MID360、整机启动、节点与参数装配 |
| `hei_robot_description` | ROS 机器人模型、传感器外参和 TF 配置 |
| `hei_robot_bridge` | ROS 与现有 LeRobot 控制链路的适配 |
| `hei_robot_localization` | 里程计、定位与建图集成 |
| `hei_robot_navigation` | Nav2 参数、行为树及导航启动 |
| `hei_robot_moveit_config` | 后续 MoveIt 2 规划组、运动学、关节限制与规划配置 |
| `hei_robot_manipulation` | 双臂任务编排、规划调用及执行协调 |
| `hei_robot_control` | 后续 ROS 控制接口与轨迹执行适配；是否采用 ros2_control 另行验证 |
| `hei_robot_interfaces` | 标准消息不能满足需求时的自定义接口 |
| `hei_robot_sim` | 仿真适配，与真机共享 ROS 接口 |

先完成一个功能的最小闭环，再按职责拆包。配置、launch 和测试放在所属包内，
依赖写入 `package.xml`；不要在包间复制驱动或协议实现。
机械臂规划结果需要经过执行适配才能驱动实机，创建 MoveIt 配置不等于已经接通控制。

第三方驱动不直接复制进自研包。Livox 等需要特殊构建流程的依赖按
[双机工作流](../../docs/git_workflow_zh.md)的依赖清单和独立 underlay 方案管理。

## 创建第一个包

先安装并验证 ROS 2 Humble。以下命令从仓库根目录执行，使用未激活 Conda 的 ROS 终端：

```bash
source /opt/ros/humble/setup.bash
cd extensions/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 hei_robot_bringup
mkdir -p hei_robot_bringup/launch hei_robot_bringup/config
```

添加实际文件后，在 `CMakeLists.txt` 配置 launch/config 的安装规则，在 `package.xml`
声明运行依赖，并补充包级 README。纯 Python 节点包可选用 `ament_python`。

## 构建和测试

实际功能包创建后，安装 colcon、初始化并更新 rosdep。新开 ROS 终端，从仓库根目录执行；
如依赖第三方 underlay，先按其文档构建，并在 ROS 之后、自研工作空间之前 source：

```bash
source /opt/ros/humble/setup.bash
cd extensions/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

运行时新开终端，从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
# 如有第三方 underlay，在这里 source 其 local_setup.bash
source extensions/ros2_ws/install/local_setup.bash
```

再按具体包 README 启动节点。不要在已 source 本工作空间的终端反复构建。
目前没有功能包，无需执行构建或启动命令。

## 与现有实机控制的边界

导航、机械臂规划、遥操作共享同一台机器人，需明确控制权及超时停止行为。
现有 host 需要完整动作字段，不能直接发送仅含底盘速度的消息；ROS 单位与驱动单位也需要转换。
桥接、动作保持和多源协调的设计见 [双机工作流](../../docs/git_workflow_zh.md)。
新增功能应通过明确接口接入，避免多个 ROS/LeRobot 进程同时占用电机串口或争抢控制。

2026-10-19：新增 [hei_robot_slam](src/hei_robot_slam/README.md)：FAST-LIO2 + RTAB-Map三维建图/只读定位/严格导出，笔记本桌面包软件基线通过，现场SLAM精度与AGX未验收。详见 [三维SLAM说明](../../docs/MID360S_SLAM.md)。
