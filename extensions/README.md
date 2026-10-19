# 自研扩展

本目录存放 HEI ReBot Lift 的自研功能，由当前 Fork 统一管理。
开发与上游同步方式见 [双机工作流](../docs/git_workflow_zh.md)。

| 目录 | 用途 | 当前状态 |
| --- | --- | --- |
| [ros2_ws/](ros2_ws/README.md) | ROS 2 Humble 功能包：底盘导航、机械臂规划控制、传感器和仿真适配 | 笔记本bringup真实数据验收通过；localization完成FAST-LIO离线接入，运动精度/AGX待验证 |

新 ROS 功能默认加入 `ros2_ws/src/`，按职责拆包；不为每个包创建独立 Git 仓库。
其他 Python 工具、配置和第三方依赖按工作流文档按需增加，无需预先创建空模块。

2026-10-17：操作与证据见 [雷达接入](../docs/MID360S_BRINGUP.md)、[FAST-LIO2](../docs/MID360S_LIO.md)。

2026-10-19：新增电脑端 [三维SLAM接入包](ros2_ws/src/hei_robot_slam/README.md)，完整操作及验收边界见 [MID360S_SLAM.md](../docs/MID360S_SLAM.md)；软件基线已验收，现场精度/AGX待验收。
