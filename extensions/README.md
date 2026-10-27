# 自研扩展

本目录存放 HEI ReBot Lift 的自研功能，由当前 Fork 统一管理。
开发与上游同步方式见 [双机工作流](../docs/git_workflow_zh.md)。

| 目录 | 用途 | 当前状态 |
| --- | --- | --- |
| [ros2_ws/](ros2_ws/) | AGX ROS 2 Humble 工作空间 | Livox、FAST-LIO2、RTAB-Map 已部署编译 |
| [hei_robot_mapping](ros2_ws/src/hei_robot_mapping/README.md) | FAST-LIO2 与 RTAB-Map 建图、地图保存与加载定位 | 编译及软件地图保存/加载/定位验证通过；实机定位待验收 |

新 ROS 功能默认加入 `ros2_ws/src/`，按职责拆包；不为每个包创建独立 Git 仓库。
其他 Python 工具、配置和第三方依赖按工作流文档按需增加，无需预先创建空模块。
