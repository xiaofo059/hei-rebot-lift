# AGX MID-360S / FAST-LIO2 / RTAB-Map 源码快照

第三方 Git 仓库及构建产物保留在 AGX 本机，不作为嵌套 Git 链接提交到父仓库。
本目录记录固定提交、修改后的本机配置/launch、ROS 2 package.xml 和驱动安装补丁。
FAST-LIO2 的 ikd-Tree 由上游 gitlink 固定为 e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4。

恢复源码及本机修改（需 Git、Python3、PyYAML 和网络）：

```bash
/usr/bin/python3 extensions/dependencies/agx_slam/restore_sources.py
```

脚本不重置已有不同版本，不覆盖已有不同配置；如报告差异，先人工核对。
外参为厂家标称值核对，不是逐台标定。此脚本不构建，也不启动节点。
配置及 launch 的版本管理来源是本目录 overlays；后续编辑本机源码副本后，
提交前应同步对应 overlay 或补丁，以保持可复现。

构建和 OpenCV 兼容处理见：
- ../../ros2_ws/LIVOX_DRIVER_INSTALL.md
- ../../ros2_ws/FASTLIO_INSTALL.md
- ../../ros2_ws/RTABMAP_INSTALL.md

本快照记录源码部署和编译结果，不代表 RTAB-Map 建图/定位实机验收。

本机 OpenCV 开发文件可通过以下命令重新解包（不使用 sudo，不替换系统包）：

```bash
bash extensions/dependencies/agx_slam/prepare_opencv_4_5.sh
```
