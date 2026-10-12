# HEI ReBot Lift 双机开发与上游同步工作流

[English](git_workflow.md) | [中文](git_workflow_zh.md)

本文面向持续复现母项目，同时在 Jetson AGX 和笔记本上开发 MID360、ROS 2 Humble
导航及其他功能的场景。默认采用：**一个 Fork、一个自研扩展目录、按功能拆包、通过
功能分支集成、用已验证的版本部署双机**。

这是协作约定，不是“母项目源码禁止修改”的审批制度。优先使用公开接口和外部配置；
接口确实不足时，可以做范围明确、可测试的小补丁。避免为了加一个功能顺手重构母项目。

> 本文中的 `extensions/`、ROS 包、桥接网关和部署记录是推荐的新结构，尚不代表已经实现。
> 本次工作流重写不创建目录、不安装依赖、不调整 Git 分支，也不迁移已有文件。
> 文中命令分阶段执行；含占位符的示例需先替换，不能整篇连续粘贴执行。

## 1. 仓库与分支分别负责什么

| 名称 | 用途 |
| --- | --- |
| `upstream` | 母项目 `https://github.com/lipengdong/hei-rebot-lift.git` |
| `origin` | 自己的 Fork `https://github.com/xiaofo059/hei-rebot-lift.git` |
| `main` | 跟随 `upstream/main`，不混入自己的功能提交 |
| `dev` | 长期集成分支：母项目更新 + 自研功能 + 双机配置和文档 |
| `feat/<topic>` | 从 `dev` 建立的短期功能分支，例如 `feat/mid360-bringup` |
| `fix/<topic>` | 缺陷修复，例如 `fix/base-command-timeout` |
| `sync/upstream-<date>` | 将新版 `main` 集成进 `dev` 前的验证分支 |
| `robot-tested/<date>-<n>` 标签 | 指向完成约定验证的 Fork 提交，用于部署、复现和回退 |

```text
upstream/main → origin/main → sync/upstream-日期 → dev → 已验证标签
                                                ↑
                                      feat/*、fix/*
```

`dev` 承载新功能；功能分支合并后可删除，模块源文件继续留在 `dev`。不需要为每个模块
维护永久分支，也不需要为 Jetson 和笔记本建立两套长期代码分支。小型文档修正可以直接
提交到 `dev`；硬件接口、导航和上游升级优先走独立分支。

共享的 `main`、`dev`、已交接的功能分支采用 merge 和正常 push，避免重写他人依赖的历史。
尚未分享的个人功能分支可以自行整理提交。

## 2. 新功能放哪里

**默认放在本仓库根目录的 `extensions/` 下，并由当前 Fork 统一管理。**
这样一份提交可以同时记录母项目基线、自研代码、参数模板和部署说明；通过目录边界
降低合并冲突，而不是把新功能散落在未纳入版本管理的文件夹里。

```text
hei-rebot-lift/
├── software/lerobot-hei-rebot-lift/  # 母项目软件，尽量小改
├── hardware/、media/、docs/          # 原有资料与项目文档
└── extensions/                     # 自研内容的统一入口，按需创建
    ├── README.md                   # 模块索引、启动顺序、运行位置、成熟度
    ├── ros2_ws/                    # 自研 ROS 2 overlay 工作空间
    │   ├── src/
    │   │   ├── hei_robot_interfaces/ # 必要的自定义 msg/srv/action
    │   │   ├── hei_robot_description/# 传感器安装 TF、模型扩展
    │   │   ├── hei_robot_bridge/     # ROS ↔ 网关，单位转换和状态映射
    │   │   ├── hei_robot_localization/ # 定位算法的集成配置
    │   │   ├── hei_robot_navigation/ # Nav2 参数、行为树、导航 launch
    │   │   ├── hei_robot_bringup/    # MID360/整机启动与参数装配
    │   │   └── hei_robot_sim/        # 仿真适配，与真机共用接口
    │   └── build/、install/、log/    # 各机器本地生成，不提交
    ├── python/hei_motion_gateway/   # LeRobot 环境中的独立 Python 包
    ├── config/
    │   ├── common/                 # 可共享的默认配置
    │   ├── jetson/                 # 机器人端模板
    │   ├── laptop/                 # 仿真/可视化端模板
    │   └── local/                  # 本机覆盖，不提交
    ├── dependencies/               # 第三方 .repos 锁定清单、SDK 构建说明
    ├── third_party/                # 按清单获取的外部源码，不提交至父仓库
    │   └── livox_ws/src/livox_ros_driver2/
    ├── scripts/                    # 可重复的环境检查、构建、启动脚本
    ├── tests/                      # 跨模块、协议和集成测试
    └── docs/
        ├── decisions/              # 接口选型、上游补丁缘由
        └── deployments/            # 双机版本及验证记录
```

不必一次创建所有空包。从 `hei_robot_bringup` 的雷达配置开始，接着实现桥接，再接入
定位和导航。普通 Python 工具放在 `extensions/python/`，不为了目录统一强行包装成 ROS 节点。

首次创建 ROS 包的示例：先进入功能分支，并准备好 ROS 2 Humble；从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
mkdir -p extensions/ros2_ws/src
cd extensions/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 hei_robot_bringup
mkdir -p hei_robot_bringup/launch hei_robot_bringup/config
```

这只生成包骨架。添加实际 launch/配置后，还需在 `CMakeLists.txt` 安装这些目录，
在 `package.xml` 声明运行依赖，并补充 README；目录存在不等于雷达功能已完成。

仓库历史中的 `software/ros_ws/` 是另一种位置方案。若已有代码，先盘点再有计划地迁移；
不要同时长期维护两套自研 ROS 工作空间。旧目录的迁移或删除应作为单独、可审阅的变更，
不要混入母项目同步。

### 模块多了怎么管理

- 一个包对应清晰职责；包间依赖写入 `package.xml`，Python 依赖写入 `pyproject.toml`。
- 默认一个自研 ROS 工作空间容纳多个包；只有依赖冲突、独立发布或构建方式特殊时才拆工作空间。
- 模块 README 记录用途、运行机器、输入输出、参数、启动命令和最小验证方法。
- `extensions/README.md` 用表格维护模块状态：计划、实验、已验证，以及依赖关系。
- 优先标准 ROS 消息；确有需要再增加 `hei_robot_interfaces`，避免重复定义相同数据。
- 只有模块需要跨项目复用、独立权限或独立发布时再拆成独立仓库，并在 `.repos` 中锁定提交。
  自研包默认不要嵌套 `git init`。

## 3. 双机代码、环境与配置

| 项目 | Jetson AGX | 笔记本 |
| --- | --- | --- |
| 源码 | 克隆同一个 Fork | 克隆同一个 Fork |
| 常驻职责 | host、电机/相机、MID360、导航执行 | 仿真、RViz、数据分析、训练和开发 |
| 联调角色 | 硬件与最终控制执行 | 可运行实验算法或遥操作客户端 |
| ROS 构建 | 本机 ARM64 构建 | 本机 x86_64 构建（以实际架构为准） |
| 环境 | LeRobot + ROS；无需为 host 安装 VR 环境 | LeRobot、VR/IK、ROS，按任务安装 |

两端目录名可以相同，绝对路径不必相同。下文 Git 命令默认在各自仓库根目录执行。
不通过复制整个工作目录同步源码，不跨机器复制 `build/`、`install/`、Conda 环境或缓存。

当前母项目 LeRobot 使用 Python 3.12，VR/IK 环境使用 Python 3.10。
ROS 2 Humble 的 Ubuntu 22.04 二进制安装支持 amd64/arm64，使用系统 Python 3.10。
先确认 Jetson 的具体型号、JetPack/L4T 和 Ubuntu 版本，不能从“AGX”推断其系统版本；
若不是兼容系统，再单独选择受支持的系统方案或验证容器部署。[ROS 官方安装说明](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/Ubuntu-Install-Debs.rst)

ROS、LeRobot、VR 使用不同终端/进程。运行 ROS 时不激活 LeRobot Conda 环境，
不通过把 `rclpy` 塞进 Python 3.12 环境来连接两者。跨环境通过显式 IPC 协议通信。

配置按“共享默认值 → 机器角色 → 本机覆盖”加载；这是新启动器需要实现的约定，
不是现有母项目已经具备的能力。共享模板提交 Git；IP、网卡、串口映射、雷达序列号等
本机值放 `config/local/`，在部署记录中注明配置版本或校验值。必要的标定资料需有备份，
不能因为不提交 Git 就只保留在一台设备上。修改已跟踪的 udev 文件仍会产生 Git 差异。

双机 ROS 联调时使用相同的 `ROS_DOMAIN_ID`、兼容的 RMW/QoS、可互通的网络与同步时钟。
雷达网段/IP、DDS 发现和 HEI 的 ZMQ 地址分别配置；设置 Domain ID 不代表网络已经打通。
真机使用实际时钟；仿真/rosbag 回放统一 `use_sim_time` 和 `/clock`。

## 4. MID360 → 定位 → 导航如何接入

以下是待实现的扩展架构，按阶段验收，不把安装雷达驱动等同于导航完成：

```text
MID360 → Livox ROS 2 驱动 → 点云/IMU → 定位/建图 → Nav2
                                              ↓ 速度请求
ROS 桥接 → 动作网关（LeRobot 环境）→ 现有 host → 底盘
                         ↑
                 观测、控制权与停止逻辑
```

### 第一阶段：雷达和坐标系

在 `hei_robot_bringup` 管理自有 launch 和参数，在 `hei_robot_description` 管理传感器
外参。先验证雷达网络、点云/IMU 频率、时间戳、坐标轴、丢包和 RViz 显示，再录短 rosbag。

MID360 使用官方 `livox_ros_driver2` 和其要求的 Livox-SDK2。驱动版本与 SDK 版本一起记录。
官方 `build.sh humble` 会清理所在工作空间的构建/安装目录，且对源码布局有要求，
因此将驱动放在独立的 `extensions/third_party/livox_ws/src/livox_ros_driver2/`，
按锁定版本的说明构建，然后作为自研工作空间的 underlay。
不要在装满自研包的工作空间里直接执行该脚本。
[驱动说明](https://github.com/Livox-SDK/livox_ros_driver2) · [构建脚本](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/build.sh)

### 第二阶段：桥接和可验证的底盘控制

`hei_robot_bridge` 接受 ROS 速度请求，独立网关在 LeRobot 环境中使用现有客户端连接 host。
先定义并测试 IPC 的消息版本、时间戳/有效期、速度单位、状态、控制模式和错误处理，再上机。

当前源码有三个必须处理的接口事实：

1. `HeiRebotLift.send_action()` 对缺失的机械臂关节字段默认填 `0.0`。
   **不能只把 `cmd_vel` 转成三个底盘字段后发给 host。** 网关应在取得新鲜反馈并明确接管后，
   锁存需保持的双臂/夹爪/升降目标，组装完整 18 维动作；反馈缺失时不解锁、不填零继续运行。
2. 底盘 `x.vel/y.vel/theta.vel` 使用驱动缩放参数，不能直接当作 ROS 的 m/s、rad/s。
   桥接需依据轮径、几何、符号和实测标定完成换算；反馈也不能未经换算就当 `/odom`。
3. host 观测通道使用 ZMQ PUSH/PULL，不是广播。多个客户端会分摊消息。
   导航模式下由网关独占 host 客户端连接，再向 ROS 发布状态；不要另起客户端“旁听”。

初版采用显式互斥模式：导航运行前停止原遥操作/录制/rollout 客户端；恢复原工作流前停止
导航网关。需要同时导航和操作双臂时，再将各请求源接入同一仲裁层，由唯一发送者组装动作。
上游 host 当前没有这套多源仲裁能力。

接口依据：[动作组装与底盘换算源码](../software/lerobot-hei-rebot-lift/src/lerobot/robots/hei_rebot_lift/hei_rebot_lift.py)、
[host 通信与看门狗源码](../software/lerobot-hei-rebot-lift/src/lerobot/robots/hei_rebot_lift/hei_rebot_lift_host.py)。上游升级后重新核对。

桥接必须拒绝过期请求；导航停止、输入超时、模式切换和网络中断时撤销速度请求并停止底盘。
网关不能持续转发旧速度，从而让 host 的看门狗失效。现有 host 看门狗停止底盘和升降，
并不构成完整的机械臂急停方案。

### 第三阶段：定位与导航

先建立稳定的里程计和 TF 链 `map → odom → base_link → lidar`，为每条 TF 指定唯一发布者。
在记录的数据和仿真上选择/验证支持 Humble、MID360 数据类型的定位方案，再联调真机。
3D 点云进入导航前明确采用投影 LaserScan 还是点云障碍层，并配置高度过滤、机器人轮廓
和传感器盲区；定位和避障是两个不同的消费者。

Nav2 的局部控制器及速度限制需匹配全向底盘，验证侧向速度支持，不能直接套用差速车参数。
验证顺序为：只读数据 → 假 host/仿真桥接 → 架空轮组方向与停止 → 低速定位 → 小范围导航。
这些是扩展的验收目标，不是声明当前仓库已有完整导航栈。
TF 职责参考 [Nav2 状态估计说明](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)；
该链接是 rolling 文档，仅用于架构参考，具体插件和参数需按安装的 Humble 版本核对。

## 5. 第三方依赖、构建与生成文件

自研源码跟随 Fork；第三方源码通过 `extensions/dependencies/*.repos` 记录 URL 和
精确提交 SHA，导入被忽略的 `extensions/third_party/`。部署不只记录会移动的 `main/master`。
[vcstool 官方说明](https://github.com/dirk-thomas/vcstool)

例如 `livox.repos` 的结构如下；先将占位符替换为实际验证过的 SHA：

```yaml
repositories:
  livox_ws/src/livox_ros_driver2:
    type: git
    url: https://github.com/Livox-SDK/livox_ros_driver2.git
    version: REPLACE_WITH_VERIFIED_COMMIT_SHA
```

安装 vcstool 后，从仓库根目录导入；Livox-SDK2 可用同样方式锁定，单独记录其构建与安装步骤：

```bash
mkdir -p extensions/third_party
vcs validate < extensions/dependencies/livox.repos
vcs import extensions/third_party < extensions/dependencies/livox.repos
```

`.repos` 只锁定源码，不安装 SDK、系统库或 Python 依赖。修改外部驱动时，应提交到自己的
驱动 Fork 并更新锁定清单，不把修改只留在被忽略的目录中。更新依赖用独立功能分支验证。

创建工作空间时，在**根 `.gitignore`** 增加以下规则；它们是待执行配置，不是本次已生效的更改：

```gitignore
/extensions/ros2_ws/build/
/extensions/ros2_ws/install/
/extensions/ros2_ws/log/
/extensions/third_party/
/extensions/config/local/
```

源码、`package.xml`、launch、参数模板、依赖清单与测试提交 Git。rosbag、数据集、权重、
大地图放仓库外的数据目录（例如 `~/hei-data/`），在 Git 中记录用途、版本、校验值和获取位置。
小型可复用测试地图可提交或使用 LFS。不要把运行数据与源代码通过同一次 `git add .` 混入。

ROS 构建示例：前提是 Humble、colcon、rosdep 已安装，rosdep 已初始化并更新，
所需第三方 underlay 已按各自文档完成构建。新开未激活 Conda、未 source 自研 overlay 的终端：

```bash
# 从本机仓库根目录开始；无 Livox 依赖的早期模块可跳过其 source 行
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
cd extensions/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

只构建某模块及其工作空间依赖时可用 `colcon build --symlink-install --packages-up-to hei_robot_navigation`。
在新终端按 ROS → 第三方 underlay → 自研 overlay 的顺序 source，再执行模块 README 中的 launch。
不要在已经 source 旧 overlay 的终端反复构建。[ROS 官方工作空间教程](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.rst)

## 6. Git 初始化与日常功能开发

两台机器都指向相同的 origin/upstream；已有克隆无需重新 clone。先查看实际状态：

```bash
git status --short --branch
git remote -v
git branch -vv
```

仅在缺少 upstream 时执行 `git remote add upstream https://github.com/lipengdong/hei-rebot-lift.git`。
推荐每个克隆设置：

```bash
git config pull.ff only
git config pull.rebase false
git config fetch.prune true
git config push.default simple
```

以下操作从干净工作区开始。已有修改先审阅并提交到所属分支，或明确暂存；不为同步自动
丢弃修改，也不用 `reset --hard` 解决分叉。

以雷达功能为例：

```bash
git fetch origin --prune
git switch dev
git merge --ff-only origin/dev
git switch -c feat/mid360-bringup
# 按需创建 extensions/ 下的功能文件，完成该阶段验证
# 仅暂存本次功能的明确路径；同时检查未跟踪文件和 diff
# 示例（文件实际存在后）：git add extensions/ros2_ws/src/hei_robot_bringup
git diff --cached
# 暂存内容确认后：
git commit -m "feat: add MID360 bringup configuration"
git push -u origin feat/mid360-bringup
```

优先向自己 Fork 的 `dev` 提 PR，记录行为变化、两端影响和验证结果。个人开发也可以本地
merge 后正常 push；若远端已前进，先 fetch/merge 并重测，不强推覆盖。

### 在另一台机器继续同一功能

功能分支是交接单位。先在当前机器提交、push，再在另一台机器获取：

```bash
git fetch origin
# 仅首次建立本地分支：
git switch --track origin/feat/mid360-bringup
# 以后改用 git switch feat/mid360-bringup，再执行：
git pull --ff-only
```

尽量避免两台机器同时修改同一功能分支；可以分成 `feat/mid360-driver` 和 `feat/mid360-rviz`。
若已出现分叉，检查 `git log --left-right --oneline HEAD...@{u}`，在一个集成位置 merge、
解决冲突并验证，再让另一端快进。远程 SSH 开发 Jetson 文件也可以，但构建/运行使用 Jetson 环境。

## 7. 跟随母项目更新

上游更新不必立即部署。先在笔记本或独立检出中阅读变更，重点检查 host/client 协议、
Python 依赖、相机字段、控制参数与启动行为。只选择一台机器负责一次集成，其余从 origin 获取结果。

先更新纯上游分支，以下命令要求本地已有 `main`、`dev`：

```bash
git fetch origin --prune
git fetch upstream --prune
git switch main
git merge --ff-only origin/main
git merge --ff-only upstream/main
git push origin main
```

任一 `--ff-only` 失败就停止该流程，检查分叉来源。不要把自己的功能 merge 到 `main`，
也不要为了“镜像”远端未经检查强推。正常情况下，本地 `main` 与本次获取的 `upstream/main` 一致。

接着在临时集成分支合入 `main`（每次使用新的分支名）：

```bash
git switch dev
git merge --ff-only origin/dev
git switch -c sync/upstream-20261012
git merge --no-ff main
# 有冲突：逐文件处理，git add 明确路径，再 git merge --continue
# 放弃本次未完成合并：git merge --abort
# 完成协议、模块、仿真检查后再发布集成分支
git push -u origin sync/upstream-20261012
```

通过面向 `dev` 的 PR 合并；本地完成时先更新 `dev`，再 `git merge --no-ff sync/upstream-20261012`，
验证最终合并结果并 `git push origin dev`。若验证期间 `dev` 有新提交，先合入集成分支并重新检查。
Jetson 等待候选版本通过离线检查后再联调，不在运行中的控制进程下面切分支或拉取代码。

### 确实需要改母项目源码时

先判断能否用自有配置、适配器或明确扩展接口解决。确需改动时，用功能分支制作小而独立的
提交，记录修改文件、原因、依赖的上游行为和回归测试。上游升级时检查补丁是否仍必要，
已被上游吸收就去掉冗余实现。可将通用修复单独整理后反馈给母项目。

HEI 内置 LeRobot 的同步边界见 [UPSTREAM.md](../software/lerobot-hei-rebot-lift/UPSTREAM.md)。
默认跟随 HEI 作者完成的 LeRobot 集成，不额外把 Hugging Face LeRobot 的 `main` 直接混入本仓库。

## 8. 验证、部署与回退

独立开发期间，两台机器不必始终停在同一提交。**双机联调和可复现实验默认使用同一 Fork
提交或标签**；不同提交只在明确记录协议兼容性时使用。仅有相同母项目版本不足以证明自研桥接兼容。

合入前按影响范围验证：文档检查链接与命令；纯算法跑单元测试；桥接验证完整动作、单位、
超时和控制权；硬件相关改动在离线/仿真通过后低速实测。ROS 使用 colcon 测试，
LeRobot 改动运行相关 pytest 和项目格式检查，不因一个文档改动启动整机。

部署记录放 `extensions/docs/deployments/`，至少包括：

- 母项目基线 SHA、Fork 候选 SHA，以及随后创建的已验证标签。
- 两端 OS、架构、JetPack（Jetson）、Python/ROS、关键依赖及第三方锁定清单版本。
- 配置/标定版本、控制模式、模型/地图/数据版本。
- 验证步骤、结果、未验证项，以及上一可用标签和回退配置。

记录先引用被测试的候选 SHA，避免在文件中填写“包含自身的最终提交 SHA”。验证通过后
提交记录，再对包含该记录的提交打标签；若同时改了代码，需要重新验证。标签名按实际日期调整：

```bash
git tag -a robot-tested/20261012-01 -m "Validated dual-machine baseline; see deployment record"
git push origin robot-tested/20261012-01
```

两端停止相关进程、确认工作区干净后部署同一标签：

```bash
git fetch origin --tags
git switch --detach robot-tested/20261012-01
# 按部署记录恢复依赖和配置；各自在本机重建 ROS，更新对应 Python 环境
# 检查 host/client、反馈、超时和停止行为后，再启用实际任务
```

这会进入 detached HEAD，适合固定版本运行。要继续开发，切回功能分支；不要把现场修复
留成无分支的提交。紧急修复可从标签新建 `fix/*`，验证后合回 `dev`。
回退同样停止进程、切到上一个已验证标签，并恢复对应依赖、构建产物和配置。
**仅切 Git 提交不会回退 SDK、Conda、系统包或标定文件。** 有部署需求时可使用独立 worktree，
避免日常开发切换影响运行检出，但仍需为可编辑 Python 安装及 ROS 构建使用明确路径。

## 9. 实施顺序

1. 保留现有母项目运行方式，在一个功能分支创建最小 `extensions/` 结构和忽略规则。
2. 两端确认系统/环境，完成 MID360 只读数据与 rosbag 验证，锁定驱动和 SDK。
3. 实现并测试桥接/网关协议、完整动作保持、单位换算和控制权切换。
4. 增加定位，再增加 Nav2；让仿真与真机复用同一套 ROS 接口和参数结构。
5. 每个阶段合入 `dev`，记录可重复的验证方法，达到部署条件后才创建已验证标签。

后续模块沿用这套目录、依赖清单和分支流程扩展；无需每加一个传感器就重新设计仓库结构。
