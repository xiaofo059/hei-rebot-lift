# HEI 四轮全向底盘 ros2_control

ROS 2 Humble，底盘独立控制；不会连接机械臂、升降机、相机或调用整个 LeRobot 主机。

## 结构和接口

- `OmniChassisController`：真正的 `controller_interface::ControllerInterface` 插件。
- `DamiaoChassisSystem`：`hardware_interface::SystemInterface` 插件，原项目同款 U2CAN 串口封装、DM4310 速度模式。
- `/cmd_vel`：`geometry_msgs/msg/Twist`，`linear.x` 前进、`linear.y` 左移，单位 m/s；`angular.z` 逆时针，单位 rad/s。应持续以 10–30Hz 发布；仅有一个最终指令源。
- `/chassis/odom`：`nav_msgs/msg/Odometry`，来自真实电机轮速反馈，支持横向速度。模拟模式是理想轮速反馈，不代表物理仿真。
- `/joint_states`：四轮关节状态。
- `controller_manager` 下加载 `joint_state_broadcaster` 和 `chassis_controller`。

轮序固定为 **RF / RR / LR / LF**，电机 ID **1 / 2 / 3 / 4**，Master ID **0x11 / 0x12 / 0x13 / 0x14**。不能按差速底盘处理。

O 型全向轮的滚动方向在对角线上。设 `d = sqrt(2)*r`、`k = half_length + half_width`：

```text
[w_RF]   1/d [-1 -1 -k] [vx]
[w_RR] =     [-1 +1 -k] [vy]
[w_LR]       [+1 +1 -k] [wz]
[w_LF]       [+1 -1 -k]
```

随后乘各轮 `wheel_signs`。沿用独立键盘测试的方向关系；LeRobot 主配置中的 X/Y 负号用于 VR 输入，不套用到 ROS 标准轴上。

## 编译与模拟检查

```bash
source /opt/ros/humble/setup.bash
cd /home/user/hei-rebot-lift/extensions/ros2_ws
colcon build --packages-select hei_chassis_controller --symlink-install
source install/setup.bash
ros2 launch hei_chassis_controller chassis.launch.py use_mock_hardware:=true
```

**默认 `use_mock_hardware:=false`，直接连接真实底盘并使能四轮。上面的模拟检查命令显式指定 `true`，不打开串口、不使能真实电机。**

另开终端 source 相同环境后：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /chassis/odom
# 以下仅用于已显式指定 use_mock_hardware:=true 的模拟模式；停止发布后 0.3 秒内清零轮指令。
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}'
```

`colcon test --packages-select hei_chassis_controller` 使用单元测试和伪终端电机模拟器，不操作真实串口。

## 实车启动

先结束独立底盘测试、LeRobot 主机和其他占用底盘的程序。首次实车验证应将底盘可靠架空、轮子离地，准备物理急停，并先验证四轮前进/横移/旋转方向及超时停车。软件默认速度限制 0.25m/s、0.5rad/s。

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 launch hei_chassis_controller chassis.launch.py
```

无参数启动默认使用真实底盘，会打开 `/dev/hei_chassis` 并使能四轮；无 `/cmd_vel` 时只发送零速度。`Ctrl+C` 正常退出发送零速并失能四轮。

## 参数

- `config/controllers.yaml`：轮径、轴距/轮距的一半、轮向、SI 速度上限、轮速/轮加速度、指令超时、话题、轮速里程计及 TF 开关。
- `config/hardware.yaml`：串口、波特率、CAN ID、外部传动比、IO 频率、反馈映射范围、反馈超时和电机 CAN 看门狗。
- 启动时可指定 `config_file:=绝对路径`、`hardware_config:=绝对路径`、`port:=设备路径`。
- `publish_robot_state:=true` 可发布附带的底盘简化模型 TF；已有完整机器人模型时保持默认 false，防止重复发布。

轮径 **0.076m** 来自原 STL，轮中心间距约 **0.447168m** 来自原 URDF。它们是模型初值；轮胎变形、装配差异、实际轮距、外部传动比及方向需实测校准，不能把模型验证当成真实距离精度验收。附带 URDF 是几何中心处的底盘示意模型，不替代整机碰撞模型。

## 建图时键盘控制

先按上文启动底盘控制器（默认使用真实底盘），雷达、FAST-LIO2、RTAB-Map 按现有流程分别启动。再另开一个 AGX 交互式终端：

```bash
source /opt/ros/humble/setup.bash
source /home/user/hei-rebot-lift/extensions/ros2_ws/install/setup.bash
ros2 run hei_chassis_controller chassis_keyboard
```

| 按键 | 动作 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 向左 / 向右横移 |
| Q / E | 向左 / 向右旋转 |
| 1 / 2 / 3 | 低 / 中 / 高速，切换时停车，重新按方向键运动 |
| 空格 / K | 停车 |
| X / Ctrl+C | 停车并退出 |

默认低速档：0.05m/s、0.15rad/s。其余档位及发布频率、话题、按键超时在 `config/keyboard.yaml`。节点启动自动读取安装目录内该文件；修改源码目录中的配置后重新编译/加载工作空间（`--symlink-install` 配置软链接生效时无需复制）。也可直接指定源码配置：

```bash
ros2 run hei_chassis_controller chassis_keyboard --ros-args --params-file /home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_chassis_controller/config/keyboard.yaml
```

节点只发布 `/cmd_vel`，不直接连接电机。只启动键盘节点不会启动底盘硬件或建图。请勿同时运行其他 `/cmd_vel` 指令源（如 Nav2 或其他遥控节点）；它不承担多指令源仲裁。

终端不能直接检测松键：从最后一个方向字符起 **0.25 秒**未收到重复按键即发送零速，发布周期还可能增加最多约 0.05 秒。按住按键依赖桌面键盘自动重复；默认首次重复延时较大时，可能出现“先走一下、短暂停车、随后连续走”的现象。可在桌面键盘设置中缩短重复延时，或采用短按点动；不要为连续移动而关闭超时停车。此节点每次仅控制一种运动，不识别多键组合。

空格、未知按键和切换档位立即发送零速；箭头转义序列被忽略并停车，避免把其末尾的 A/D 当成横移命令。X、Ctrl+C、SIGTERM、SIGHUP 和正常终端 EOF 退出前发送零速，并在终端可用时恢复其设置；节点崩溃或 SSH 断开还由底盘自身 0.3 秒指令超时兜底。键盘焦点应留在该终端，切到 RViz 后不会收到新的键盘输入，剩余按键租期到期即停车。远程使用需 `ssh -t user@192.168.0.142` 分配终端；建议先在 AGX 本机验证。

## 停车与故障行为

1. 控制器使用单调时钟判定 `/cmd_vel` 超时，默认 0.3 秒；显式零指令、NaN/Inf 指令和超时直接清零，停车不经过加速度斜坡。重新激活必须收到新指令。
2. 串口 IO 在独立线程，控制循环仅交换固定大小数据；发送和接收有工作量/时限约束。硬件层独立限制电机速度/加速度，控制循环停止更新超过 0.2 秒也清零。
3. 任一轮反馈超过 0.2 秒未更新或状态不是 enabled=1，停止并失能四轮，故障锁存。不会自动清故障/重新使能；查明原因后重新启动。寄存器回复不能冒充新鲜电机反馈。
4. 激活前写入并读回 VEL 模式 `CTRL_MODE=3`、CAN `TIMEOUT=6000`；TIMEOUT 单位是 **50µs**，6000 即 **300ms**。仅写 RAM，不保存 flash、不设编码器零位、不改臂/升降参数。读回 PMAX/VMAX/TMAX 与 YAML 不一致时拒绝激活，避免错误反馈换算。
5. 电机 CAN 看门狗用于主机崩溃/USB 断开后的电机侧保护；真实固件的超时停车效果仍须架空实测。软件不能替代物理急停。进程被 SIGKILL 时不能执行退出清理。

### 启动失败：四轮使能反馈不齐

`Not all four motors returned fresh enabled feedback` 表示四轮未全部返回新鲜的已使能反馈，不等同于串口无法连接。启动过程会报告具体电机 ID、状态码，或区分无状态回复与反馈过期。收到电机故障码时直接拒绝激活，发送零速度并失能四轮，不会自动清除故障或反复使能。

其中 `state=0xd (communication timeout)` 是电机侧通信超时故障；寄存器仍可能正常回复。先退出底盘控制器和其他底盘程序，检查电源、USB/CAN 连接；确认连接恢复后，重新上电底盘清除锁存故障，再启动控制器。不要通过放宽反馈超时或取消四轮使能检查绕过故障。正常使用 `Ctrl+C` 退出，避免强制杀进程导致来不及发送失能指令。反复出现时需进一步检查通信链路和主机调度，重新上电恢复不代表已确定最初的触发原因。

## 与当前 SLAM / Nav2 的关系

当前 FAST-LIO2 + RTAB-Map 保留 `map → odom → body`。本包默认 **`publish_odom_tf: false`**，只发布独立 `wheel_odom` 数值里程计，避免两个来源同时决定 `odom → body/base_link`。不要把独立轮速积分直接改名成 FAST-LIO2 的 `odom` 而不建立它们之间的关系。

接入 Nav2 时，由已测量的雷达/IMU 安装外参提供 `body → base_link` 静态连接；Nav2 的 `robot_base_frame` 和 footprint 使用真实底盘参考点、尺寸。若以后进行轮速/IMU/FAST-LIO 融合，应先明确唯一的 odom TF 发布者。控制器不创建猜测的雷达安装外参，也不更改建图/定位功能包。

接口实现参考 [ros2_control Humble 硬件组件文档](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)。电机状态和速度帧依据项目内 `dm_can.py` 及 [DM-J4310 厂商手册](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf)。
