# 20261017-lio · 03-LIO-BASE 交付证据索引

任务：HEI-LIDAR-20261017-03-LIO-BASE。分支 `feat/mid360s-lio` 基线 `1aeb8a1`。
主文档：[docs/MID360S_LIO.md](../../../../MID360S_LIO.md)。
结论口径：**静态审计 + 离线回放证据完成；外参未现场标定；无运动数据；不声称 LIO 可靠或 SLAM 完成。**

## 1. 冻结基线（本轮实际读取/构建的 SHA）

| 对象 | SHA / 版本 | 许可（按冻结树内文件） |
| --- | --- | --- |
| FAST_LIO_ROS2（`fast_lio`） | `2fffc570a25d0df172720bac034fbdb6a13d2162`（分支 `ros2`） | `package.xml` 写 BSD，但 `LICENSE` 是 GPL-2.0 全文 ⇒ **元数据矛盾，UNCONFIRMED** |
| ikd-Tree（子模块） | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`（分支 `fast_lio`） | 冻结树只有 3 个文件、**无 LICENSE** ⇒ **UNKNOWN（待人工确认）** |
| livox_ros_driver2（已冻结） | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`（1.2.6） | 沿用 02 阶段记录 |
| Livox-SDK2（已冻结） | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`（v1.3.1） | 沿用 02 阶段记录 |
| 官方手册 | MID-360S User Manual 2026-06-01，sha256 `bf064396c2a5254bc3fc98044d65b2dcf3ad3354e942eee1ee46b37e6e0c80d4` | — |
| 正式 bag | `/home/robo/hei-data/mid360s_live_20261017_retest01` | 02-LIVE 已 PASS |

## 2. 命令与退出码（本轮真实执行）

| 命令 | 退出码 | 备注 |
| --- | --- | --- |
| `bash -n extensions/tools/build_fast_lio_underlay.sh` | **0** | 语法 |
| `extensions/tools/build_fast_lio_underlay.sh`（干净 shell，`PARALLEL=4`） | **0** | 1 package finished，**1min 21s**；patch 逐字节校验通过；`ldd` 无 `not found`；可执行 `fastlio_mapping` 2,084,864 B |
| `colcon build --packages-select hei_robot_localization` | **0** | — |
| `colcon test --packages-select hei_robot_localization` + `colcon test-result` | **0** | **51 tests, 0 errors, 0 failures, 0 skipped** |
| 离线冒烟 smoke03 / smoke04 | 见 §4 | **不作为退出行为验收**（原因见 §4） |

完整日志（不入库，路径留档）：
`/home/robo/hei-data/mid360s_lio_20261017/logs/build_fastlio_v2.log`、
`…/logs/test_localization_pkg.log`、`…/smoke03/`、`…/smoke04/`。

## 3. 输入语义证据（只读短窗口，未全量遍历）

* `data_fastlio_consumer_audit_30s.json` —— 30 s 窗口（300 帧 / 5,998,848 点）：
  帧内逆序、帧尾语义、“过滤后末保留点 vs 最大保留点”的差（94/300 帧 > 0，最大 457,347 ns）、
  `line` 0–3 均分、`tag & 0x30` 三类分布（0x20 占 1.01%，会被 avia_handler 丢弃）。
* `data_imu_window_5-15s.json` —— 2000 条 IMU：加速度均值 `[0.029147, -0.043168, 0.993587]` g、
  范数均值 `0.994976`、gyro 均值 `[-0.006383, 0.000337, 0.002846]` rad/s、orientation 恒 `[0,0,0,1]`。
* `manual_mid360s_imu_section.txt` —— 手册 §3.4/§3.5 原文摘录（IMU 三轴与点云同向、芯片位置 [11.0, 23.29, -44.12] mm）。

## 4. 离线冒烟与“退出行为”边界（重要）

环境：`ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=1`，2 倍速回放正式 bag，只读。

| 轮次 | 收到输出 | 停止方式 | 结果 |
| --- | --- | --- | --- |
| smoke01 | `/Odometry` **522**、点云 **1044**（两路各 522 帧） | 对**进程组** SIGINT（错误） | 双重信号：`fastlio_mapping` exit -2、审计节点关停抛异常。**仅作陷阱记录，不作为任何通过证据** |
| smoke03 | `/Odometry` **790**（全部有限、时间戳单调）、点云 **1578**（仅元数据）、`|pos|max` ≤ **0.102 m** | 只对 launch 父进程 SIGINT | **SIGINT 被忽略**：`/proc/<pid>/status SigIgn=0000000001001006`。原因是后台 `&` 启动的 shell 把 SIGINT/SIGQUIT 置为 SIG_IGN，子进程继承，`setsid` 不会重置信号处置。父进程 exit 143 是 **Codex 有界 TERM 清理**，不是算法退出 |
| smoke04 | `/Odometry` **890**（全部有限、时间戳单调）、点云 **1778**（仅元数据）、`|pos|max` ≤ **0.0999 m** | 同一缺陷 + 有界等待/升级 | 子进程在父进程被清理后自行结束；最终 `NO_RESIDUAL` |

运行中 `ros2 node list` 恰好 4 个节点（`/fastlio_mapping`、`/lio_input_audit`、
`/lio_output_collector`、`/rosbag2_player`）—— **审计节点唯一**；
`fastlio_mapping` 日志各出现一次 `IMU Initial Done` 与 `Initialize the map kdtree`。

**结论边界**：冒烟只证明“有限数值、时间戳单调、frame 契约、输出话题存在、进程可回收”，
**不**证明里程计精度，**不**构成运动验收，**不**证明退出行为合格。
**退出行为验收由 Codex 独立执行**（其 harness 需用 Python `Popen(..., preexec_fn=`把
SIGINT/SIGQUIT 显式恢复为 `SIG_DFL`，且只对父进程发一次 SIGINT 让它转发，避免
`killpg` + launch 二次信号）。本证据目录保留上述失败/受限记录，不做美化。


## 4.1 Codex 独立复跑（最终退出行为证据，结论 PASS）

Codex 独立 harness（Popen + preexec_fn 将 SIGINT 恢复 SIG_DFL，只对父进程发一次 SIGINT）结果：

- launch **exit 0**；3 个子进程全部 cleanly 退出，**无 escalation、无 process died**；
- **420** 条 /Odometry 覆盖源时间 **41.895 s**，全部有限、时间戳单调、四元数模 ≈ 1；
- /cloud_registered **419** 帧 / **56,970** 点，/cloud_registered_body **419** 帧 / **792,839** 点，XYZ 非有限 0，时间戳单调；
- TF 仅 camera_init → body；
- 审计节点仅出现“context 已关停后 take_message”警告并 exit 0；运行期异常仍会传播（有单测固定）。

原始结果：/home/robo/hei-data/mid360s_lio_20261017/codex_smoke/result.json（Codex 自归档）。
Codex 亦独立复跑 143 项 bringup 测试与本包最终 52 项测试。

仍属**静止 bag 离线回放**：只证明有限数值/时间单调/frame 契约/正常退出与回收，
**不**证明里程计精度、**不**构成运动验收。
## 4.2 口径修正（2026-10-17 末轮 review）

- 早期 collector 只对 /Odometry 判定有限性（odom_all_finite）；**点云 XYZ 非有限检查仅在 §4.1 的
  Codex 最终逐点验证中做过**，历史元数据不得被引用为“XYZ 全部有限”。
- 证据目录内 smoke03/smoke04 的 audit JSON 是**当时版本**产物，键名仍为旧 `gravity_direction_est`；
  其值是**比力单位方向（≈ +up）**，不是重力方向。现行代码已改名并显式给出取负的
  `gravity_unit_direction`（见主文档 §9.4）。为保证“原始证据不可改写”，这里保留旧键并加此说明。
- smoke01 的实际计数为 /Odometry 522、点云 1044（此前文档误写 349/698，已按文件核对更正）。
## 5. 未执行 / UNKNOWN

未执行：硬件采集、任何运动、现场外参标定、RTAB-Map、AGX 部署、导航、commit/push。
UNKNOWN：现场 `R_IL`/`t_IL`、IMU 轴符号与安装方向、NoSync 同步误差边界、
IMU 噪声/随机游走、`base_link` 与 `body` 外参、固件/供电记录、上游许可状态。
现场最少操作见主文档 §10。

## 6. Codex 最终 Review / 归档

- [独立离线输出与正常退出](codex_offline_smoke.json)、[可复现独立验证脚本](codex_verify_lio.py)、[独立IMU窗口统计](codex_imu_window.json)。
- [最终接入包构建与52项回归](codex_localization_pytest.txt)、[原bringup143项回归](codex_bringup_pytest.txt)。
- 最终方向语义修正后只更新审计报告双键（比力/重力相反），未改FAST-LIO二进制、launch、参数或原始bag；已重建Python接入包并独立52 passed。完整回放证据保留当时运行SHA，未重写历史JSON。
- 仓库内manual摘录和smoke04停止日志副本仅去除行尾空白；外部原始文件保留。
- 当前仅03-LIO-BASE软件接入通过；03-LIO运动精度、现场轴向/动态外参、SLAM、AGX均未验收。
