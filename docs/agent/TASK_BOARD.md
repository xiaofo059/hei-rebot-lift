# 多 Agent 任务台账（TASK_BOARD）

> ⚠️ **本文件由 Codex 单写。** DSH 仅在 003-DOCS 初始化时写入一次；之后任何 DSH 不得修改本文件。
> 状态定义与流转见 [../MULTI_AGENT_WORKFLOW.md](../MULTI_AGENT_WORKFLOW.md) 第 15 节。
> **只有 Codex 确认验收后才能写入 `DONE`；执行 Agent 报告结束不等于验收通过。**

初始化时间：2026-10-14（Asia/Shanghai）｜初始化者：DSH-LAPTOP（003-DOCS）｜base SHA：`3bef400fcf6cc9860d16f23f0140f3bb255b2716`

## 任务列表

| Task ID | 优先级 | Owner | 执行 Agent | 状态 | 依赖 | 证据/备注 |
| --- | --- | --- | --- | --- | --- | --- |
| HEI-MA-20261014-001-LAPTOP | P1 | Codex | DSH-LAPTOP | DONE | — | 2026-10-14 Codex 核对原始输出及 [本机事件摘录](evidence/20261014-laptop-events.json)：主机/目录/SHA/PROBE_OK；RF-001 撤回后复审接受。 |
| HEI-MA-20261014-001-AGX | P1 | Codex | DSH-AGX | DONE | — | 2026-10-14 Codex 展开原始命令核对 ubuntu/aarch64/根目录/SHA/PROBE_OK。 |
| HEI-MA-20261014-002-CANCEL | P2 | Codex | DSH-LAPTOP / DSH-AGX | DONE | 001-LAPTOP、001-AGX | 能力验证父任务通过：两端 UI 已停止/tool aborted；本机即时 ps 和 AGX 稍后 /proc 检查均无指定探针。仅验证当前 turn，非后台队列或急停。 |
| HEI-MA-20261014-002-CANCEL-LAPTOP | P2 | Codex | DSH-LAPTOP | CANCELLED | 001-LAPTOP | 预期取消的 45 秒前台探针；本机事件 seq164/167 记录 ABORTED。 |
| HEI-MA-20261014-002-CANCEL-AGX | P2 | Codex | DSH-AGX | CANCELLED | 001-AGX | 预期取消的 45 秒前台探针；远端工具 aborted，后续 PROBE_PROC_COUNT=0。 |
| HEI-MA-20261014-003-AGX-EVIDENCE | P2 | Codex | DSH-AGX | DONE | 002-CANCEL | 新建远端证据 JSON，经 Codex 文件 diff 与原始输出复核；[本机核对副本](evidence/20261014-agx-probe.json) SHA256 与远端一致，详见执行日志。 |
| HEI-MA-20261014-003-DOCS | P1 | Codex | DSH-LAPTOP | DONE | 001、002 | 2026-10-14 Codex复审RF-002～008关闭；文档链接、空白、结构与范围检查通过，证据见执行日志。 |
| HEI-MA-20261014-004-ACCEPT | P0 | Codex | Codex | DONE | 001、002、003 | 2026-10-14 Codex独立验收通过；双端UI闭环成立，API/无人值守/部署及硬件未验。详见EXECUTION_LOG最终验收。 |
| HEI-ROBOT-20261014-LIFT-01 | P1 | Codex | DSH-AGX | PLANNED | HEI-LIFT-20261014-PREFLIGHT、HEI-LIFT-IO-READONLY-02 | 用户明确授权纯升降动作“下至底端后回0 mm”；现场人员确认行程/负载/急停条件。源码只读。AGX空闲、tty无占用；限位IO 2秒20帧新鲜，upper active/lower released。下限位与满行程未验证；按软限位与实时IO监控逐步下降，若下限位不触发则不超−700 mm；之后只沿回程向上归零。异常/反馈丢失即停止并由现场急停兜底。 |

## 初始化任务计划与验收范围

下表的编号对应上表完整 Task ID，优先级、执行方、依赖与实时状态以任务列表为准。

| 编号 / 名称 | 目录与允许修改 | 实施步骤 | 主要风险 | 验收标准 |
| --- | --- | --- | --- | --- |
| 001 双端环境与最小闭环 | 各自主目录下本项目；只读 | 确认空闲会话→分派主机/Git/Python/标准库探针→回收原始输出→Review | 端口与机器混淆、错误解释摘要 | 两端身份不同且目录正确；各自 PROBE_OK；tracked 无新增改动 |
| 002 双端停止能力 | 无文件修改；仅前台标准库 sleep | 派发带唯一标识的45秒任务→观察运行→停止→检查输出/指定进程 | 将 turn取消误当物理急停 | 两端 aborted；指定探针不残留；明确队列与后台任务边界 |
| 003-AGX-EVIDENCE 远端文件交付 | AGX 仅新增 docs/agent/evidence/20261014-agx-probe.json | 核对环境→复核取消探针→写 JSON→读取/diff/哈希→主控核验 | 覆盖已有文件、错误远端结果 | JSON 可解析；机器/SHA/版本符合原始输出；哈希核对；无其他 tracked 修改 |
| 003-DOCS 流程文档实施 | 五份指定文档；README_FOR_AGENTS.md、docs/README_zh.md 各一处导航 | 读取原有架构/Git文档→编写→自检→Codex Review→DSH修复 | 把初始化限制推广为长期规则、伪造能力 | 覆盖用户八阶段/十四字段/七层测试及真实能力边界；所有 findings 关闭 |
| 004-ACCEPT 独立验收与归档 | Codex 维护本台账、追加日志、evidence | 独立读文档/diff→检查链接/空白/JSON→记录缺口→总结 | 只相信 DSH 报告、忽略未跟踪文件 | 结果可追溯；原有 dirty 保留；未实测项明确；不误报机器人功能通过 |

本轮不进行算法开发、训练、仿真、部署、硬件读写或运动测试。新任务需重新检查主机、目录、Git dirty 和解释器；当前 PID/端口是日期快照。

## MID-360S 项目任务（2026-10-16）

用户已批准分阶段实施；当前批次为 01/02。路线：笔记本接入 → FAST-LIO2 → RTAB-Map 三维闭环/地图定位 → AGX 安装 → 桥接 → Nav2。首版室内平地，允许 RViz 指定大致初始位置。配套供电/网线已备齐，但用户本轮确认雷达**尚未连接**，接好后通知。后续底盘运动须另获授权。

开发分支 `feat/mid360s-bringup`，base `f97fc4c14216d202d63269a8de1424ae646ca14d`；开工前工作区干净。Codex 单写任务板和执行日志，DSH 仅写任务白名单内实现和专项报告。

| Task ID（前缀 HEI-LIDAR-20261016-） | 优先级 | Owner | 执行 Agent | 状态 | 依赖 | 证据/验收范围 |
| --- | --- | --- | --- | --- | --- | --- |
| 01-SOFTWARE | P1 | Codex | DSH-LAPTOP | DONE | — | UI 实际派发/执行；Codex 复核原始机器/Git输出，独立 source Humble 后 rclpy、rosbag2_py 导入成功。仅软件基线通过。 |
| 01-HARDWARE | P1 | Codex | DSH-LAPTOP + 用户接线 | BLOCKED | 用户接好雷达 | 型号/固件、供电接线、实际 IP；未连接不能判定通过。 |
| 02-PREP | P1 | Codex | DSH-LAPTOP | DONE | 01-SOFTWARE | 2026-10-16 Codex 复审 001～005 关闭；独立 pytest 126 passed，5 个异常 bag 全部 exit 1、正常合成对照 exit 0；仅软件准备验收。详见执行日志及 20261016-mid360s-codex-review.json。 |
| 02-LIVE | P1 | Codex | DSH-LAPTOP | BLOCKED | 01-HARDWARE、02-PREP DONE | 等用户接好雷达；点云/IMU/逐点时间、30 分钟稳定性、原始 bag 与回放均 NOT_RUN。 |
| 03-LIO | P1 | Codex | DSH-LAPTOP | PLANNED | 02-LIVE DONE | FAST-LIO2；先回放后小推车采集，检查外参/时间/去畸变。 |
| 04-SLAM | P1 | Codex | DSH-LAPTOP | PLANNED | 03-LIO DONE | RTAB-Map 闭环优化、数据库/优化点云/轨迹保存；3 个位置重启定位验收。 |
| 05-AGX | P1 | Codex | DSH-AGX | PLANNED | 04-SLAM DONE | 同 SHA ARM64 部署、固定底盘安装、外参/遮挡、30 分钟性能。 |
| 06L-BRIDGE | P1 | Codex | DSH-LAPTOP | PLANNED | 04-SLAM DONE | 假 host 验证 SI 单位、完整 18 维动作、控制权、超时停止。 |
| 06A-BRIDGE | P1 | Codex | DSH-AGX | PLANNED | 05、06L Review、运动授权 | 轮组方向/速度标定、停止延迟和停车距离。 |
| 07L-NAV | P1 | Codex | DSH-LAPTOP | PLANNED | 04、06L Review | 三维地图生成二维导航地图；RTAB-Map 定位、DWB 全向、点云障碍层。 |
| 07A-NAV | P1 | Codex | DSH-AGX | PLANNED | 05、06A、07L Review、运动授权 | 初始 0.1 m/s、0.2 rad/s；至少 5 个目标、障碍、取消、定位/通信丢失测试。 |

未执行项均记 NOT_RUN；任何阶段数据/定位异常均不推进后续阶段。专项启动说明及真实结果见新增 bringup 文档和执行日志。

### 后续阶段的批准验收目标

- 02-LIVE：点云 10 Hz 设置时平均 9–11 Hz，IMU 达到已核实标称频率；至少 1800 秒双流有效覆盖，无大于 0.5 秒断流、源时间倒退/冻结；保存 CustomMsg/IMU 原始值及已有 TF。建议录 31 分钟留启停余量；物理点云、轴向与时间同步另核验，软件 QC 不替代这些检查。
- 03-LIO：先离线回放，再固定小推车实采直线/横移/转弯/静止/闭环；外参和 IMU 单位处理必须显式适配，不把已有 imu_link 冒充内置 IMU。
- 04-SLAM：结构明显的室内约 30–50 米闭环，有回环约束及优化证据；墙面重复扫描错位目标 ≤10 cm，标记起点误差目标 ≤15 cm/5°。3 个已建图位置重启，允许约 0.5 m/20° 初始提示，目标 30 秒内恢复到 ≤20 cm/5°。交付数据库、优化点云和轨迹。
- 05-AGX：目标机独立构建已审定 SHA，固定底盘安装、实测外参/遮挡；30 分钟资源/温度/延迟记录；不把查版本当部署通过。
- 06L/06A：唯一网关，SI 单位转换，完整 18 维动作与新鲜状态保持，导航/原遥操作互斥；存活网关输入超过 250 ms 未更新撤销速度。验证退出、反馈过期、TF/定位丢失、通信中断，独立测 host 看门狗与实际停车距离；动作测试须新授权。
- 07L/07A：RTAB-Map 唯一发布 map→odom，LIO 适配器唯一发布 odom→base_link；二维可通行地图、点云障碍层与 DWB 全向控制。初始平移 ≤0.1 m/s、角速 ≤0.2 rad/s，至少 5 个目标，目标误差 ≤20 cm/10°；取消、障碍和数据/定位丢失均可停止或安全重规划。初版固定收拢双臂与升降，轮廓按整机测量。

## 维护规则

1. 每次状态变更必须同时写入证据来源（命令、退出码、原始输出位置）。
2. 不删除历史条目；状态回退时保留原结论与更正记录。
3. 新增任务前先检查 ID 唯一性与依赖。
4. `DONE` 由 Codex 在验收后填写，并注明验收日期与依据。

## 当前终态与异常状态规则

截至2026-10-14，本轮6项DONE、2项预期CANCELLED；无未结束任务。取消子任务的工具中止是停止能力验证的预期结果。

- TODO：尚未规划；PLANNED：任务、依赖、白名单与验收已定义。
- DISPATCHED：发送后已看到回显；RUNNING：已看到执行证据；REVIEW：交付可审查；TESTING：主控开始独立验证；DONE：证据满足验收。
- BLOCKED：缺少明确前置条件，记录所需输入/恢复条件；FAILED：本次执行失败且未达验收。
- NEEDS_FIX：Review发现需修问题，修复后回REVIEW再验证；CANCELLED：任务被取消，不等于所有后台进程或硬件停止。
- 恢复或重派必须确认当前turn、队列、指定进程及工作区状态；不得盲目重复提交。
