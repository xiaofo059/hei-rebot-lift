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
