# 2026-10-14 初始化证据索引

维护者：Codex。以下只保留本次任务所需片段，不复制完整用户会话、推理流或凭据。

| 文件 | 来源与用途 | 边界 |
| --- | --- | --- |
| [20261014-laptop-events.json](20261014-laptop-events.json) | Codex 用本机 zstd 流式读取实际 DSH 会话日志，按 seq 精确筛选原始工具结果、公开更正文本、turn 终态 | 本地 session-4196ea91-d130-44d7-8b3c-e10c8c30a187；不代表 AGX 日志 |
| [20261014-agx-probe.json](20261014-agx-probe.json) | DSH-AGX 创建远端同名文件；Codex 查看原始命令输出与文件 diff 后转录本地副本，并独立计算 SHA256 匹配 | 是哈希校验后的单文件回收，不是 Git 同步或远程工作区挂载 |
| [20261014-agx-raw-review.txt](20261014-agx-raw-review.txt) | Codex 在 3081 页面展开原始工具记录后提取可见辅助功能节点 | 包含失败退出码2、heredoc重试、版本、哈希、JSON与Git状态；节点编号仅该次UI有效 |
| [20261014-agx-review.jpg](20261014-agx-review.jpg) | 3081 页面实际文件 diff 截图 | 证明查看了远端交付内容，不独立证明机器人可运行 |

AGX 原文件：`/home/user/hei-rebot-lift/docs/agent/evidence/20261014-agx-probe.json`。
两份 JSON 的 SHA256 均为 `097ccc48f3fe48ae33a25c8f501464b8bbed6bc2a22f3bae3ceb87bcea573d31`。
文件内容保持一致，没有为增加本地元数据而改写远端 JSON。

本机事件索引：seq30 开工身份/Git；seq90 算术探针 `PROBE_OK`/`EXIT=0`；seq98 收尾Git；
seq145 首任务完成；seq152 RF-001公开更正；seq154 更正回合结束；seq164 工具 ABORTED；seq167 turn aborted。
本机原日志路径已写入JSON。文件为拼接的 Zstd frames，使用完整流解码，不能只读第一个frame。

## 复核方式

在笔记本项目根目录运行以下只读命令：

```bash
sha256sum docs/agent/evidence/20261014-agx-probe.json
python3 -m json.tool docs/agent/evidence/20261014-agx-probe.json
python3 -m json.tool docs/agent/evidence/20261014-laptop-events.json
```

取消探针的验收依据是运行命令卡、原始 aborted 结果、指定进程核查，未看到 START 的实时 stdout。
AGX 进程复核发生在稍后的证据任务，不能推出取消延迟上限；没有测试队列清空、后台作业终止或机器人急停。
远端 JSON 是 AGX 的唯一新增交付文件；本机文档和其他证据没有自动同步到 AGX。
