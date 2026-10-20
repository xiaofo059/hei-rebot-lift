# 执行与复审日志（EXECUTION_LOG）

> **追加式日志**：执行 Agent 只追加自己的条目，不改写他人条目；Codex 可追加 Review/复审结论。
> 证据要求见 [TASK_TEMPLATE.md](TASK_TEMPLATE.md)；流程见 [../MULTI_AGENT_WORKFLOW.md](../MULTI_AGENT_WORKFLOW.md)。
> 本文件不复制会话原文；只记录可核查的摘要与证据位置，并做脱敏。

---

## 2026-10-14 · HEI-MA-20261014-001-LAPTOP · 双端只读探针（笔记本）

- **机器指纹**：`fo` / x86_64 / `/home/robo/robotopia/hei-rebot-lift` / DSH `0.2.0-rc.2`（PID 60284，监听 `127.0.0.1:3080`）
- **base SHA**：`3bef400fcf6cc9860d16f23f0140f3bb255b2716`（`dev`）
- **执行内容**：`hostname`、`uname -m`、`pwd`、`git rev-parse --show-toplevel`、`git status --short --branch`、`git rev-parse HEAD`、`git remote -v`；Python 路径/版本；`conda env list`；`dsh --version`；`ps`/`ss` 核对；标准库探针 `sum(range(10)) == 45`。
- **结果**：全部命令退出码 0；探针输出 `PROBE_OK`；dsh web 仅监听回环 `127.0.0.1:3080`。
- **改动**：无（tracked 零改动）。
- **未验证**：硬件、机器人库、ROS 2 骨架、远端新鲜度（未 fetch）。
- **Review finding**：RF-001（P2）——误报 SVG "并发重命名"并归因 Kilo。
- **复审记录**：Codex 展开首末原始 `git status` 证伪；执行端**撤回**该结论与归因；Codex 接受更正。
- **状态**：REVIEW → 终态由 Codex 判定。

## 2026-10-14 · HEI-MA-20261014-001-AGX · 双端只读探针（AGX）

- **机器指纹**：`ubuntu` / aarch64（Jetson AGX Orin）/ `/home/user/hei-rebot-lift` / DSH `0.2.0-rc.2`（PID 18465）
- **base SHA**：同一 `dev` @ `3bef400fcf6cc9860d16f23f0140f3bb255b2716`
- **结果**：原始 `hostname`/`uname -m`/根目录/SHA/`PROBE_OK` 已由 Codex 展开核对。
- **未验证**：业务解释器版本（`lerobot5`/`rebot`）待复核。
- **状态**：REVIEW → 终态由 Codex 判定。

## 2026-10-14 · HEI-MA-20261014-002-CANCEL · 中断演练

- **执行内容**：两端各运行一次 `python3 -u -c "...time.sleep(45)..."`，前台执行，不重试。
- **结果**：运行约 **24 秒**时被 Codex 点击停止；UI 显示已停止；工具返回 `Error: tool call aborted`。开始输出可见。
- **核对**：本地 `ps` 未发现残留 probe 进程；**AGX 子进程复核待补**。
- **结论边界**：DSH 停止**不能保证**后台任务/队列/硬件动作全部停止；**不得将 DSH 停止当作机器人急停**。
- **未验证**：AGX 残留进程、硬件侧影响、队列中未开始的任务。
- **状态**：REVIEW。

## 2026-10-14 · HEI-MA-20261014-003-DOCS · 多 Agent 文档基础设施（笔记本）

- **机器指纹**：`fo` / x86_64 / `/home/robo/robotopia/hei-rebot-lift` / DSH `0.2.0-rc.2`
- **base SHA**：`3bef400fcf6cc9860d16f23f0140f3bb255b2716`（`dev`）
- **允许新增**：`docs/MULTI_AGENT_WORKFLOW.md`、`docs/agent/TASK_TEMPLATE.md`、`docs/agent/TASK_BOARD.md`、`docs/agent/REVIEW_CHECKLIST.md`、`docs/agent/EXECUTION_LOG.md`
- **允许修改**：`README_FOR_AGENTS.md`、`docs/README_zh.md`（各一处简短导航链接）
- **Git 例外**：串行文档任务，直接在 `dev` 工作区实施，**不新建分支、不 commit/push**。
- **检查**：5 文件存在性、相对链接、`git diff --check`、`git status` 范围核对。
- **未验证**：内容在真实多 Agent 调度下的运行效果；英文文档 `docs/README.md` 未同步（超出本次授权范围）。
- **状态**：REVIEW。

### 003-DOCS 修复轮（RF-002 ~ RF-007）· 追加记录

- **性质**：Codex 独立 Review 判定 `NEEDS_FIX` 后的**最小修复**。
- **TASK_BOARD 未改动**（由 Codex 单写；本轮只改主文档、模板、清单与日志）。
- **RF-002**：主文档 §5 由"项目路线图"更正为**每个任务**的 Phase 1–8 工作流程（需求分析 → 任务规划 → 任务分配 → 生成 14 字段提示词 → 执行监控纠偏 → 独立 Review 与修复循环 → 分层测试验收 → 总结）；删除"规划项仓库没有对应代码"的泛化，改为"不得由未执行推断未实现"。
- **RF-003**：§10 与模板 §9 覆盖 7 级测试（静态 / 单元 / 模块集成 / 本地仿真 / AGX 软件环境 / 硬件非运动 / 实机运动），L0 环境探针为可选附加；每级记录通过/失败/未执行/不适用与证据；**代码实现 / 仿真通过 / AGX 部署 / 实机通过**分开陈述；纯仿真与可逆文档任务不新增无关确认要求。
- **RF-004**：REVIEW_CHECKLIST 补全通用开发审查项（代码正确性/接口/数据结构/异常、坐标系 TF 与 rad/m/mm、关节顺序方向与 IK 约束、限位速度加速度与急停超时掉线异常指令上电使能、维护性/日志资源/并发/依赖/测试、18 维与三相机/ROS 2 骨架/ARM64 兼容）；范围判据改为"**新增差异 ⊆ 白名单 且原有 dirty 保留**"；明确清单面向未来开发，本轮约束非永久规则。
- **RF-005**：§1/§6/§12 与模板 §6 区分**长期规则**与**本次约束**；保留实机运动等用户逐项授权门槛；补充新 Agent 上手顺序（先 README_FOR_AGENTS 与架构文档 → 再本工作流 → 再接任务）。
- **RF-006**：§8 改为可执行步骤（派发前置检查 3080/3081、目标 cwd/host/base SHA、会话空闲与队列；任务回显才 `DISPATCHED`、工具运行才 `RUNNING`、原始输出与 diff 回收进入 `REVIEW`；仅活跃回合检查点轮询，无永久监控）；§13 更新故障表（UI 失联停止派发、日志仅取证非派发通道、API 仅源码确认未集成、CLI headless 未试运行且不能接管已锁定 web 会话、手工降级步骤）。
- **RF-007**：主文档 §3.1/§3.3 更新 AGX 事实并追加更正（见下）。

### 更正记录（追加，不改写原条目）

1. **002-CANCEL "开始输出可见" 不实**：实际看到的是"运行中的命令卡"，**未看到 `START` 的 stdout**。原条目保留，以本条为准。
2. **002-CANCEL "AGX 子进程复核待补" → 已更新**：AGX 侧 `/proc` 复核 `PROBE_PROC_COUNT=0`（后续复核值）；该值**不能**用于宣称"停止延迟上限"。
3. **002-CANCEL 状态语义**：子任务状态 `CANCELLED` 属**预期测试结果**（停止能力验证的正常产物），不表示失败；父级"停止能力"验证可由 Codex 判定 `DONE`。
4. **001-LAPTOP RF-001 根因更正**：并非"转义文件名差异"，而是**误读转义文件名**；首末两次 `git status` 的文件名**实际相同**，**无已证实的并发修改**，Kilo 归因为无依据推断。
5. **001-AGX "业务解释器版本待复核" → 已更新**：`lerobot5` → `/home/user/miniforge3/envs/lerobot5/bin/python`，Python **3.12.14**。
6. **证据归属**：AGX 远端新增 `docs/agent/evidence/20261014-agx-probe.json`，SHA256 `097ccc48f3fe48ae33a25c8f501464b8bbed6bc2a22f3bae3ceb87bcea573d31`；Codex 已看原始输出与文件 diff 并**独立匹配哈希**。`docs/agent/evidence/` 与本地 `evidence/` 由 **Codex 管理**，执行 Agent 不修改、不新建。
7. **跨机同步**：本套文档当前**仅存在于笔记本工作区**；AGX 不会自动拥有。

### 003-DOCS 第二次复审修复（残留 8 项）· 追加记录

- **性质**：第二次独立复审后的**仅相关文档**修复；TASK_BOARD 与 `docs/agent/evidence/` **未改动**（由 Codex 管理）。
- **保留原则**：以下为**追加更正**，历史条目（含"待补""开始输出可见"）一律保留不改写。

1. **API 表述更正**（原"仅源码层面确认存在未认证调用路径"易误导安全判断）：
   准确表述为——已从**安装源码**确认 API 存在，并要求 **Host/Origin 检查与认证 Cookie**；
   本轮**没有执行独立认证 API 调用**，**未实现 API 调度**。§13 故障表同步改为"**有源码证据、无调用实测**"。
2. **硬件禁令表述更正**（原"每个子任务都必须自带禁硬件约束"是永久禁令）：
   改为**每个任务写清硬件允许/禁止范围**、无授权不得硬件访问、本次初始化全部禁止；
   危险操作须用户明确授权且永不绕过联锁；**普通 L6 只读核查按已有任务授权执行**，不要求逐项重复确认。
3. **分层选择更正**（原"纯仿真或可逆文档任务通常只需 L1"错误）：
   改为**文档任务通常 L1**、**仿真任务按目标执行 L2/L3/L4**，均不需要无关硬件确认；
   明确**仅查版本不构成 AGX 部署通过**、**L6 通过不能宣称运动通过**（L6/L7 结论拆开）。
4. **规则范围更正**：§1 标题改为"本次初始化非目标（003-DOCS）"；
   §6 第 2 条由"默认拒绝"改为**核对已有任务授权与风险、关键缺项先澄清**，
   避免已授权的可逆环境工作被永久拒绝。
5. **状态语义更正**：§5 Phase 8 是 **Codex 已验收后的最终总结**（非"等待验收"）；
   §8.2 明确"**输入 → 点击发送 → 看到回显**"才 `DISPATCHED`，**实际开始独立验证**才 `TESTING`，
   结果完成后才判定 `DONE` / `NEEDS_FIX`。
6. **证据归属更正**（本条**取代**上一轮更正记录第 6 条中"本地 `evidence/`"的表述）：
   本机 `docs/agent/evidence/` 由 **Codex 单写**；AGX 该 JSON 是 **DSH-AGX 按专门白名单**创建；
   未来按**每任务 file owner** 协调。**不存在根目录 `evidence/`**，此前表述作废；
   本机核对副本**已落盘**，不是未来计划。
7. **模板与清单更正**：模板元数据补"**任务名称**"；明确**允许实例化任务计划**，
   禁止的只是复制另一份模板定义。清单支持 `PASS`/`FAIL`/`NOT_RUN`/`N/A(原因)`；
   **超时失能由设备安全设计决定，不得一概掉电**（重力轴风险）。
8. **AGX 真实异常补充**（不能称所有 AGX 命令都一次成功）：首次 `/proc` 扫描因 shell 引号错误
   返回 **exit 2**：`unexpected EOF while looking for matching double quote`；
   DSH-AGX **改用 heredoc 只读重试**成功，得到 `PROBE_PROC_COUNT=0`。
   该次失败与重试输出已由 Codex 通过 UI 展开核对；**无持续失败、未扩大权限**。

### 003-DOCS RF-008（P1 阻断项）· 追加记录

- **矛盾修复**：`REVIEW_CHECKLIST.md` 原第 65 行"异常退出路径会失能"与第 62–63 行"不得一概掉电/失能（重力轴风险）"冲突。
  现改为：**上电与使能**——使能/失能顺序明确；异常退出按**该轴或机构经评估的安全策略**处置
  （如保持、制动、受控停止或安全失能），并**验证执行顺序**。
- **排查**：全文扫描"失能/掉电"，未再发现任何**一概失能**的要求；余下出现处均为已限定的安全策略表述。
- **措辞统一**：主文档 §7 与清单 I 统一为"**模板定义只维护一份，允许实例化填写任务计划**"。
- **本轮范围**：只改主文档、清单与日志；TASK_BOARD 与 `docs/agent/evidence/` 未改动；未运行硬件。

---

## 记录约定

- 每条包含：日期、Task ID、机器指纹、base SHA、执行摘要、结果、改动、未验证项、状态。
- 发现错误结论时**追加更正条目**，不删除原条目。
- 不记录 token、私钥、证书、完整用户历史或大段日志原文。

## 2026-10-14 · Codex 最终独立验收 · HEI-MA-20261014-004-ACCEPT

**结论：本轮协作基础设施验收通过。** 适用范围为在线浏览器会话的双端分派、取证、取消与审查修复闭环；不是机器人算法、部署或硬件功能验收。
执行分工：DSH-LAPTOP 实施五份文档和两处导航并修复；DSH-AGX 实施只读探针与远端证据文件；Codex 主控维护台账、提取证据、独立审查与验证。内部 Codex 辅助审查 Agent 仅只读复核，不冒充 DSH 执行端。

### 环境与能力验收

| 能力 | 笔记本 DSH | AGX DSH | 实际证据与边界 |
| --- | --- | --- | --- |
| 页面可达、发送任务 | PASS | PASS | 3080/3081 各自出现任务回显并执行；001 探针与003交付均完成 |
| 状态与命令输出读取 | PASS | PASS | 运行卡、完成/停止状态、展开原始工具输出；不依赖口头报告 |
| 取消当前 turn | PASS（限定探针） | PASS（限定探针） | 前台45秒探针被中止，原始工具 aborted；未观测到 START stdout |
| 指定探针残余检查 | PASS（即时 ps） | PASS（稍后 /proc） | 未见指定探针；AGX复核不能证明取消延迟上限 |
| Review 纠正指令 | PASS | PASS | LAPTOP RF-001及文档多轮修复；AGX RF-AGX-001报告补正已收到并结束 |
| 修改文件读取 | PASS | PASS（通过UI） | 本机直接读文件/diff；AGX展开文件diff并匹配回收副本哈希 |
| 会话日志文件读取 | PASS | 未验证 | 本机读取准确 session 的 Zstd；AGX本轮仅读取UI工具日志，未直接读取远端压缩日志 |
| 独立 HTTP API / headless 调度 | 未验证 | 未验证 | API为源码核查、CLI为帮助核查，均未执行独立任务调用 |
| 清空排队任务 / 所有后台作业停止 | 未验证 | 未验证 | 源码表明 cancel 保留 inbox，不能将单次停止推广至全部作业 |
| 无人值守持续监控 | 未建立 | 未建立 | 本轮只采用活跃会话检查点轮询 |

Codex 本机 `ps`/`ss` 再核对：60284 为 pts/4 的本地 DSH；60751 为 pts/2 的现有 SSH；61088 为 pts/3 的3081隧道。
3080/3081 均在本地回环监听。三个进程角色已核实，但终端工具只能读取当前终端，未逐屏读取三个终端缓冲区。
新建 BatchMode SSH 返回 `Permission denied (publickey,password)`；已有会话/隧道保持可用。未尝试读取认证凭据或改变SSH配置。

### Review findings 与复审结论

| Finding | 严重度 | 修复与验收 |
| --- | --- | --- |
| RF-001 | P2 | 撤回SVG并发重命名推断；原始首末git输出相同；关闭 |
| RF-002 | P1 | 八阶段改为每任务完整流程；关闭 |
| RF-003 | P1 | 七层测试、部署与运动结论分开；关闭 |
| RF-004 | P1 | 补齐算法/安全/工程/平台审查与增量dirty判定；关闭 |
| RF-005 | P2 | 长期规范与本次约束分离；关闭 |
| RF-006 | P2 | 实际UI派发/状态/取消/降级步骤及API边界；关闭 |
| RF-007 | P2 | 最新AGX版本、残余检查、未看到START、证据归属更正；关闭 |
| RF-008 | P1 | 删除一概异常退出失能要求，按具体机构安全策略处置；关闭 |
| RF-AGX-001 | P2 | 远端补报首次引号错误exit2、heredoc重试成功及证据边界；关闭 |

最后一次安全条目修复后，Codex 重新读取文件确认；辅助只读审查亦核验了本机事件摘录与原日志一致。
历史日志中的 REVIEW/待补/旧表述为当时状态，以本节和任务台账最终状态为准。

### 验证与文件回收

- L0：双端身份/Git/解释器与标准库算术探针通过。
- L1：主控独立检查文档相对链接、代码围栏闭合、新文件尾随空白/末尾换行、tracked `git diff --check`、十四字段与八阶段/七层结构，通过。
- JSON：两份证据文件可解析；本机精确事件序号及 aborted/PROBE_OK 与原日志一致；AGX JSON 本机独立SHA256匹配远端。
- L2/L3/L4：本轮未执行机器人单元、集成或仿真测试，对文档初始化不适用。
- L5：只核查AGX身份与解释器版本；依赖导入、构建、部署测试未执行，不能报告部署通过。
- L6/L7：硬件非运动及运动测试均未执行，未使能电机，未调用机器人控制库。
- Git：HEAD仍为 `3bef400fcf6cc9860d16f23f0140f3bb255b2716`；索引无变更；已有tracked文件仅两个README各加一行导航。新增均为指定文档、台账和本次证据；原有 `.kilo/`、SVG保留；无commit/push。
- [证据索引](evidence/README.md)、[最终静态检查记录](evidence/20261014-acceptance.json) 可复核。记录是本次快照，不是永久监控服务。

**同步记录**：远端 `/home/user/hei-rebot-lift/docs/agent/evidence/20261014-agx-probe.json` 由 DSH-AGX 创建。
Codex 通过UI回收可见内容，转录到本机同名相对路径，独立SHA256为
`097ccc48f3fe48ae33a25c8f501464b8bbed6bc2a22f3bae3ceb87bcea573d31`，与远端原始sha256sum一致。
这仅是证据单文件回收，整套文档未同步到AGX。当前同HEAD不表示两端工作区相同。

### 未解决的能力限制与下一步

- 当前调度可用依赖两端 DSH、既有SSH隧道、已登录浏览器会话；结束本轮后没有自动后台监控。下次派发前重新核对身份与会话状态。
- 页面显示“完全权限”；任务白名单是提示词与Review约束，**不是新建立的操作系统沙箱或硬件联锁**。任何真实运动仍须用户明确授权和独立安全措施。
- 无需用户手工提交本轮任务。若日后需脱离浏览器独立远程读文件/运行命令，需配置可用SSH认证或另立任务验证受认证API；本轮未扩展访问权限。
- 下次AGX正式开发前，按Fork开发分支/Git工作流同步审定文档并记录commit和目标HEAD；当前无自动提交或同步。
- 现有初始化闭环无剩余阻断项；任意长时任务、断线恢复、排队取消和实机安全的可靠性仍需各自专项验证。

## 2026-10-16 · MID-360S 第一批 01/02 启动（Codex）

- 用户批准实施分阶段计划；最新现场回复：雷达尚未连接，接好后通知。当前不具备真实数据、固件或实际雷达 IP 证据；不启动机器人 host、不操作电机。
- 基线：`fo` / x86_64 / Ubuntu 22.04.5；工作区起始干净，`dev` @ `f97fc4c14216d202d63269a8de1424ae646ca14d`。Codex 创建 `feat/mid360s-bringup`（exit 0）。
- 本机 DSH：PID 140208，进程 cwd 经 `/proc/140208/cwd` 核对为本项目；3080 UI 新建独立会话“HEI-LIDAR 软件只读基线检查”，已输入/发送十四字段 01-SOFTWARE 提示词，实际观察到工具运行与最终报告，展开核对 Git 原始输出。不是 API 调度，也不是 Codex 内部子 Agent。
- 01-SOFTWARE 接受范围：机器/目录/SHA 一致，首末 status 干净；ROS Humble/RViz、colcon、CMake 和编译器存在。Codex 独立以 `bash --noprofile --norc` + `source /opt/ros/humble/setup.bash` + `/usr/bin/python3` 导入 rclpy/rosbag2_py 成功（exit 0）。PATH-only 导入失败属于未 source，不能声称 ROS 缺失。vcstool 非必需，可用精确 SHA 的 Git 获取依赖。
- 权限边界：DSH 工作区沙箱中的 sudo 报 no-new-privileges；Codex 本机 `sudo -n true` 提示需要密码。是两种不同限制，不代表用户机器 sudo 配置损坏；不读取口令、不调整全局权限。
- 网络快照：enp7s0 `192.168.50.1/24`，邻居 `192.168.50.2`；Wi-Fi `192.168.1.144/23`，默认路由 via `192.168.1.1`；常见雷达默认网段与 Wi-Fi 重叠。未改配置，已有邻居不是已确认的雷达。
- 02-PREP 范围：雷达缺席期间仅准备独立依赖、参数化 bringup、录包/质量工具及文档。01-HARDWARE 与 02-LIVE 不作通过结论；03 及以后保留前置验收门槛。

### 02-PREP 阶段 Review（尚未验收）

- DSH 实际下载并冻结官方 Livox-SDK2 v1.3.1 @ `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`、livox_ros_driver2 1.2.6 @ `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`；源代码包含 Mid360s handler/config。安装仅在被忽略的 `extensions/third_party/`。
- SDK 和 driver 实际构建；初次遇 Conda CMake 4.x 与旧最低版本不兼容，改用系统 CMake 的干净 ROS shell。驱动组件初次缺 SDK 运行库路径，DSH 添加安装 RPATH 后，Codex 独立 source underlay 并检查 `liblivox_ros_driver2.so`：SDK 解析到本项目安装前缀，`not found=false`、exit 0。原始消息 `ros2 interface show livox_ros_driver2/msg/CustomMsg` 成功。
- RF-LIDAR-001 P1：Codex 独立调用配置生成器的目录校验，首次 `extensions/config/local/` 尚不存在时实际抛 FileNotFoundError；要求向上寻找存在的祖先并回归测试。
- RF-LIDAR-002 P2：构建脚本须拒绝 tracked 源码 dirty，保护已有 driver/launch，并将 ldd 自身失败判为失败；不能仅比对 HEAD 后宣称冻结源码可复现。
- RF-LIDAR-003 P2：核对冻结 driver 的硬编码 IMU frame `livox_frame`、加速度原样传递。Codex 读取官方 MID360 系列协议：acc 单位 g、gyro 单位 rad/s；原始 bag 不改值，下游不能未经适配直接按 SI 融合，orientation 未提供。DSH 自身获取协议页被网络策略拒绝，记录限制并用冻结源码核对；不绕过策略。
- RF-LIDAR-004 P1（阻断）：Codex 通过真实 rosbag2 API 生成 **SYNTHETIC** 临时包并调用 CLI；IMU 仅 0.1 秒、IMU/点云时间完全不重叠、所有点云非法 CDR、所有点云 width/data/fields 为空，四例实际均错误 exit 0。要求双流全时段/断流/源时间检查、损坏/空点云失败、默认完整 header 扫描、采样结果不得 PASS、阈值校验及对应回归。临时包在 data/mid360s 下独立临时目录，用后清理；从未发布为实时传感器。
- 以上纠正已通过 DSH 排队发送后“插话发送”即时交付并看到任务回显；当前执行继续，不能宣称已经修复或最终通过。

### 02-PREP 最终独立验收（Codex，2026-10-16）

- RF-LIDAR-005 P2：文档原先重编号阶段、将未执行命令统称已实测、构建目录未回根目录，以及正式包未明确 CustomMsg 切换；要求对齐批准计划、逐块标执行状态、修正 source/路径、单 driver 切换与 31 分钟录包说明。已复核修正，历史 DSH 交付报告保持“等待 Review”的原始时态；当前结论以本节为准。
- RF-LIDAR-004 首次修复仍漏掉中段冻结：Codex 用合法 24 字节点步长的 SYNTHETIC PC2，源时间第 6～10 帧冻结到第 5 帧后恢复，实际仍 exit 0。再次即时纠正，改为逐相邻源时间对检查重复/倒退及源时间最大间隔，增加话题名不能相同的参数检查。
- Codex 独立测试命令：干净系统 shell source `/opt/ros/humble/setup.bash`、Livox underlay、项目 overlay 后，`/usr/bin/python3 -m pytest -q extensions/ros2_ws/src/hei_robot_bringup/test`；实际 `126 passed in 5.04s`，exit 0（无跳过）。DSH 另有 colcon build/test 记录，不能以其自述替代本独立结果。
- Codex 在修复后的代码上重新用真实 rosbag2 API 构造 **SYNTHETIC_ONLY** 的 6 个独立对照：IMU 短流、源时间不重叠、非法 CDR、空点云、中段时间冻结，全部 exit 1；合法合成对照 exit 0。临时数据生成在忽略目录，用后清理，没有发布实时话题。此结果不代表真实雷达通过。
- 首次配置目录问题已有缺失多级路径回归；构建脚本已有 tracked dirty 拒绝、预存 launch 保护和 ldd 失败检查；源代码/冻结 SHA/实际组件库运行路径再次核对。RF-LIDAR-001～005 关闭。DSH 沙箱不能获取协议页不影响 Codex 已通过官方网页与冻结源码双重核对单位事实。
- 语法与范围：`bash -n extensions/tools/build_mid360s_underlay.sh`、`git diff --check` exit 0；第三方源码交付时干净。DSH 的 dirty 保护测试曾临时修改并还原其任务内下载的 CHANGELOG，不能将“交付时干净”扩张为“全过程未修改”。
- 结论：01-SOFTWARE 与 02-PREP DONE；01-HARDWARE、02-LIVE BLOCKED 等接线。真实驱动启动、点云/IMU、RViz、正式 bag/回放、LIO/SLAM/导航、AGX、运动均 NOT_RUN。没有修改网络配置、机器人控制源码或设备状态。未部署持续后台监控。
- 专项输出：`docs/MID360S_BRINGUP.md`；DSH 原始证据 `docs/agent/evidence/mid360s/`；独立验收 JSON `docs/agent/evidence/20261016-mid360s-codex-review.json`。本批审查通过的源码、文档与证据在功能分支提交；具体提交 SHA 以 Git 日志为准，不向远端或上游推送。
- 最终文档 QA：Codex 发现包 README 的 4 处相对路径层级错误，作为 RF-LIDAR-005 的补项重新交给 DSH；DSH 仅修改该 README。Codex 复查 3 份新增说明的 11 个相对链接，0 损坏，空白检查通过。软件源代码未变化，不重复无关测试。独立验收 JSON 追加已审查源文件 SHA256，便于关联本次测试与交付版本；DSH 当前回合结束且无待发送队列，未见 driver/robot host 进程。

## 2026-10-16 · MID-360S 真实点云冒烟测试（Codex）

- 用户确认雷达已通过 USB 转网口接入笔记本，并授权启动 Livox driver、RViz；本次只测试传感器，不启动机器人 host、不控制底盘或升降，不修改源码/现场配置。
- 启动前确认无旧 driver/RViz 残留。实时网络快照：USB ASIX `enx9c69d3cd61a9=192.168.1.5/32`，雷达 `192.168.1.135` 有直连 `/32` 路由且邻居 REACHABLE；WiFi `192.168.1.144/23` 与机器人高速口 `enp7s0=192.168.50.1/24` 保持分离。
- 实际启动 `livox_ros_driver2 1.2.6` + MID-360S SDK2 `v1.3.1` + 自研 `mid360s_bringup.launch.py`，仅一个 driver，格式 PointCloud2，设定 10 Hz。RViz 点云 Status OK；用户截图约 31 FPS。独立运行约 15 秒 `ros2 topic hz /livox/lidar`，统计均值约 9.6 Hz（窗口中最大到达间隔约 0.2 秒）。只证明本次点云链路与可视化；未测 `/livox/imu` 频率、CustomMsg 逐点时间或录包。
- RViz 同时报告 `rviz_default_plugins/Imu` 类型不存在；这是 IMU 可视化插件错误，不影响 PointCloud2，但 IMU topic 尚未独立验收。
- 按 Ctrl+C 复现退出 SIGABRT（PID 306909，core 存 systemd-coredump；`coredumpctl debug 306909` 可检查）。GDB：`free(): invalid size` 出现在 exit handler，`spdlog::logger` shared_ptr dispose 位于系统 `libspdlog.so.1`，调用源为 Livox SDK 全局 logger 静态析构；SDK vendored spdlog 为 1.3.1，ROS 系统 spdlog 为 1.9.2。版本/符号冲突是高置信度根因方向；尚未改动或修复第三方源码。
- 独立复核当前没有残留 driver/RViz 进程；本次测试前 Git 工作区干净。`/tmp/mid360s-shutdown-repro/run.log` 保留本次启动输出。02-LIVE 仅点云冒烟测试通过；下一步验证 IMU 与 CustomMsg/短包，处理退出清理缺陷，之后执行正式 31 分钟数据包、离线 QC 和回放。03-LIO 仍不得启动。

## 2026-10-16 · MID-360S 02-LIVE 短包复核及正式实采验收（Codex）

- 用户要求 Codex 执行 02-LIVE 真实数据验收并判断是否通过；按已批准边界仅操作笔记本侧 MID-360S 和 ROS 录包/QC，不启动机器人 host，不使能或移动任何机构，不修改雷达网卡配置。全程只运行一个 Livox driver。
- 修复验收工具的 CustomMsg 顺序判据后，DSH 报告包测试 138 项通过、`git diff --check` 通过。Codex 独立复跑干净 Humble 环境下 `colcon test --packages-select hei_robot_bringup --event-handlers console_direct+`：138 passed，0 errors/failures/skips；独立 `git diff --check` 通过。`offset_time` 逆序依上游发布源码按信息统计，不作失败条件，因点按 UDP 到达次序添加且没有排序保证。
- 58.852 秒稳定真实短包 `/home/robo/hei-data/mid360s_diag_20261016_180730` 在阈值 `duration=30s, max-gap=0.5s, IMU=190–210Hz` 下独立 QC PASS：588 CustomMsg、11,765 IMU，点云约 10Hz，IMU 均值 199.891Hz；结构/解码、时间连续性通过。逐点 offset 统计 90,169 次逆序（INFO）。隔离 ROS_DOMAIN_ID=77 下 2 倍速回放该短包，播放器 exit 0；subscriber 收到 CustomMsg 和 IMU。此回放不代表正式包全量回放。
- 95.6 秒启动期真实短包 `/home/robo/hei-data/mid360s_short_20261016_1803` 独立 QC FAIL：LiDAR/IMU bag 到达间隔分别最多 1.532s/1.447s，header 间隔最多 0.600s/1.400s；CustomMsg 结构合法、offset 逆序仅 INFO，IMU 均值 197.193Hz。说明新判据没有抹掉真实数据缺口。
- 正式 CustomMsg + IMU bag：`/home/robo/hei-data/mid360s_live_20261016_1827`，开始 `18:27:14.753856121`、结束 `18:58:21.665166011`，时长 1866.911s（31:06.9），约 7.1 GiB；18,669 CustomMsg、373,386 IMU。录包器收到 Ctrl+C 后写完缓存并正常关闭。正式 bag QC 完整运行约 35.6 分钟，退出码 1（FAIL），JSON `/home/robo/hei-data/mid360s_live_20261016_1827_qc.json`。
- 正式包通过项：全消息解码成功、0 条 CustomMsg 结构错误，373,383,456 个点可读；IMU 平均 200.001Hz，bag 到达最大 gap 0.021782s、header 最大 gap 0.017810s，双流覆盖约 1866.793s；LiDAR bag 到达最大 gap 0.227s。LiDAR 源 `header.stamp` 倒退 2 次、最大间隔 0.713431755s，超过 0.5s 门限。全包逐点 offset 逆序 2,899,209 次（INFO），最大 offset 702.866013ms；未发现冻结帧。
- 针对异常时刻，对只读 SQLite 时间索引范围查询并解码 60 条相邻 LiDAR 消息：看到 header 间隔 +700.107ms、随后 −600.660ms 回退；随后出现 140,736 点帧（offset 最大 702.866ms），下一帧 header 间隔 +713.432ms。此时间相关性提示帧聚合/队列时序异常，但根因未证实；不可归咎网络或驱动实现。
- driver 在正式录包器关闭后停止；SDK 完成 deinit 后 Livox 子进程以 SIGBUS（exit -7）终止，launch 父进程 exit 0 但日志有 child died。残留进程核查未发现 livox driver、rosbag recorder 或机器人控制进程。异常与此前 Ctrl+C SIGABRT 是不同实际退出信号，本轮根因未修复。
- 最终验收：02-LIVE **NEEDS_FIX / FAIL**，不能进入 03-LIO。已达成真实长时采集、IMU 频率/数据结构、短包回放抽查和完整离线 QC；阻塞项为 LiDAR 源时间倒退/0.5s 间隔门限失败、driver 退出 SIGBUS、正式长包尚未回放。不得通过放宽 QC 阈值隐藏异常。后续需针对采集时钟/驱动队列/帧聚合做诊断，修复或隔离 SDK 清理问题后重跑长时验收。

## 2026-10-17 · MID-360S 时钟/退出修复与正式复测（完成：PASS）

- 用户明确授权继续源时间诊断、退出修复、长时 QC 和正式包完整回放；本轮仅笔记本传感器链路。网络实测雷达 `192.168.1.135` 经 USB `enx9c69d3cd61a9=192.168.1.5/32` 直连，ping 2/2；WiFi 当前 `172.22.22.189/24`，机器人高速口 DOWN，未修改网络/固件/OS 时间同步设置。DSH PID 21932 cwd 已核对正确，3080 UI 实际下发 02-FIX-20261017 并回收执行报告。
- 旧正式 bag 新取证：按 SQLite `messages.id` 插入顺序，IMU id73137→73138 的 source header 与 receive 同时回拨约 598ms；按 timestamp 排序会制造额外 LiDAR 回退、掩盖 IMU 回拨。driver NoSync 原实现同时用 `high_resolution_clock`（本机为 CLOCK_REALTIME）打戳和划帧；系统时钟回拨导致时间回退与发布调度等待追平。140,736 点/702.866ms 大帧与该等待吻合；大帧后的 713.432ms source gap 在插入序也存在，不能全部归为排序伪影。谁调整了系统时钟尚无旧 journal 证据，不能直接归因 NTP。当前被动 UDP 读到两路 `time_type=0`，读完已释放端口。
- Codex 无硬件因果实验：仅 dlopen SDK + `LivoxLidarSdkInit(nullptr,nullptr,nullptr)`，源码在初始化 logger 后返回 false，未创建设备/套接字。原库单独 3/3 exit0，预加载 system spdlog 后 3/3 SIGBUS(-7)；符号隔离后的 SDK 两组合均 3/3 exit0（共12次）。56 个 C API 导出集合完全一致，内部 spdlog/fmt/livox C++ 导出消除；实际 driver 组件 ldd 绑定本项目修复 SDK，ROS 后端仍使用系统 spdlog。
- DSH 实施 tracked SDK 导出隔离 patch、NoSync epoch 锚点 + steady elapsed / steady 帧调度 patch，PTP/GPS 原路径保留；构建脚本支持冻结 SHA + 精确 patch，未知 tracked/untracked 变更在 apply/revert 前拒绝，提供临时 Git 状态机测试。Codex Review 要求修正首次 anchor 读取顺序、map 的 LINK_DEPENDS、拒绝前不修改工作区、证据中真实 gap 与排序伪影的区分，已复审关闭。NoSync 锚点是主机近似 epoch，不证明设备硬件同步或多传感器对时。
- RF-LIDAR-006：Codex 发现 bag_qc 原先接收/打印 lidar_hz_min/max 但缺少 `lidar.rate` 判定，明确扩大白名单由 DSH 最小增补，并加5Hz/20Hz失败、9/11Hz边界和CustomMsg低频合成测试。原门限不变；旧正式 FAIL 结论保留。Codex 独立系统 Python/三层 ROS 环境 pytest：143 passed in5.64s，无skip；独立 underlay regression、git diff --check 通过。DSH 另报告干净→apply→build 与重复构建成功，产物已冻结。
- Codex 实际 driver 短测：CustomMsg→PointCloud2→CustomMsg 各15s，3次均收两路数据且 SIGINT 正常 exit0，无升级停止。初始几秒存在历史启动缓存/源时间大间隔，记录未隐藏；这3次只验收生命周期，不据此宣称数据合格。正式采集增加10s稳定预热并单独保留 warmup_streams，再启动完整1865s监测/录包窗口。
- 正式复测 RUNNING：bag `/home/robo/hei-data/mid360s_live_20261017_retest01`；ROS_DOMAIN_ID=86、ROS_LOCALHOST_ONLY=1，单 driver。独立 live monitor 采用 raw subscription 统计两流 header 顺序/间隔及 monotonic 到达间隔，并测 CLOCK_REALTIME 与 monotonic 差分，避免只依赖 rosbag timestamp 排序。全部运行/原始源码diff备份/哈希/预检/短测证据在 `/home/robo/hei-data/mid360s_retest_20261017_evidence/`；DSH交付在 `docs/agent/evidence/mid360s/20261017-fix/`。全量 QC 与正式包回放尚未执行，此时不可判定 02-LIVE DONE。
- 采集收尾：正式bag开始16:10:01.298816203、结束16:41:06.056230792，1864.757414589s；18,648 CustomMsg、372,964 IMU，391,612条消息，7.1GiB，DB文件7,628,042,240字节。独立live monitor整个窗口18,650帧/373,013 IMU（包含录包边界之外的少量回调，包内计数以metadata为准），header倒退/重复0，max source gap0.111438554s/0.016172456s；主机clock差分200ms采样未见>5ms步进。recorder和driver均exit0、无需升级停止。原始DB3 SHA256 `4ea1bdeb48a0a77637bd4a4d64ad9c5b8d7a1cf401cd93b8b24208c45f6afdc5`，完整清单 `data_artifact_manifest.json`。
- 追加生命周期验证：实际日常 `ros2 launch hei_robot_bringup mid360s_bringup.launch.py ... pointcloud_format:=pointcloud2 launch_rviz:=false` 运行12s后SIGINT；launch parent exit0，child `process has finished cleanly`，SDK deinit/析构完整、无child died。初版验证器误匹配小写pointcloud2导致结果FAIL；Codex依据未改动原始日志中的`PointCloud2 format`纠正大小写匹配并保留初版结果，修正后PASS；未重跑硬件或修改driver。证据在 `production_launch_smoke/`。
- 独立进程级时钟故障注入：拦截当前测试进程的clock_gettime，仅偏移CLOCK_REALTIME返回值；真实production `host_time.h` 在回拨约590ms/前跳约1310ms时仍只递增约10ms。未修改OS时间或采集进程。DSH将此证据纳入 `test_process_clock_steps.cpp` 并接入underlay回归，probe改用SONAME以移除x86路径硬编码；Codex独立Review+重跑扩展suite PASS，构建产物/patch哈希未变；ARM64未实测。
- 完整QA启动：生产bag_qc原代码、原阈值（1800s/9–11Hz/190–210Hz/gap0.5s/全量）全包执行，外部wrapper只调用原收集函数后打印已有计数，不改变结果或门限；输出 `/home/robo/hei-data/mid360s_live_20261017_retest01_qc.log` 和同名 `_qc.json`（完成后生成）。正式2倍回放在Domain87/localhost隔离进行，先扫描全量原始CDR形成每话题计数及串接SHA256，再由raw订阅器核对实际传输的每条消息序列；短包对照已PASS（588/11765条、哈希全一致），正式回放仍待结果。
- 正式回放结果PASS：player exit0，完整播放1864.757s原始包、2倍速，接收验证阶段wall约941.6s（含3s发现等待和5s末尾排空）。18,648条CustomMsg/372,964条IMU全部收到；bytes数、计数、完整长度前缀CDR序列SHA256均与原包一致：点云 `928fecb6052555b2542f4abc5f5513e5ad20dcf881deaa34ffa0e84ae147c839`，IMU `1cea3e5afb554b4636a4c9ad999f6dfda1abec93a16277aeb92dd3748a76db52`。两路回放header倒退/重复0。证据 `formal_replay01/result.json`；无订阅丢失/重排造成的内容差异。02-REPLAY DONE，逐点QC仍在执行，02-LIVE总体尚不能DONE。

- 全量QC完成：exit0 / PASS，30项中27 PASS、3 INFO、0 FAIL；18,648帧/372,964 IMU全部解码，372,972,288点全部检查。点云10.000Hz、IMU200.006Hz；两路header倒退/重复0；最大接收gap0.111917994s/0.018015592s，最大源gap0.111438554s/0.016172456s，双流源覆盖1864.653s。未采样、未放宽原门限；逐点offset到达顺序INFO不等于时间戳倒退。
- 最终独立交叉核对：metadata、QC、capture、完整回放计数一致；两流全量CDR字节数/序列SHA256一致；冻结构建产物及patch哈希未变；原六份dirty基线均可重建且哈希一致。143项测试、扩展underlay suite与5次实际driver正常退出证据归档。补丁目录局部.gitattributes仅豁免统一diff格式必需的空白上下文，应用后源码仍执行diff --check。
- 验收结论：02-FIX、02-RETEST、02-REPLAY与02-LIVE数值数据/生命周期验收DONE / PASS。详见 [验收索引](evidence/mid360s/20261017-acceptance/README.md) 与 codex_review.json。旧失败包保留；不声称完成硬件精确对时、物理标定、AGX或LIO/SLAM。未commit/push，未改网络/OS时间/机器人控制。

## 2026-10-17 · 03-LIO 开始（进行中）

- 用户授权开始IMU单位/轴向/外参与FAST-LIO2接入。预检分支feat/mid360s-lio、工作区干净；本地DSH PID21932及项目cwd核验，UI新会话下发结构化白名单任务并看到进行中。
- 先审计FAST-LIO真实消费者（加速度归一化、逐点时间、外参方向、TF），再实现离线接入；不启动硬件/机器人host、不改原包或已验收Livox产物。Codex维护台账，DSH只写任务白名单。

- 重派更正：首轮因DSH provider anthropic无adapter而失败，界面进行中并不代表实际运行；该轮未实施。用户修正配置后重派到当前DeepSeek，已看到实际read/command调用。原有dirty仅Codex台账。官方S手册已下载外部数据目录，第22页IMU同向、r_LI=[11,23.29,-44.12]mm；t_IL需取反，名义值不等于现场标定。

- 独立Review修复点：帧尾由末点改保留点max、冗余pcl_ros改显式tf2_ros依赖、patch未知dirty拒绝/子模块SHA干净校验/稀疏安全回退/编译并发限制、审计计时器API、NaN/Inf启动参数、startup测试孤儿清理、运行期异常传播。
- DSH smoke退出harness失败保留：后台shell继承SIG_IGN、setsid不重置；Codex从/proc确认SIGINT被忽略，有界TERM清理03/04轮。改用Python Popen显式恢复SIG_DFL，仅向launch父发一次SIGINT。最终独立27秒wall窗口得到420条odom（41.895s源时间），两路各419帧（56,970+792,839点XYZ全有限）；TF仅camera_init→body；launch exit0，3个child cleanly，无escalation，运行产物hash未变。关停context无效后审计只记录取消息告警，运行异常仍传播。原数据未改。
- Codex独立回归：bringup143 passed/8.41s；localization51 passed/5.77s，无skip。最初两个目录一起pytest因同名test_launch_static导入冲突，改各包独立进程后通过，不改测试判据。最终方向字段报告语义修正待补测。

- 最终补测：报告API重命名比力/up方向并显式给重力负向，补符号测试；初次pytest使用旧非symlink安装副本而失败，重建Python接入包后完整52 passed/5.69s。未改FAST-LIO二进制/launch/参数，无需重复长时采集。根目录/ROS工作空间入口索引更新当前阶段，旧架构快照保留。03-LIO-BASE DONE、03-LIO PARTIAL。

## 2026-10-19 · HEI-LIDAR-04-SLAM-LAPTOP-BASE-20261019

用户授权电脑端FAST-LIO + RTAB-Map完整三维SLAM软件部署。Codex已通过3080新会话提交含14项约束的结构化提示，看到DSH实际执行Git、ROS、apt依赖检查；PID15200 cwd=/home/robo/robotopia/hei-rebot-lift。现有分支feat/mid360s-lio @1aeb8a1，已有LIO改动保留；Codex备份dirty哈希及tracked diff至/home/robo/hei-data/mid360s_slam_20261019/codex_baseline/。RTAB-Map尚未安装，sudo非交互不可用（DSH报告），依赖部署仍在调查，不声称部署通过。03-LIO桌面动态全包验证有2331条有效odom；不等同现场30–50米闭环精度验收。04总体未DONE，本次仅软件子任务。

### 2026-10-19 · 04-SLAM 电脑端终审完成

DSH已交付hei_robot_slam与RTAB-Map 0.23.13独立本地前缀依赖，Codex独立Review并要求修正R01–R10。特别核查无图像one-to-one proximity默认空guess，按0.23.13-humble源码25d2135与A/B实测固化RGBD/ProximityOdomGuess=true，注册/优化阈值保留；静态/数据/生命周期回归独立87 passed（10.11s，0 skipped）。原始输出：/home/robo/hei-data/mid360s_slam_20261019/codex_final_pytest.log/xml。

旧参数Codex独立完整回放：2335原始云/46708IMU全部收到、2331输出帧逐帧XYZI哈希相同，52节点/14空间约束；作为旧配置基线保留。新参数完整回放：mapping_proxguess_20261019_121802，2331有效输出、55地图帧/221轨迹、TF历史查询116/116；独立SQLite55节点/55扫描、25独立type2约束、integrity ok、视觉告警0。DB SHA=b30fbaecb9d31c273808e4bff2e1a77bba396c2a1bf138207ee0b356a06356b3。新正式导出48855个有限XYZ/55姿态；Codex独立解析并生成codex_final_map_preview.png。旧库A2只读定位36条/info中9次proximity接受、初始位姿被消费、原库SHA不变；不称三位置现场重定位通过。

执行异常如实保留：harness取消并不终止独立session中的launch。Codex核对PID84665/PGID后接管，观察bag player正常EOF再只给父INT一次，5子节点clean、无升级/残留；父wait返回码不可得，原harness不能称clean。另一次预创建session导致启动被正确拒绝，等待空订阅的initialpose发布器由Codex精确中断；后续采用有界发布并检查订阅。接管原始记录codex_final_mapping_supervision.json。

依赖独立verify、全新外部prefix默认安装与拒绝覆盖均通过；安装脚本不改系统源/全局环境。当前feat/mid360s-lio @1aeb8a1，未commit/push，未运行实机控制。软件子任务DONE，04总体PARTIAL，现场30–50m精度、3位置重定位、live/RViz人工目视/AGX留待下一批。仓库索引：docs/agent/evidence/mid360s/20261019-slam/codex_final_acceptance.json。

### 2026-10-19 · 实时地图保存与只读定位入口补充

- 用户希望把实时建图结果放在 `extensions/ros2_ws/src/hei_robot_slam/map/`，并用同一实机雷达在 RViz 查看已建地图上的定位。检查确认 map 目录此前不存在；新增 `.gitignore` 排除地图与导出数据。建图通过 `output_root:=.../map` + 每轮唯一 `session` 写入，不改通用数据根目录。
- 发现原 `slam_localization.launch.py` 需要 rosbag，只能离线回放。新增 `slam_live_localization.launch.py`：真实雷达硬件和未标定外参双 opt-in、Domain 89/localhost 隔离、指定已存在数据库、复用 FAST-LIO 名义参数校验与 XYZI 适配、RTAB-Map 定位参数强制只读、RViz 可选；启动信息记录数据库 SHA256，且不启动底盘/机械臂/升降。
- 验证：`colcon build --packages-select hei_robot_slam` 通过；新 launch `--show-args` 通过；全包 pytest **89 passed**；`git diff --check` 通过；`git check-ignore --no-index` 确认 map 子目录数据库会被忽略。未启动真实雷达/算法节点，未验证实机地图保存、初始位姿消费、定位成功率或 DB 运行前后哈希；仍待用户现场执行。
- 未 commit/push，未切分支；原有 LIO 与 SLAM 实测产物保留。操作命令已补充到 `extensions/ros2_ws/src/hei_robot_slam/README.md` 与 `docs/MID360S_SLAM.md`。

### 2026-10-19 · 用户报告的实时定位目视现象（未独立验收）

- 用户报告已用实时雷达建图、加载同一地图观察定位；雷达放在接近建图初始位置/朝向时，RViz 画面看似与地图对齐；将偏航相对初始朝向转约 90° 后，画面明显错位/漂移，用户判断重定位失败。
- 证据边界：这是用户操作后的目视描述/截图，不含本次运行的 `/info` 接受统计、同步原始日志、数据库哈希前后对照或独立测量误差；Codex 未在本轮接管该实时会话。因此只记录为有限现场观察，不判断具体根因，也不标记 04-SLAM 现场定位通过。
- 结论：可继续做固定安装和导航前的软件集成规划，但定位鲁棒性、不同初始朝向重定位、外参与精度仍须用可复核数据单独验收；雷达尚未固定到机器人，AGX部署、桥接和导航仍按任务板保持 PLANNED。

### 2026-10-20 · HEI-SLAM-OBS-20261020 派发

- 核对分支及干净工作区；3080 DSH 页面空闲后提交14字段任务，UI已显示运行中。实施只限hei_robot_slam监测/测试/README及新证据目录。
- 运行中的定位链路保留，禁止DSH占用Domain89；测试隔离Domain91。监测修复不等同定位精度验收。

### 2026-10-20 · HEI-SLAM-OBS-20261020 独立审查与验收

- 已纠正候选link日志误算accepted、状态键尾斜杠、QoS深度位置参数、关闭落盘顺序、初值尝试与历史区分、缺失ID未知及零优化次数计数；DSH实施并冻结。
- 独立全量122 tests通过（Domain92）；最后优化帧计数修正后独立32 tests通过，DSH最终全量123 tests通过。历史真实数据重分析保留9次接受，不作为本轮现场定位通过。
- 只读monitor现场接收108条/info并正常SIGINT关闭，final=true/closed=true/写错误0；最终版monitor保持运行，目录见codex_acceptance.json，原算法节点未重启、原地图SHA一致。
- 未执行新的初值偏差恢复/机器人动作/精度测试，未提交推送。

### 2026-10-20 · 用户确认功能测试并关闭现场

- 单次SIGINT关闭定位launch81476和RViz81608，父会话exit0；随后停止只读monitor81576并确认final/closed=true、write_errors=0。无算法/GUI残留，数据库SHA与定位前一致、SQLite integrity ok/40节点。
- 最终连续记录：81接受帧/2076Info/2初值/2拒绝。原始shutdown_review.json在/home/robo/hei-data/localization_observability/yaw180_20261020_155411/。
- 已下发14字段DSH总结任务，仅允许docs/agent/MID360S_SLAM_TEST_SUMMARY_20261020.md；小范围功能通过与精度/AGX/全局重定位边界分开。监测摘要ID字段缺陷如实列待办，未擅自扩展为代码修复。

- DSH已交付MID360S_SLAM_TEST_SUMMARY_20261020.md；Codex逐段核对关键计数、实际run并列路径、测试版本、现场与synthetic来源及已知小缺陷。纠正将15:47/15:49现场测试误判为metric修复前的时序错误后，审查通过；Markdown本地链接全存在、git diff --check通过。未commit/push。
