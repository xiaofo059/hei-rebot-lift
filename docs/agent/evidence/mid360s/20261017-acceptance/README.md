# MID-360S 02-LIVE 独立验收（2026-10-17）

结论：**PASS / DONE**。笔记本 DSH 实施修复，Codex 独立审查、真实采集、全量 QC 和完整回放；范围限笔记本传感器数据与正常启停。

| 项目 | 实际结果 |
| --- | --- |
| 正式采集 | 10秒预热后1864.757414589秒，约31分05秒；7.1GiB |
| 消息与点 | 18,648 CustomMsg、372,964 IMU；检查372,972,288点 |
| 全量 QC | exit0；27 PASS、3 INFO、0 FAIL；未采样 |
| 平均频率 | 点云10.000Hz、IMU200.006Hz |
| header 倒退/重复 | 两路均0 |
| 最大源间隔 | 点云0.111438554秒、IMU0.016172456秒 |
| 最大接收间隔 | 点云0.111917994秒、IMU0.018015592秒 |
| 双流源时间覆盖 | 1864.653秒，超过1800秒门槛 |
| 完整回放 | 原包完整2倍速；两路消息计数、字节数、全序列CDR SHA256完全一致；player exit0 |
| 软件测试 | 143 passed；独立underlay回归、进程内时钟跳变注入通过 |
| 实际驱动退出 | 3次短测、正式采集、日常ros2 launch共5次正常退出 |

## 原因与修复

- NoSync 原实现用主机 realtime 打戳/划帧；旧包按SQLite插入序证实主机/源时间同时回拨约598ms。现用共同epoch锚点加steady elapsed，并用steady划帧。旧大帧之后713.432ms源间隔是真实异常，未归咎于全部排序伪影。调整旧系统时间的具体来源未确认。
- SDK内置spdlog与ROS系统spdlog符号冲突导致退出崩溃；独立12次加载顺序A/B可重现。SDK现只导出56项公共C API，正常析构保留。
- 补齐QC原先遗漏的点云频率判据及5项回归；9–11Hz、IMU190–210Hz、gap≤0.5秒、覆盖≥1800秒门限不变。
- 修复补丁/构建说明见 [patches README](../../../../../extensions/dependencies/patches/README.md)，取证见 [DSH交付](../20261017-fix/README.md)。

## 验收证据

- [独立Review及限制](codex_review.json)
- [正式全量QC](formal_qc.json)
- [正式完整回放](formal_replay.json)
- [实际采集/退出](formal_capture.json)
- [原包与构建哈希清单](data_artifact_manifest.json)
- [143项测试](codex_pytest.log)、[underlay回归](codex_underlay_regression.log)
- [原工作区保护审计](preservation_audit.txt)（仓库副本仅去除行尾空白；外部原件保持不变）

原始bag：`/home/robo/hei-data/mid360s_live_20261017_retest01`。
完整原始日志、脚本、旧包取证及A/B结果：`/home/robo/hei-data/mid360s_retest_20261017_evidence/`。
QC完整输出：`/home/robo/hei-data/mid360s_live_20261017_retest01_qc.json`及同前缀`_qc.log`。
QC参数：`--pointcloud-format custom --expected-duration 1800 --lidar-hz-min 9 --lidar-hz-max 11 --imu-hz-min 190 --imu-hz-max 210 --max-gap 0.5`；外部wrapper只打印进度并调用生产collector，无采样或改写判据。

## 边界与交接

旧失败包、启动期瞬态均原样保留；正式窗口在显式10秒预热后录制。NoSync host锚点不代表硬件PTP/GPS精确同步。原IMU加速度单位为g且orientation无效，进入LIO前需完成单位、外参、轴向核验。固件/线束/供电记录与实际IMU配置键待补齐；AGX/ARM64、LIO、SLAM、导航均未执行。

Git分支`feat/mid360s-bringup`，验收基线HEAD `687328412ab20953c10acabb7c2add098faa44de`。保留原dirty修改，未commit/push；未改网络、OS时钟或机器人控制。
