# 正式 bag 源时间异常取证（insertion-id vs timestamp-sorted）

数据：`/home/robo/hei-data/mid360s_live_20261016_1827`（只读；1866.911 s，
18,669 CustomMsg / 373,386 IMU）。以下全部用 SQLite 只读查询 + 手写 CDR header 解析，
未跑全量 Python QC。

## 1. 设备时间类型（被动 UDP，Codex 采集，未启动 driver）

`/home/robo/hei-data/mid360s_retest_20261017_evidence/passive_udp_both.json`：

- 点云端口 56301：`time_type=0`（NoSync），两包 `raw_timestamp` 间隔 480,000 ns；
- IMU 端口 56401：`time_type=0`（NoSync）。

⇒ driver 走 NoSync 分支：`GetEthPacketTimestamp()` 用主机时钟替代设备时间，
`CheckTimer()` 用主机时钟做非同步帧调度。

## 2. 按插入顺序（messages.id）核对 header / receive

SQL（只取 id 区间，不扫全表；header 只需 CDR 前 ~20 字节）：

```sql
SELECT id, topic_id, timestamp, substr(data,1,96)
  FROM messages WHERE id BETWEEN 73040 AND 73320 ORDER BY id;
```

关键行（recv=bag 记录的 receive 时间，hdr=消息 header.stamp）：

| id | topic | recv (ns) | header (ns) | 说明 |
| --- | --- | --- | --- | --- |
| 73137 | IMU | 1792146783542791479 | 1792146783542563514 | 回拨前最后一条 |
| 73138 | IMU | 1792146782944476521 | 1792146782944256898 | **两者同时 -598.307 ms** |
| 73139 | LiDAR | 1792146782945120402 | 1792146783448337921 | 帧在回拨前发布、回拨后入 bag |
| 73285 | LiDAR | 1792146783674725645 | 1792146782944049438 | 巨帧：point_num=140736，offset max=702.866013 ms，timebase=header |
| 73301 | LiDAR | 1792146783747710127 | 1792146783657481193 | 巨帧之后：header **+713.432 ms** |

按 id 顺序的 header 差分：

- IMU：**恰好 1 次**负差分 = -598.307 ms（id 73138）→ 主机 CLOCK_REALTIME 回拨；
- LiDAR：巨帧 id 73285 相对上一真实帧(id 73139) **-504.288 ms**（巨帧 timebase 是积压缓冲首点），
  随后 id 73301 相对巨帧 **+713.432 ms**（真实前跳）。

⇒ 真实记录顺序下源时间确实发生不连续；原 QC 的 FAIL 方向正确，不能用「读取伪影」一笔勾销。

## 3. 按 bag 时间戳排序（rosbag2 SequentialReader 的读取顺序）

`ORDER BY timestamp, id` 复现出的正是原 QC 的 4 个数字：

| 类型 | id | dH | 真伪 |
| --- | --- | --- | --- |
| 回退 | 73012 | -600.660 ms | **排序伪影**（回拨前的帧，receive 落在回拨后区间） |
| 回退 | 73285 | -402.556 ms | 真实（巨帧 timebase 回退），幅度因邻帧不同 |
| 前跳 | 73139 | +700.107 ms | **排序伪影**（回拨前发布/回拨后入 bag 的帧被排到前面） |
| 前跳 | 73301 | +713.432 ms | 真实（id 顺序同样存在） |

同一排序下 IMU：**0 次回退、0 个 >500 ms gap** → 排序把 IMU 的 -598.307 ms 真实回拨
**掩盖**了。结论：bag 时间戳排序不可当作原始顺序，诊断必须用 `messages.id`。

## 4. 巨帧成因（机制推导，数值自洽）

`CheckTimer()` 非同步分支（修复前）用 `high_resolution_clock`（=CLOCK_REALTIME）：

```cpp
auto now_time = std::chrono::high_resolution_clock::now();
... if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) return;
```

CLOCK_REALTIME 回拨 598 ms 后 `now_time - last_pub_time_` 变成负数 → 发布停摆约 0.598 s，
期间点全部积压；时钟追平后一次性发布一帧。

- 积压时长 = 巨帧 offset span = 702.866 ms；
- MID-360S ~200,000 点/s ⇒ 703 ms × 200k/s ≈ 140,600 点 ≈ 实测 **140,736** 点；
- `timebase` = 缓冲首点时间 ⇒ 比上一真实帧回退 504 ms。

⇒ LiDAR/IMU header 回退 + 巨帧 + 713 ms 前跳是同一个主机时钟回拨事件的两个后果。

## 5. 主机时钟证据与边界

- 诊断时 `timedatectl`：`systemd-timesyncd` active/synchronized，offset +114.668 ms，
  jitter 108.795 ms；昨日 18:30–18:35 日志已不可用 ⇒ **回拨根因未能从系统日志确认**。
- 不关闭 NTP、不改系统时钟配置（本修复不需要）。
- 回拨幅度 ~598 ms 由 id 73137→73138 的 receive 与 header 同步跳变直接给出。
