# AGX 静止建图诊断（2026-10-22）

## 范围
在 AGX /home/user/hei-rebot-lift 执行 slam_live_mapping.launch.py，ROS_DOMAIN_ID=89、ROS_LOCALHOST_ONLY=0、rviz=false；沿用现场雷达配置，未改网络、标定、参数或算法源码。按用户提供的雷达静止条件解释结果；本次没有独立视觉运动真值。

## 实测
| 测量窗口 | FAST-LIO 最大相对位移 | 最大相对转角 | 里程计时间戳延迟中位数 | 适配点云延迟中位数 |
| --- | --- | --- | --- | --- |
| 第一轮，去除采样前5秒后的50秒 | 7.03 cm | 1.16 deg | 1.357 s | 1.386 s |
| 第二轮稳定运行，探针关闭，25秒 | 2.82 cm | 0.59 deg | 0.0245 s | 0.0529 s |
| 同一第二轮进程，开启探针，25秒 | 1.80 cm | 0.45 deg | 0.0253 s | 0.0534 s |

所有窗口 map->camera_init 均为单位变换、没有变化；camera_init->body 与 /Odometry 同步变化。/tf 只有 FAST-LIO 和 RTAB-Map 两个发布者，分别负责两条边，没有发现重复同边发布证据。第一轮点云接近10 Hz，IMU接近200 Hz，无需电脑RViz即可测到前端漂移。不同轮次窗口和启动状态不同，不能将漂移差异归因于探针。

## 结论
1. 静止位姿漂移发生在 FAST-LIO 前端，并被传给 RTAB-Map；本次不是 RTAB-Map map->camera_init 校正跳变。
2. TF extrapolation 表示请求点云时间晚于收到的最新 camera_init->body，属于时间配对/到达顺序问题；该警告本身不能证明真实运动，也不能解释全部前端漂移。
3. 第一轮存在持续约1.3秒的本机前端滞后，第二轮稳态没有复现。同一进程开探针对比未增加延迟，故“探针必然导致延迟”假设不成立。第一轮未同时测原始CustomMsg，不能精确定位积压发生在哪个队列。
4. 当前LIO使用0.5m surf/map滤波、厂商名义未标定外参；IMU均值/噪声、几何退化、初始估计及时间对齐是候选排查方向，尚未逐项证实。不能把其中一项写成已确认根因。

## 建议修复顺序
- 同时录制原始/livox/lidar、/livox/imu和输出/Odometry、/tf，先在AGX本机复现与离线回放，记录接收时刻、源时间戳、帧跨度、漂移和处理耗时。
- 排查SDK/driver/Fast-LIO输入积压及TF到达顺序；若需调整执行器/订阅队列，应先验证前后端同一帧时间戳匹配。不能任意平移时间戳、固定body TF或仅扩大wait_for_transform来掩盖。
- 在同一份bag上对比适合室内尺度的体素/有效特征设置，结合IMU初始化和标定验证判断改善；不直接修改未验证的外参或IMU单位。
- Wi-Fi可视化单独做降频/降采样话题，保留AGX本机算法原始数据链路。

## 证据
已提交 `static_probe.json`、`static_no_report.json`、`static_report_on.json` 和 `raw_timing.json`。详细 ROS 日志 `static_mapping.log`、`raw_mapping.log` 保留在本机工作区，按日志忽略规则不纳入 Git。原始数据采样脚本短时加入订阅可能影响对应窗口负载；稳态取样单独进行。测试输出保存在 AGX 仓库 `map/static_diagnosis_20261022_144023` 和 `map/raw_timing_20261022_144248`。本次试验完成后停止自己创建的节点，未修复算法或宣称建图质量通过。
