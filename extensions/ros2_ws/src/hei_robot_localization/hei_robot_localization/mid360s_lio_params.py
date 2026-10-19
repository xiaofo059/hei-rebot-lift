"""MID-360S 离线 FAST-LIO2 参数集：生成、校验与“未标定”门控。

参数取值逐项对应本轮源码审计（行号见 lio_contract.py 顶部）。这里只有**显式**决定，
不复制上游 yaml 的全部默认值；未列出的键沿用上游默认。

关键决定
--------
* preprocess.lidar_type = 1 (AVIA)：FAST-LIO 只有在 lidar_type==AVIA 时才订阅
  livox_ros_driver2/CustomMsg（laserMapping.cpp:921-929），MID-360S 的原始包正是
  CustomMsg。写 4 (MID360) 会改成订阅 PointCloud2，与本包输入不符。
* preprocess.scan_line = 4：driver 对 MID-360 系列固定 line = i % 4
  （pub_handler.cpp:402 + comm.h:82 kLineNumberMid360=4）；avia_handler 用
  line < N_SCANS 过筛，取 4 才不会丢点（实测 4 条线各占 1/4）。
* common.time_sync_en = false：正式包的 NoSync 时间是“主机 epoch 锚点 + steady
  已过时间”的共同映射，不是硬件 PTP；打开 time_sync_en 会让 FAST-LIO 自己**估**
  一个 lidar-IMU 偏置（laserMapping.cpp:333-338），等于凭空造同步。
* common.time_offset_lidar_to_imu = 0.0：没有独立标定的 lidar-IMU 时间偏置，不填假值。
* mapping.extrinsic_T/R：厂商名义值（手册 §3.5 + 上游 config/mid360.yaml 一致），
  **未经现场标定**；因此 launch 要求显式 uncalibrated:=true 并记录该状态。
* mapping.extrinsic_est_en = false：未标定时不允许在线估计外参把不确定性“吃掉”，
  否则结果无法区分“标定好了”与“滤波器把外参漂过去了”。
* pcd_save.pcd_save_en = false：离线冒烟不写点云大文件。
"""

from __future__ import annotations

from pathlib import Path

from hei_robot_localization import lio_contract as C

#: 本实验使用的完整参数（键为 FAST-LIO 的参数名）
FASTLIO_PARAMS: dict = {
    "feature_extract_enable": False,
    "point_filter_num": 3,
    "max_iteration": 3,
    "filter_size_surf": 0.5,
    "filter_size_map": 0.5,
    "cube_side_length": 1000.0,
    "runtime_pos_log_enable": False,
    "map_file_path": "./test.pcd",
    "common": {
        "lid_topic": "/livox/lidar",
        "imu_topic": "/livox/imu",
        "time_sync_en": False,
        "time_offset_lidar_to_imu": 0.0,
    },
    "preprocess": {
        # 1 = AVIA：只有这个取值会让 FAST-LIO 订阅 CustomMsg
        "lidar_type": 1,
        "scan_line": 4,
        "blind": 0.5,
        # 仅对 PointCloud2 处理路径生效；CustomMsg 路径按 ns/1e6 直接得到 ms
        "timestamp_unit": 3,
        "scan_rate": 10,
    },
    "mapping": {
        "acc_cov": 0.1,
        "gyr_cov": 0.1,
        "b_acc_cov": 0.0001,
        "b_gyr_cov": 0.0001,
        "fov_degree": 360.0,
        "det_range": 100.0,
        # 未标定：禁止在线估计外参
        "extrinsic_est_en": False,
        "extrinsic_T": list(C.FASTLIO_NOMINAL_EXTRINSIC_T),
        "extrinsic_R": list(C.FASTLIO_NOMINAL_EXTRINSIC_R),
    },
    "publish": {
        "path_en": True,
        "effect_map_en": False,
        "map_en": True,
        "scan_publish_en": True,
        "dense_publish_en": False,
        "scan_bodyframe_pub_en": True,
    },
    "pcd_save": {
        "pcd_save_en": False,
        "interval": -1,
    },
}

#: 实验元数据：进入 launch 日志与审计报告，避免“跑通=已标定”的误读
EXPERIMENT_METADATA: dict = {
    "input": "MID-360S 原始 rosbag（CustomMsg + sensor_msgs/Imu，未做单位换算）",
    "extrinsic_status": "UNCALIBRATED_NOMINAL",
    "extrinsic_source": (
        "Livox Mid-360S User Manual 2026-06-01 §3.5 p.22 的 IMU 芯片位置 "
        "[11.0, 23.29, -44.12] mm（点云坐标）+ 同段“IMU 三轴与点云坐标同向”，"
        "经 p_I = R_IL p_L + t_IL、R_IL = I 得 t_IL = [-0.011, -0.02329, +0.04412] m；"
        "与上游 config/mid360.yaml 的 extrinsic_T 一致（交叉核对，不是来源）"
    ),
    "imu_acc_unit": "g（driver 冻结源码自述 + 本包短窗口实测范数≈0.995）",
    "imu_gyro_unit": "rad/s",
    "imu_orientation": "无效（driver 未发布，消息字段恒为默认值）",
    "acc_conversion": "无（FAST-LIO IMU_Processing.hpp:260 自行归一化）",
    "time_sync": "NoSync（主机 epoch 锚点 + steady 已过时间），非硬件 PTP",
    "tf_contract": (
        "唯一发布者 fastlio_mapping；camera_init -> body。"
        "本包不发布任何 TF，不发布 base_link 或 map。"
    ),
}

#: 允许的 lidar_type 取值（本实验只接受 CustomMsg 路径）
_ALLOWED_LIDAR_TYPE = 1


def get_param(params: dict, dotted: str):
    """按 a.b.c 取值。"""
    cur = params
    for key in dotted.split("."):
        if not isinstance(cur, dict) or key not in cur:
            raise KeyError(dotted)
        cur = cur[key]
    return cur


def validate(params: dict) -> list[str]:
    """返回问题列表；空列表视为通过。"""
    problems: list[str] = []

    def expect(dotted: str, wanted, why: str) -> None:
        try:
            got = get_param(params, dotted)
        except KeyError:
            problems.append(f"缺少参数 {dotted}（{why}）")
            return
        if got != wanted:
            problems.append(f"参数 {dotted}={got!r} 与实验契约 {wanted!r} 不符（{why}）")

    expect("preprocess.lidar_type", _ALLOWED_LIDAR_TYPE, "只有 AVIA 会订阅 CustomMsg")
    expect("preprocess.scan_line", 4, "driver 对 MID-360 系列 line = i % 4")
    expect("common.time_sync_en", False, "NoSync 不能靠软件猜偏置")
    expect("common.time_offset_lidar_to_imu", 0.0, "没有独立标定的时间偏置")
    expect("common.lid_topic", "/livox/lidar", "原始包话题")
    expect("common.imu_topic", "/livox/imu", "原始包话题")
    expect("feature_extract_enable", False, "特征提取路径不适用于 4 线 MID-360S")
    expect("mapping.extrinsic_est_en", False, "未标定不允许在线估计外参")

    try:
        t_il = get_param(params, "mapping.extrinsic_T")
        r_il_flat = get_param(params, "mapping.extrinsic_R")
    except KeyError:
        t_il = None
        r_il_flat = None
    if t_il is not None:
        if len(t_il) != 3:
            problems.append("mapping.extrinsic_T 必须是 3 个元素")
        else:
            want = list(C.FASTLIO_NOMINAL_EXTRINSIC_T)
            if max(abs(float(t_il[i]) - want[i]) for i in range(3)) > 1e-9:
                problems.append(
                    "mapping.extrinsic_T=" + repr(list(t_il)) + " 与手册推导的名义值 "
                    + repr(want) + " 不符（注意 t_IL = -R_IL r_LI，符号不能照抄芯片坐标）"
                )
    if r_il_flat is not None:
        if len(r_il_flat) != 9:
            problems.append("mapping.extrinsic_R 必须是 9 个元素（行优先）")
        else:
            want = list(C.FASTLIO_NOMINAL_EXTRINSIC_R)
            if max(abs(float(r_il_flat[i]) - want[i]) for i in range(9)) > 1e-9:
                problems.append(
                    "mapping.extrinsic_R 不是手册所述“同向”的单位阵；"
                    "若确有现场标定结果，请另建参数文件并注明来源"
                )
    return problems


def load_and_validate(path: str | Path) -> dict:
    """读取 yaml 并校验；不合法时抛 ValueError（信息可直接给人看）。"""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - Humble 自带 pyyaml
        raise ValueError("缺少 PyYAML，无法读取参数文件") from exc
    p = Path(path)
    if not p.is_file():
        raise ValueError(f"参数文件不存在：{p}")
    with p.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"{p} 不是 YAML 映射")
    node_cfg = data.get("/**")
    if not isinstance(node_cfg, dict) or "ros__parameters" not in node_cfg:
        raise ValueError(f"{p} 缺少 /**/ros__parameters 结构（FAST-LIO launch 直接把它当参数文件）")
    params = node_cfg["ros__parameters"]
    problems = validate(params)
    if problems:
        raise ValueError("参数文件不符合实验契约：\n  - " + "\n  - ".join(problems))
    return params


def launch_notice() -> str:
    """launch 启动时打印的契约摘要（也进审计报告）。"""
    lines = [
        "[hei_robot_localization] 离线 LIO 实验契约（未标定，仅离线）",
        "  外参        : " + EXPERIMENT_METADATA["extrinsic_status"],
        "  来源        : " + EXPERIMENT_METADATA["extrinsic_source"],
        "  IMU 单位    : acc=g / gyro=rad/s / orientation 无效",
        "  单位换算    : " + EXPERIMENT_METADATA["acc_conversion"],
        "  时间同步    : " + EXPERIMENT_METADATA["time_sync"],
        "  TF          : " + EXPERIMENT_METADATA["tf_contract"],
        "  未知项      : " + "；".join(C.KNOWN_UNKNOWNS[:3]),
    ]
    return "\n".join(lines)
