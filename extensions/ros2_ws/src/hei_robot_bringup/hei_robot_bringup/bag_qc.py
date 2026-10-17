#!/usr/bin/env python3
"""MID-360S 原始 rosbag 离线质量验收。

只做**数据时间基准的数值一致性检查**。它不能证明硬件时间同步精度，
也不能替代现场对点云/IMU 物理正确性的检查；本工具只回答
“这段原始包在话题、类型、数量、频率、断流、时间单调性、消息结构上是否可验收”。

单位口径（driver 1.2.6 冻结源码核对，本工具**不做**换算）：
原始 `sensor_msgs/Imu.linear_acceleration` 单位是 **g**（非 m/s²），
`angular_velocity` 单位是 rad/s，`orientation` 未发布。详见 `mid360s_config.py`。
因此**不能**把原始 Imu 消息当作已符合 SI 的融合输入。

CustomMsg 逐点时间口径（driver 1.2.6 冻结源码核对，2026-10-16）：
  ``CustomPoint.offset_time`` 是相对**本帧首点**（``timebase``）的 **ns** 偏移
  （``src/lddc.cpp:395``：``uint32(points[i].offset_time - pkg.base_time)``，
  ``base_time = points_clouds_.at(0).offset_time`` 即帧内首个**被追加**的点）；
  帧内点的追加顺序 = UDP 包**到达**顺序（``src/comm/pub_handler.cpp:149/240/388-390``，
  全链路无任何排序），包内才按 ``point_interval`` 递增。因此 ``points[]`` **不保证**
  按 ``offset_time`` 有序，"逐点 offset 逆序/回退"不是数据非法判据，只做 INFO 统计。

判定原则（RF-LIDAR-004 修复后）：
  * **两条流都要**满足持续时间、断流、帧头有效/单调/非冻结；
  * 两条流的**源帧头时间范围必须重叠**，且重叠窗口覆盖要求时长（同一采集窗口）；
  * 类型不可导入、反序列化失败、缺帧头、点云/CustomMsg 结构非法 → **FAIL**，不得跳过；
  * CustomMsg **逐点 offset_time 的顺序/分布只做 INFO 统计**（driver 按包到达次序追加、不排序），
    不作 FAIL 判据；**消息级**的 ``header.stamp`` 时间戳判据不受影响，仍全量执行；
  * 帧头时间检查**默认全量**；显式使用 `--header-sample` 主动采样会丢失信息，
    因此只能给 INCONCLUSIVE，永远不能给 PASS；
  * 阈值参数必须是有限正数且上下限次序正确；`--header-sample 0` 等绕过用法直接拒绝。

退出码：
  0 = PASS         全部达标（含已给出的 IMU 频率判据，且未做任何采样）
  1 = FAIL         明确不达标
  2 = 用法或输入错误（路径不存在、阈值非法等）
  3 = INCONCLUSIVE 数据本身无违规，但缺少判定所需输入（如未给 IMU 标称频率，或做了采样）
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from hei_robot_bringup.mid360s_config import (
    CUSTOMMSG_TYPE,
    DEFAULT_IMU_TOPIC,
    DEFAULT_LIDAR_TOPIC,
    IMU_ANGULAR_VELOCITY_UNIT,
    IMU_FRAME_ID_UPSTREAM,
    IMU_LINEAR_ACCELERATION_UNIT,
    IMU_ORIENTATION_PUBLISHED,
    IMU_TYPE,
    POINTCLOUD2_TYPE,
    XFER_FORMAT_BY_NAME,
)

EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_USAGE = 2
EXIT_INCONCLUSIVE = 3

DEFAULT_EXPECTED_DURATION_S = 1800.0
DEFAULT_LIDAR_HZ_MIN = 9.0
DEFAULT_LIDAR_HZ_MAX = 11.0
DEFAULT_MAX_GAP_S = 0.5

NS = 1e-9

#: PointField.datatype -> 字节数（sensor_msgs/PointField）
POINTFIELD_SIZE = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}

#: CustomPoint.msg 声明 ``uint32 offset_time``；这是唯一有依据的上界（ns）
UINT32_MAX = 0xFFFFFFFF

#: 每个流最多记录多少条“结构问题”示例，避免报告爆炸
MAX_PROBLEM_SAMPLES = 5


# --------------------------------------------------------------------------
# 结果模型
# --------------------------------------------------------------------------


@dataclass
class Check:
    check_id: str
    status: str  # PASS / FAIL / INFO
    detail: str


@dataclass
class PointTimeStats:
    """CustomMsg 逐点 offset_time 的**统计**信息（只用于 INFO，不参与 FAIL 判定）。

    依据 driver 1.2.6 冻结源码：点按 UDP 包到达次序追加、全程不排序，
    所以逆序/回退是正常现象；消息类型也没有给 offset_time 定义上界。
    """

    frames: int = 0
    points: int = 0
    #: 相邻点 offset 逆序（后者更小）的总处数
    inversions: int = 0
    #: 出现逆序的帧数
    frames_with_inversions: int = 0
    #: 单次最大回退量（ns）
    worst_regression_ns: int = 0
    #: 全帧 offset 相同的帧数（driver 在 point_interval==0 时合法产生）
    frozen_frames: int = 0
    min_offset_ns: int | None = None
    max_offset_ns: int | None = None


@dataclass
class StreamStats:
    topic: str
    expected_type: str
    found_type: str | None = None
    type_error: str | None = None

    count: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    max_gap_s: float = 0.0
    max_gap_at_s: float | None = None
    regressions: int = 0
    worst_regression_s: float = 0.0

    decode_failures: int = 0
    missing_header: int = 0

    header_checked: int = 0
    #: 相邻帧头时间**倒退**（stamp < prev）的次数
    header_regressions: int = 0
    #: 相邻帧头时间**重复/冻结**（stamp == prev）的次数。
    #: 只统计整段的唯一值集合会漏掉“中段冻结后恢复”（RF-LIDAR-004 残留），
    #: 因此必须逐相邻对检查。
    header_repeats: int = 0
    #: 相邻帧头时间的最大正向间隔（源时间跳变；接收时间可能仍然连续）
    header_max_gap_s: float = 0.0
    header_max_gap_at_ns: int | None = None
    header_first_ns: int | None = None
    header_last_ns: int | None = None

    #: PointCloud2 结构检查
    pc2_struct_bad: int = 0
    pc2_fields: list[str] = field(default_factory=list)
    pc2_problems: list[str] = field(default_factory=list)

    #: CustomMsg 结构检查（FAIL 级）
    custom_bad: int = 0
    custom_points_checked: int = 0
    custom_problems: list[str] = field(default_factory=list)
    #: CustomMsg 逐点时间统计（INFO 级，不参与 FAIL）
    custom_point_time: PointTimeStats = field(default_factory=PointTimeStats)

    #: 显式采样导致覆盖不完整
    sampled: bool = False

    @property
    def span_s(self) -> float:
        if self.first_ns is None or self.last_ns is None:
            return 0.0
        return max(0.0, (self.last_ns - self.first_ns) * NS)

    @property
    def avg_hz(self) -> float:
        span = self.span_s
        if self.count < 2 or span <= 0:
            return 0.0
        return (self.count - 1) / span

    @property
    def header_span_s(self) -> float:
        if self.header_first_ns is None or self.header_last_ns is None:
            return 0.0
        return max(0.0, (self.header_last_ns - self.header_first_ns) * NS)


# --------------------------------------------------------------------------
# 消息结构校验
# --------------------------------------------------------------------------


def validate_pointcloud2(msg: Any) -> list[str]:
    """PointCloud2 必须是结构完整、非空的点云。

    覆盖 RF-LIDAR-004 复现的 width=0 / data 空 / fields 空 仍被放过的问题；
    也覆盖“timestamp 字段声明 offset=16 FLOAT64 但 point_step 只有 20”这种
    自相矛盾的合成输入（合法最小 point_step 为 24）。
    """
    problems: list[str] = []
    width = int(getattr(msg, "width", 0))
    height = int(getattr(msg, "height", 0))
    point_step = int(getattr(msg, "point_step", 0))
    row_step = int(getattr(msg, "row_step", 0))
    fields = list(getattr(msg, "fields", []) or [])
    data = getattr(msg, "data", b"") or b""

    if width <= 0:
        problems.append(f"width={width} 非法")
    if height <= 0:
        problems.append(f"height={height} 非法")
    if not fields:
        problems.append("fields 为空")
    names = {f.name for f in fields}
    for axis in ("x", "y", "z"):
        if axis not in names:
            problems.append(f"缺少字段 {axis}")
    if point_step <= 0:
        problems.append(f"point_step={point_step} 非法")
    else:
        for f in fields:
            size = POINTFIELD_SIZE.get(int(f.datatype), 0) * max(1, int(f.count))
            if size == 0:
                problems.append(f"字段 {f.name} datatype={f.datatype} 非法")
            elif int(f.offset) + size > point_step:
                problems.append(
                    f"字段 {f.name} offset={f.offset}+{size} 超出 point_step={point_step}"
                )
    if width > 0 and point_step > 0:
        expected_row = point_step * width
        if row_step != expected_row:
            problems.append(f"row_step={row_step} != point_step*width={expected_row}")
        elif height > 0:
            expected_len = row_step * height
            if len(data) != expected_len:
                problems.append(f"data 长度 {len(data)} != row_step*height={expected_len}")
    if width > 0 and not data:
        problems.append("data 为空")
    return problems


def _as_uint32_offset(raw: Any) -> int | None:
    """把 CustomPoint.offset_time 读成 uint32；不可表示时返回 None。

    依据只有消息类型本身：``CustomPoint.msg`` 声明 ``uint32 offset_time``。
    这里**只**做类型可表示性检查，不对取值大小下业务判据
    （消息与 driver 都没有给 offset_time 定义上界）。
    """
    if isinstance(raw, bool) or not isinstance(raw, int):
        return None
    if not 0 <= raw <= UINT32_MAX:
        return None
    return raw


def validate_custommsg(msg: Any) -> list[str]:
    """CustomMsg 的结构合法性（FAIL 级）：point_num 与数组一致、点非空、逐点 offset_time 可表示。

    **不**检查 ``points[]`` 是否按 ``offset_time`` 有序，也不检查单帧 offset 是否"冻结"：
    driver 1.2.6 按 UDP 包到达次序逐点追加且全链路无排序
    （``src/comm/pub_handler.cpp:149/240/388-390``），包到达次序与时间次序不一致是合法现象，
    消息类型也允许全帧 offset 相同（``point_interval==0`` 时 driver 就会这样产出）。
    逐点时间分布只做 INFO 统计，见 :func:`collect_custommsg_point_time`。
    """
    problems: list[str] = []
    points = list(getattr(msg, "points", []) or [])
    point_num = int(getattr(msg, "point_num", -1))

    if point_num != len(points):
        problems.append(f"point_num={point_num} != len(points)={len(points)}")
    if not points:
        problems.append("points 为空")
        return problems

    for idx, point in enumerate(points):
        if not hasattr(point, "offset_time"):
            problems.append(f"第 {idx} 个点缺少 offset_time 字段")
            break
        raw = point.offset_time
        if _as_uint32_offset(raw) is None:
            problems.append(
                f"第 {idx} 个点 offset_time={raw!r} 不是 uint32 可表示值"
                f"（CustomPoint.msg 声明 uint32，范围 0–{UINT32_MAX}）"
            )
            break
    return problems


def collect_custommsg_point_time(stats: PointTimeStats, msg: Any) -> bool:
    """累计 CustomMsg 逐点 ``offset_time`` 的统计量（**只**用于 INFO，不参与 FAIL 判定）。

    上游口径（driver 1.2.6 冻结源码，见模块 docstring）：
      * 单位 ns：``offset_time = 包时间戳 + i * point_interval``（``pub_handler.cpp:388``）；
      * 帧内首点 offset 恒为 0：``lddc.cpp:395`` 以帧内首个被追加的点为基准；
      * ``points[]`` 顺序 = UDP 包到达顺序，**不排序**，故相邻逆序/回退属正常现象。

    统计按**整帧原子**提交：只要存在"缺少 offset_time / 不可表示"的点
    （该帧已由 :func:`validate_custommsg` 判 FAIL），本函数不写入任何局部计数，
    避免"前半帧合法、后半帧非法"时产生不完整统计。返回是否已提交本帧。
    """
    points = list(getattr(msg, "points", []) or [])
    offsets: list[int] = []
    for point in points:
        value = _as_uint32_offset(getattr(point, "offset_time", None))
        if value is None:
            return False  # 结构问题由 validate_custommsg 记为 FAIL，整帧不计入 INFO 统计
        offsets.append(value)
    if not offsets:
        return False

    stats.frames += 1
    stats.points += len(offsets)
    if len(offsets) > 1:
        regressions = [a - b for a, b in zip(offsets, offsets[1:]) if b < a]
        if regressions:
            stats.frames_with_inversions += 1
            stats.inversions += len(regressions)
            stats.worst_regression_ns = max(stats.worst_regression_ns, max(regressions))
        if len(set(offsets)) == 1:
            stats.frozen_frames += 1
    low, high = min(offsets), max(offsets)
    stats.min_offset_ns = low if stats.min_offset_ns is None else min(stats.min_offset_ns, low)
    stats.max_offset_ns = high if stats.max_offset_ns is None else max(stats.max_offset_ns, high)
    return True


def custom_point_time_detail(stats: PointTimeStats) -> str:
    """生成 CustomMsg 逐点 offset_time 的 INFO 报告文本（含上游口径与不确定性）。"""
    return (
        f"逐点 offset_time 分布（仅 INFO，不作 FAIL 判据）：{stats.frames} 帧 / {stats.points} 点；"
        f"相邻逆序 {stats.inversions} 处，涉及 {stats.frames_with_inversions} 帧，"
        f"单次最大回退 {_fmt_ns(stats.worst_regression_ns)}；"
        f"全帧 offset 相同（冻结）{stats.frozen_frames} 帧；"
        f"offset 范围 {_fmt_ns(stats.min_offset_ns)} – {_fmt_ns(stats.max_offset_ns)}（相对本帧基准点）。"
        "上游口径：offset_time 为 ns、相对本帧首个被追加的点（lddc.cpp:395），"
        "而点按 UDP 包到达次序追加、全链路无排序（pub_handler.cpp:149/240/388-390），"
        "故逆序/回退是包到达次序与时间次序不一致的正常表现。"
        "offset_time 的**合法上界未由消息类型或 driver 定义**（uint32 内即合法），"
        "本工具不据样本特征设阈值；若需按帧周期判界，须由上游/配置先给出定义。"
    )


def _fmt_ns(ns: int | None) -> str:
    """把 ns 值格式化成人可读文本（仅用于报告）。"""
    if ns is None:
        return "无"
    if ns >= 1_000_000:
        return f"{ns} ns（{ns / 1e6:.3f} ms）"
    if ns >= 1_000:
        return f"{ns} ns（{ns / 1e3:.3f} µs）"
    return f"{ns} ns"


# --------------------------------------------------------------------------
# bag 读取
# --------------------------------------------------------------------------


def _detect_storage_id(bag_path: Path, override: str | None) -> str:
    if override:
        return override
    if bag_path.is_file():
        suffix = bag_path.suffix.lower()
        if suffix == ".mcap":
            return "mcap"
        if suffix == ".db3":
            return "sqlite3"
        raise ValueError(
            f"无法从扩展名判断存储格式：{bag_path}（请用 --storage-id 指定 sqlite3 或 mcap）"
        )
    if bag_path.is_dir():
        if any(bag_path.glob("*.mcap")):
            return "mcap"
        return "sqlite3"
    raise ValueError(f"bag 路径不存在：{bag_path}")


def _import_rosbag2():  # pragma: no cover - 依赖运行环境
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "错误：无法导入 rosbag2_py。请在已 source ROS 2 的终端中运行：\n"
            "  source /opt/ros/humble/setup.bash\n"
            f"（原始错误：{exc}）"
        ) from exc
    return SequentialReader, StorageOptions, ConverterOptions


def _type_class(type_name: str):  # pragma: no cover - 依赖运行环境
    from rosidl_runtime_py.utilities import get_message

    return get_message(type_name)


def _stamp_ns(msg: Any) -> int | None:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def collect_stats(
    bag_path: Path,
    storage_id: str,
    lidar_topic: str,
    imu_topic: str,
    lidar_type: str,
    header_sample: int | None,
) -> tuple[dict[str, StreamStats], dict[str, str], list[str]]:
    """单遍扫描 bag，对每条消息做时间与结构校验。

    默认**全量**检查帧头时间；``header_sample`` 为显式采样上限（会标记 sampled=True，
    使整体判定不可能为 PASS）。

    关于“接收时间倒退”的实测限制：rosbag2 的 sqlite3 存储读取时会按时间戳排序
    （实测：写入 1000,2000,3000,2500,4000 → 读出 1000,2000,2500,3000,4000）。
    因此接收时间倒退对这类包几乎不可触发；真正能发现时间基准异常的是消息内帧头时间。
    """
    SequentialReader, StorageOptions, ConverterOptions = _import_rosbag2()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    stats = {
        lidar_topic: StreamStats(topic=lidar_topic, expected_type=lidar_type),
        imu_topic: StreamStats(topic=imu_topic, expected_type=IMU_TYPE),
    }
    warnings: list[str] = []
    msg_classes: dict[str, Any] = {}
    for topic, st in stats.items():
        st.found_type = topic_types.get(topic)
        if st.found_type is None:
            continue
        try:
            msg_classes[topic] = _type_class(st.found_type)
        except Exception as exc:  # noqa: BLE001 - 类型不可导入 = 无法验收，评价阶段 FAIL
            st.type_error = f"{type(exc).__name__}: {exc}"

    from rclpy.serialization import deserialize_message

    prev_ns: dict[str, int] = {}
    header_prev: dict[str, int] = {}
    header_seen: dict[str, int] = {}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        st = stats.get(topic)
        if st is None:
            continue
        st.count += 1
        if st.first_ns is None:
            st.first_ns = t_ns
        st.last_ns = t_ns

        prev = prev_ns.get(topic)
        if prev is not None:
            delta = (t_ns - prev) * NS
            if delta < 0:
                st.regressions += 1
                st.worst_regression_s = max(st.worst_regression_s, -delta)
            elif delta > st.max_gap_s:
                st.max_gap_s = delta
                st.max_gap_at_s = prev * NS
        prev_ns[topic] = t_ns

        cls = msg_classes.get(topic)
        if cls is None:
            continue  # 类型不可导入已在 type_error 记录，评价阶段会 FAIL

        # 帧头采样：仅当显式给出上限时才跳过部分消息
        if header_sample is not None:
            seen = header_seen.get(topic, 0)
            if seen >= header_sample:
                st.sampled = True
                continue
            header_seen[topic] = seen + 1

        try:
            msg = deserialize_message(data, cls)
        except Exception as exc:  # noqa: BLE001 - 解码失败必须计入 FAIL
            st.decode_failures += 1
            if len(warnings) < MAX_PROBLEM_SAMPLES:
                warnings.append(f"{topic} 反序列化失败（第 {st.count} 条）：{exc}")
            continue

        stamp = _stamp_ns(msg)
        if stamp is None:
            st.missing_header += 1
        else:
            st.header_checked += 1
            if st.header_first_ns is None:
                st.header_first_ns = stamp
            st.header_last_ns = stamp
            hprev = header_prev.get(topic)
            if hprev is not None:
                # 逐相邻对判定：倒退、重复（中段冻结）、正向跳变
                if stamp < hprev:
                    st.header_regressions += 1
                elif stamp == hprev:
                    st.header_repeats += 1
                else:
                    gap = (stamp - hprev) * NS
                    if gap > st.header_max_gap_s:
                        st.header_max_gap_s = gap
                        st.header_max_gap_at_ns = hprev
            header_prev[topic] = stamp

        if topic == lidar_topic:
            if st.found_type == CUSTOMMSG_TYPE:
                problems = validate_custommsg(msg)
                # 结构非法的帧不进入 INFO 统计：只统计结构合法帧，避免半帧计数
                if not problems:
                    collect_custommsg_point_time(st.custom_point_time, msg)
                st.custom_points_checked = st.custom_point_time.points
                if problems:
                    st.custom_bad += 1
                    if len(st.custom_problems) < MAX_PROBLEM_SAMPLES:
                        st.custom_problems.extend(problems[:MAX_PROBLEM_SAMPLES])
            elif st.found_type == POINTCLOUD2_TYPE:
                problems = validate_pointcloud2(msg)
                if not st.pc2_fields:
                    st.pc2_fields = [f.name for f in (getattr(msg, "fields", []) or [])]
                if problems:
                    st.pc2_struct_bad += 1
                    if len(st.pc2_problems) < MAX_PROBLEM_SAMPLES:
                        st.pc2_problems.extend(problems[:MAX_PROBLEM_SAMPLES])

    return stats, topic_types, warnings


# --------------------------------------------------------------------------
# 判定
# --------------------------------------------------------------------------


def evaluate(
    stats: dict[str, StreamStats],
    topic_types: dict[str, str],
    lidar_topic: str,
    imu_topic: str,
    args: argparse.Namespace,
) -> tuple[list[Check], str]:
    checks: list[Check] = []
    lidar = stats[lidar_topic]
    imu = stats[imu_topic]
    inconclusive_reasons: list[str] = []

    def add(check_id: str, ok: bool, detail: str) -> None:
        checks.append(Check(check_id, "PASS" if ok else "FAIL", detail))

    if not topic_types:
        checks.append(Check("bag.topics", "FAIL", "bag 中没有任何话题（空包）"))
        return checks, "FAIL"
    checks.append(
        Check("bag.topics", "INFO", f"共 {len(topic_types)} 个话题：" + ", ".join(sorted(topic_types)))
    )

    # ---------------- 点云流：存在性/类型/可导入/数量/解码
    add(
        "lidar.present",
        lidar.found_type is not None,
        f"话题 {lidar_topic} " + ("存在" if lidar.found_type else "不存在（点云流缺失）"),
    )
    if lidar.found_type is None:
        checks.append(Check("lidar.skipped", "INFO", "点云流缺失，后续点云检查未执行"))
    else:
        add(
            "lidar.type",
            lidar.found_type == lidar.expected_type,
            f"{lidar_topic} 类型 {lidar.found_type}（期望 {lidar.expected_type}）",
        )
        add(
            "lidar.type_loadable",
            lidar.type_error is None,
            f"消息类型可导入（{lidar.found_type}）"
            if lidar.type_error is None
            else f"消息类型无法导入：{lidar.type_error}"
            "（CustomMsg 需要 source livox underlay）",
        )
        add("lidar.count", lidar.count > 0, f"{lidar_topic} 消息数 {lidar.count}")
        if lidar.type_error is None and lidar.found_type == lidar.expected_type:
            add(
                "lidar.decode",
                lidar.decode_failures == 0,
                f"反序列化失败 {lidar.decode_failures}/{lidar.count} 条",
            )
            add(
                "lidar.header_present",
                lidar.missing_header == 0,
                f"缺少帧头 {lidar.missing_header}/{lidar.header_checked + lidar.missing_header} 条",
            )
            self_check_stream_common(add, lidar, "lidar", args)
            # RF-LIDAR-006：CLI/JSON 一直记录 --lidar-hz-min/max，但此前只有 IMU
            # 有频率判据；点云频率必须同样给出明确 PASS/FAIL（门限不改）。
            add(
                "lidar.rate",
                args.lidar_hz_min <= lidar.avg_hz <= args.lidar_hz_max,
                f"点云平均频率 {lidar.avg_hz:.3f} Hz"
                f"（要求 {args.lidar_hz_min:g}–{args.lidar_hz_max:g} Hz）",
            )
            if lidar.found_type == POINTCLOUD2_TYPE:
                add(
                    "lidar.pc2_structure",
                    lidar.pc2_struct_bad == 0,
                    f"PointCloud2 结构非法 {lidar.pc2_struct_bad} 条；字段 {lidar.pc2_fields}"
                    + (f"；示例：{lidar.pc2_problems}" if lidar.pc2_problems else ""),
                )
                checks.append(
                    Check(
                        "lidar.pc2_point_time",
                        "INFO" if "timestamp" in lidar.pc2_fields else "FAIL",
                        "点级时间字段存在（timestamp）"
                        if "timestamp" in lidar.pc2_fields
                        else f"PointCloud2 缺少点级 timestamp 字段（字段：{lidar.pc2_fields}）",
                    )
                )
            elif lidar.found_type == CUSTOMMSG_TYPE:
                add(
                    "lidar.custom_structure",
                    lidar.custom_bad == 0,
                    f"CustomMsg 结构非法 {lidar.custom_bad} 条；"
                    f"结构合法帧内检查 {lidar.custom_points_checked} 点"
                    + (f"；示例：{lidar.custom_problems}" if lidar.custom_problems else ""),
                )
                checks.append(
                    Check(
                        "lidar.custom_point_time",
                        "INFO",
                        custom_point_time_detail(lidar.custom_point_time),
                    )
                )

    # ---------------- IMU 流
    if imu.found_type is None:
        checks.append(Check("imu.present", "FAIL", f"话题 {imu_topic} 不存在（IMU 流缺失）"))
        checks.append(Check("imu.skipped", "INFO", "IMU 流缺失，后续 IMU 检查未执行"))
    else:
        add(
            "imu.present",
            True,
            f"话题 {imu_topic} 存在",
        )
        add(
            "imu.type",
            imu.found_type == IMU_TYPE,
            f"{imu_topic} 类型 {imu.found_type}（期望 {IMU_TYPE}）",
        )
        add(
            "imu.type_loadable",
            imu.type_error is None,
            "消息类型可导入" if imu.type_error is None else f"消息类型无法导入：{imu.type_error}",
        )
        add("imu.count", imu.count > 0, f"{imu_topic} 消息数 {imu.count}")
        if imu.type_error is None and imu.found_type == IMU_TYPE:
            add(
                "imu.decode",
                imu.decode_failures == 0,
                f"反序列化失败 {imu.decode_failures}/{imu.count} 条",
            )
            add(
                "imu.header_present",
                imu.missing_header == 0,
                f"缺少帧头 {imu.missing_header}/{imu.header_checked + imu.missing_header} 条",
            )
            self_check_stream_common(add, imu, "imu", args)

            checks.append(
                Check(
                    "imu.units",
                    "INFO",
                    f"原始值原样保留（driver 未做换算）：angular_velocity 单位 "
                    f"{IMU_ANGULAR_VELOCITY_UNIT}，linear_acceleration 单位 "
                    f"{IMU_LINEAR_ACCELERATION_UNIT}（**不是** m/s²）；"
                    f"orientation {'已发布' if IMU_ORIENTATION_PUBLISHED else '未发布（默认值，不得当作有效姿态）'}。"
                    f"frame={IMU_FRAME_ID_UPSTREAM}（上游硬编码，与配置的点云 frame 不同）。"
                    "本工具不做 SI 转换，也不证明该 IMU 可用于融合。",
                )
            )

            if args.imu_hz_min is None or args.imu_hz_max is None:
                checks.append(
                    Check(
                        "imu.rate",
                        "INFO",
                        f"IMU 平均频率 {imu.avg_hz:.3f} Hz，但未提供标称频率判据"
                        "（--imu-hz-min/--imu-hz-max）",
                    )
                )
                inconclusive_reasons.append("未提供 IMU 标称频率判据")
            else:
                add(
                    "imu.rate",
                    args.imu_hz_min <= imu.avg_hz <= args.imu_hz_max,
                    f"IMU 平均频率 {imu.avg_hz:.3f} Hz（要求 {args.imu_hz_min:g}–{args.imu_hz_max:g} Hz）",
                )

    # ---------------- 同一采集窗口：源帧头时间范围重叠与覆盖
    sampling_active = lidar.sampled or imu.sampled
    if sampling_active:
        # 采样时两端帧头范围都被截断，重叠无法在全量数据上判定。
        # 这里既不判 PASS 也不伪造 FAIL，而是标 INFO 并把整体压到 INCONCLUSIVE。
        checks.append(
            Check(
                "streams.window",
                "INFO",
                f"使用 --header-sample={args.header_sample} 采样："
                f"点云检查 {lidar.header_checked}/{lidar.count} 条，"
                f"IMU 检查 {imu.header_checked}/{imu.count} 条；"
                "无法在全量数据上判定同一采集窗口与重叠覆盖，整体不能判 PASS",
            )
        )
        inconclusive_reasons.append("显式采样导致帧头时间未被全量检查")
    else:
        both_usable = (
            lidar.header_checked > 0
            and imu.header_checked > 0
            and lidar.header_first_ns is not None
            and imu.header_first_ns is not None
        )
        if not both_usable:
            checks.append(
                Check(
                    "streams.window",
                    "FAIL",
                    "无法确定两条流的源帧头时间范围（帧头缺失、解码失败或类型不可用），"
                    "不能证明同一采集窗口",
                )
            )
        else:
            overlap_start = max(lidar.header_first_ns, imu.header_first_ns)
            overlap_end = min(lidar.header_last_ns, imu.header_last_ns)
            overlap_s = (overlap_end - overlap_start) * NS
            add(
                "streams.header_overlap",
                overlap_s > 0,
                f"源帧头时间范围重叠 {overlap_s:.3f}s"
                f"（点云 {lidar.header_first_ns}–{lidar.header_last_ns} ns，"
                f"IMU {imu.header_first_ns}–{imu.header_last_ns} ns）",
            )
            add(
                "streams.window_coverage",
                overlap_s >= args.expected_duration,
                f"同一采集窗口重叠 {overlap_s:.3f}s（要求 >= {args.expected_duration:.1f}s）",
            )

    if any(c.status == "FAIL" for c in checks):
        return checks, "FAIL"
    if inconclusive_reasons:
        checks.append(
            Check("verdict.inconclusive", "INFO", "缺少判定所需输入：" + "；".join(inconclusive_reasons))
        )
        return checks, "INCONCLUSIVE"
    return checks, "PASS"


def self_check_stream_common(add, st: StreamStats, label: str, args: argparse.Namespace) -> None:
    """两条流共用的时间类检查：时长、断流、接收时间单调、帧头单调与非冻结。"""
    add(
        f"{label}.duration",
        st.span_s >= args.expected_duration,
        f"覆盖时长 {st.span_s:.1f}s（要求 >= {args.expected_duration:.1f}s）",
    )
    add(
        f"{label}.gap",
        st.max_gap_s <= args.max_gap,
        f"最大断流 {st.max_gap_s:.3f}s（要求 <= {args.max_gap:g}s）"
        + (f"，位于 t≈{st.max_gap_at_s:.3f}s" if st.max_gap_at_s is not None else ""),
    )
    add(
        f"{label}.time_regression",
        st.regressions == 0,
        f"接收时间倒退 {st.regressions} 次"
        + (f"（最大 {st.worst_regression_s:.6f}s）" if st.regressions else "")
        + "；注：rosbag2 sqlite3 读取时按时间戳排序，该检查对这类包基本不可触发",
    )
    add(
        f"{label}.header_monotonic",
        st.header_checked > 0 and st.header_regressions == 0 and st.header_repeats == 0,
        f"帧头时间严格递增（逐相邻对检查 {st.header_checked} 条；"
        f"倒退 {st.header_regressions} 次，重复/冻结 {st.header_repeats} 次）"
        if st.header_checked > 0
        else "没有任何可用帧头时间（0 条），无法证明时间基准有效",
    )
    add(
        f"{label}.header_gap",
        st.header_checked > 0 and st.header_max_gap_s <= args.max_gap,
        f"源帧头最大间隔 {st.header_max_gap_s:.3f}s（要求 <= {args.max_gap:g}s）"
        + (f"，起于 {st.header_max_gap_at_ns} ns" if st.header_max_gap_at_ns is not None else "")
        + "；该检查针对传感器时间跳变，接收时间可能仍然连续",
    )


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def _positive_finite(name: str, value: float) -> float:
    if value is None or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值，实际 {value!r}")
    if value <= 0:
        raise ValueError(f"{name} 必须为正数，实际 {value}")
    return value


def validate_args(args: argparse.Namespace) -> None:
    """阈值必须合法，且不允许用采样/0 值绕过验收（RF-LIDAR-004）。"""
    if not args.lidar_topic or not args.imu_topic:
        raise ValueError("--lidar-topic 与 --imu-topic 不能为空")
    if args.lidar_topic == args.imu_topic:
        raise ValueError(
            f"--lidar-topic 与 --imu-topic 不能相同（都是 {args.lidar_topic}）："
            "否则两条流的统计会被合并成一条，产生虚假结论"
        )

    _positive_finite("--expected-duration", args.expected_duration)
    _positive_finite("--lidar-hz-min", args.lidar_hz_min)
    _positive_finite("--lidar-hz-max", args.lidar_hz_max)
    if args.lidar_hz_min > args.lidar_hz_max:
        raise ValueError(
            f"--lidar-hz-min({args.lidar_hz_min:g}) 不能大于 --lidar-hz-max({args.lidar_hz_max:g})"
        )
    _positive_finite("--max-gap", args.max_gap)

    if (args.imu_hz_min is None) != (args.imu_hz_max is None):
        raise ValueError("--imu-hz-min 与 --imu-hz-max 必须同时提供或同时省略")
    if args.imu_hz_min is not None:
        _positive_finite("--imu-hz-min", args.imu_hz_min)
        _positive_finite("--imu-hz-max", args.imu_hz_max)
        if args.imu_hz_min > args.imu_hz_max:
            raise ValueError(
                f"--imu-hz-min({args.imu_hz_min:g}) 不能大于 --imu-hz-max({args.imu_hz_max:g})"
            )

    if args.header_sample is not None:
        if args.header_sample < 1:
            raise ValueError(
                f"--header-sample={args.header_sample} 非法：它只能减少检查量、不能用于通过验收；"
                "省略该参数即为全量检查（默认）"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mid360s_bag_qc",
        description="MID-360S 原始 rosbag 离线质量验收（时间基准与消息结构的数值一致性检查）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "退出码：0=PASS，1=FAIL，2=用法/输入错误，3=INCONCLUSIVE（缺少判据或做了采样）\n"
            "默认全量检查帧头时间；显式 --header-sample 会丢失信息，故只能 INCONCLUSIVE。\n"
            "注意：本工具不能证明硬件时间同步精度，也不做 IMU 单位换算。\n"
        ),
    )
    parser.add_argument("--bag", required=True, help="rosbag2 目录、.db3 或 .mcap")
    parser.add_argument("--storage-id", default=None, choices=["sqlite3", "mcap"], help="覆盖存储格式自动判断")
    parser.add_argument("--lidar-topic", default=DEFAULT_LIDAR_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument(
        "--pointcloud-format",
        default="pointcloud2",
        choices=sorted(XFER_FORMAT_BY_NAME),
        help="点云消息类型：pointcloud2（xfer_format=0）或 custom（xfer_format=1）",
    )
    parser.add_argument("--expected-duration", type=float, default=DEFAULT_EXPECTED_DURATION_S)
    parser.add_argument("--lidar-hz-min", type=float, default=DEFAULT_LIDAR_HZ_MIN)
    parser.add_argument("--lidar-hz-max", type=float, default=DEFAULT_LIDAR_HZ_MAX)
    parser.add_argument("--max-gap", type=float, default=DEFAULT_MAX_GAP_S)
    parser.add_argument(
        "--imu-hz-min",
        type=float,
        default=None,
        help="IMU 标称频率下限；不提供则整体判定为 INCONCLUSIVE",
    )
    parser.add_argument("--imu-hz-max", type=float, default=None)
    parser.add_argument(
        "--header-sample",
        type=int,
        default=None,
        help="仅调试用：只检查每条流前 N 条帧头。会强制 INCONCLUSIVE，不能用于判 PASS",
    )
    parser.add_argument("--json-out", default=None, help="把结构化结果写入该 JSON（用于取证）")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        validate_args(args)
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    bag_path = Path(args.bag).expanduser()
    if not bag_path.exists():
        print(f"错误：bag 路径不存在：{bag_path}", file=sys.stderr)
        return EXIT_USAGE

    try:
        storage_id = _detect_storage_id(bag_path, args.storage_id)
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    lidar_type = POINTCLOUD2_TYPE if args.pointcloud_format == "pointcloud2" else CUSTOMMSG_TYPE

    print(f"bag        : {bag_path}")
    print(f"storage    : {storage_id}")
    print(f"点云话题   : {args.lidar_topic}  期望类型 {lidar_type}")
    print(f"IMU 话题   : {args.imu_topic}  期望类型 {IMU_TYPE}")
    print(
        f"阈值       : 时长>={args.expected_duration:g}s  "
        f"点云 {args.lidar_hz_min:g}–{args.lidar_hz_max:g}Hz  两流最大断流<={args.max_gap:g}s"
    )
    print(f"帧头检查   : {'全量' if args.header_sample is None else f'采样前 {args.header_sample} 条（将判 INCONCLUSIVE）'}")
    print()

    try:
        stats, topic_types, warnings = collect_stats(
            bag_path, storage_id, args.lidar_topic, args.imu_topic, lidar_type, args.header_sample
        )
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 - 打不开的包要如实报告
        print(f"错误：无法读取 bag（{type(exc).__name__}: {exc}）", file=sys.stderr)
        return EXIT_USAGE

    checks, verdict = evaluate(stats, topic_types, args.lidar_topic, args.imu_topic, args)

    for check in checks:
        print(f"[{check.status}] {check.check_id:<28} {check.detail}")
    for warning in warnings:
        print(f"[WARN] {warning}")

    print()
    print(f"结论: {verdict}")
    if verdict == "INCONCLUSIVE":
        print("  → 数据本身未见违规，但缺少判定所需输入，不能判整体验收通过。")
    elif verdict == "PASS":
        print("  → 仅表示时间基准与消息结构的数值一致性检查通过；")
        print("     不证明硬件时间同步精度，也不表示 IMU 单位已符合 SI。")

    if args.json_out:
        payload = {
            "bag": str(bag_path),
            "storage_id": storage_id,
            "verdict": verdict,
            "thresholds": {
                "expected_duration_s": args.expected_duration,
                "lidar_hz": [args.lidar_hz_min, args.lidar_hz_max],
                "max_gap_s": args.max_gap,
                "imu_hz": [args.imu_hz_min, args.imu_hz_max],
                "header_sample": args.header_sample,
            },
            "checks": [asdict(c) for c in checks],
            "streams": {topic: asdict(st) for topic, st in stats.items()},
            "warnings": warnings,
            "note": (
                "仅数据时间基准与消息结构的数值一致性检查；不证明硬件时间同步精度，"
                "也不表示点云/IMU 物理正确。原始 IMU 为未换算值："
                f"linear_acceleration 单位 {IMU_LINEAR_ACCELERATION_UNIT}（非 m/s²），"
                f"angular_velocity 单位 {IMU_ANGULAR_VELOCITY_UNIT}，"
                "orientation 未发布；不得据此视为已符合 SI 的融合输入。"
            ),
        }
        out = Path(args.json_out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"结构化结果: {out}")

    return {"PASS": EXIT_PASS, "FAIL": EXIT_FAIL, "INCONCLUSIVE": EXIT_INCONCLUSIVE}[verdict]


if __name__ == "__main__":
    sys.exit(main())
