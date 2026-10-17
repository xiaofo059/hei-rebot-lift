"""bag 质量验收工具的无硬件测试（全部使用 SYNTHETIC 合成数据）。

重要：这里生成的 bag 全部是**合成数据**，不含任何真实雷达/IMU 观测。
测试只验证判定逻辑与退出码，不表示硬件链路或时间同步已通过。
下文任何"合法乱序""逆序 N 处"均为人工构造的 SYNTHETIC 场景，
不是某个真实 rosbag 的观测结果，也不能当作硬件证据。

包含 RF-LIDAR-004 的四个独立复现（此前四例均被错误判为 PASS/exit 0）。

CustomMsg 逐点时间：driver 1.2.6 按 UDP 包到达次序追加点、全链路不排序
（pub_handler.cpp:149/240/388-390），因此**逐点 offset_time 顺序/分布只做 INFO 统计**，
不是 FAIL 判据；消息级 header.stamp 判据不受影响，仍有独立回归用例。
"""

from __future__ import annotations

from pathlib import Path

import pytest

rosbag2_py = pytest.importorskip("rosbag2_py", reason="需要 rosbag2_py（source ROS 2 后运行）")
rclpy_serialization = pytest.importorskip("rclpy.serialization")

from builtin_interfaces.msg import Time  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from hei_robot_bringup import bag_qc  # noqa: E402
from hei_robot_bringup.mid360s_config import CUSTOMMSG_TYPE, IMU_TYPE, POINTCLOUD2_TYPE  # noqa: E402

LIDAR = "/livox/lidar"
IMU = "/livox/imu"

PC2_POINT_STEP = 24  # x@0 F32, y@4 F32, z@8 F32, intensity@12 F32, timestamp@16 F64
PC2_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
]


def _header(t_ns: int) -> Header:
    header = Header()
    header.frame_id = "lidar_link"
    header.stamp = Time(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000)
    return header


def _pc2_msg(t_ns: int, *, points: int = 1, empty: bool = False) -> PointCloud2:
    """构造结构合法的 PointCloud2。

    ``empty=True`` 复现 RF-LIDAR-004 第 ④ 例：width=0 / data 空 / fields 空。
    """
    msg = PointCloud2()
    msg.header = _header(t_ns)
    if empty:
        return msg
    msg.height = 1
    msg.width = points
    msg.fields = PC2_FIELDS
    msg.is_bigendian = False
    msg.point_step = PC2_POINT_STEP
    msg.row_step = PC2_POINT_STEP * points
    msg.is_dense = True
    msg.data = b"\x00" * (msg.row_step * msg.height)
    return msg


def _imu_msg(t_ns: int) -> Imu:
    msg = Imu()
    msg.header = _header(t_ns)
    return msg


def write_bag(
    bag_dir: Path,
    lidar_times: list[int],
    imu_times: list[int],
    *,
    lidar_type: str = POINTCLOUD2_TYPE,
    imu_type: str = IMU_TYPE,
    lidar_payloads: list[bytes] | None = None,
    imu_payloads: list[bytes] | None = None,
) -> Path:
    """写一个合成 bag（SYNTHETIC：除消息类型与时间戳外无真实数据）。"""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(name=LIDAR, type=lidar_type, serialization_format="cdr")
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(name=IMU, type=imu_type, serialization_format="cdr")
    )

    if lidar_payloads is None:
        lidar_payloads = [serialize_message(_pc2_msg(t)) for t in lidar_times]
    assert len(lidar_payloads) == len(lidar_times)
    for t_ns, payload in zip(lidar_times, lidar_payloads):
        writer.write(LIDAR, payload, t_ns)

    if imu_payloads is None:
        imu_payloads = [serialize_message(_imu_msg(t)) for t in imu_times]
    assert len(imu_payloads) == len(imu_times)
    for t_ns, payload in zip(imu_times, imu_payloads):
        writer.write(IMU, payload, t_ns)

    del writer
    return bag_dir


def _times(start_s: float, duration_s: float, hz: float) -> list[int]:
    n = int(round(duration_s * hz)) + 1
    return [int((start_s + i / hz) * 1e9) for i in range(n)]


def _payloads_pc2(times: list[int], **kwargs) -> list[bytes]:
    return [serialize_message(_pc2_msg(t, **kwargs)) for t in times]


@pytest.fixture(scope="module")
def good_bag(tmp_path_factory) -> Path:
    """30 s、点云 10 Hz、IMU 100 Hz、结构合法的合成包。"""
    bag = tmp_path_factory.mktemp("bags") / "good"
    return write_bag(bag, _times(0.0, 30.0, 10.0), _times(0.0, 30.0, 100.0))


def _qc(*args: str) -> int:
    return bag_qc.main(list(args))


# ==========================================================================
# RF-LIDAR-004：四个独立复现（修复前全部错误 PASS/exit 0）
# ==========================================================================


def test_rf004_case1_short_imu_stream_fails(tmp_path, capsys):
    """①点云 2 s@10 Hz，IMU 仅 0.1 s@100 Hz → IMU 时长不足，必须 FAIL。"""
    bag = write_bag(tmp_path / "case1", _times(0.0, 2.0, 10.0), _times(0.0, 0.1, 100.0))
    rc = _qc("--bag", str(bag), "--expected-duration", "1.5")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "imu.duration" in out


def test_rf004_case2_non_overlapping_streams_fail(tmp_path, capsys):
    """②点云 1–3 s、IMU 101–103 s，两流完全不重叠 → 必须 FAIL。"""
    bag = write_bag(tmp_path / "case2", _times(1.0, 2.0, 10.0), _times(101.0, 2.0, 100.0))
    rc = _qc("--bag", str(bag), "--expected-duration", "1.5")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "streams.header_overlap" in out


def test_rf004_case3_undecodable_pointcloud_fails(tmp_path, capsys):
    """③全部点云载荷是非法 CDR → 解码失败必须 FAIL，不能只 WARN 后判通过。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "case3",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_payloads=[b"\x00\x01\x00\x00"] * len(times),  # 只有封装头，无消息体
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.decode" in out


def test_rf004_case4_empty_pointcloud_structure_fails(tmp_path, capsys):
    """④PointCloud2 width=0 / data 空 / fields 空 → 必须 FAIL。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "case4",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_payloads=_payloads_pc2(times, empty=True),
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.pc2_structure" in out


def test_rf004_case4b_width_zero_but_data_present_fails(tmp_path):
    """④变体：width=0 但 data 非空，同样非法。"""
    times = _times(0.0, 6.0, 10.0)
    msgs = []
    for t in times:
        msg = _pc2_msg(t)
        msg.width = 0
        msg.row_step = 0
        msgs.append(serialize_message(msg))
    bag = write_bag(tmp_path / "case4b", times, _times(0.0, 6.0, 100.0), lidar_payloads=msgs)
    assert _qc("--bag", str(bag), "--expected-duration", "5") == bag_qc.EXIT_FAIL


# ==========================================================================
# 两条流的时间门槛
# ==========================================================================


# ==========================================================================
# RF-LIDAR-006：点云平均频率判据（SYNTHETIC；门限仍为 9–11 Hz）
# ==========================================================================


def _rate_bag(tmp_path, name: str, lidar_hz: float):
    """合成 10 s 包：点云 lidar_hz、IMU 100 Hz（SYNTHETIC，非硬件证据）。"""
    bag = write_bag(tmp_path / name, _times(0.0, 10.0, lidar_hz), _times(0.0, 10.0, 100.0))
    return _qc(
        "--bag", str(bag),
        "--expected-duration", "5",
        "--imu-hz-min", "95", "--imu-hz-max", "105",
    )


@pytest.mark.parametrize("lidar_hz", [5.0, 20.0])
def test_lidar_rate_out_of_range_fails(tmp_path, capsys, lidar_hz):
    """RF-LIDAR-006：点云平均频率 5 Hz / 20 Hz 超出 9–11 Hz → 必须 FAIL。

    修复前 lidar_hz_min/max 只出现在打印与 JSON 阈值里，evaluate() 从不判定，
    这类包会被漏检。
    """
    rc = _rate_bag(tmp_path, f"lidar_rate_{int(lidar_hz)}", lidar_hz)
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "[FAIL] lidar.rate" in out, out
    assert "点云平均频率" in out


@pytest.mark.parametrize("lidar_hz", [9.0, 11.0])
def test_lidar_rate_boundary_passes(tmp_path, capsys, lidar_hz):
    """RF-LIDAR-006：恰好 9 Hz / 11 Hz（闭区间边界）→ lidar.rate PASS。"""
    rc = _rate_bag(tmp_path, f"lidar_rate_edge_{int(lidar_hz)}", lidar_hz)
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_PASS, out
    assert "[PASS] lidar.rate" in out, out


def test_lidar_rate_check_also_applies_to_custommsg(tmp_path, capsys):
    """RF-LIDAR-006：CustomMsg 流同样受 lidar.rate 判据约束（5 Hz → FAIL）。"""
    times = _times(0.0, 10.0, 5.0)
    bag = write_bag(
        tmp_path / "custom_rate_low",
        times,
        _times(0.0, 10.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[_custom_payload(t, points=3, offsets=[0, 5000, 10000]) for t in times],
    )
    rc = _qc(
        "--bag", str(bag),
        "--expected-duration", "5",
        "--pointcloud-format", "custom",
        "--imu-hz-min", "95", "--imu-hz-max", "105",
    )
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "[FAIL] lidar.rate" in out, out


def test_imu_midstream_gap_fails(tmp_path, capsys):
    """IMU 中段挖掉 1 s → 断流超限必须 FAIL。"""
    imu = [t for t in _times(0.0, 10.0, 100.0) if not (4.0e9 < t < 5.0e9)]
    bag = write_bag(tmp_path / "imu_gap", _times(0.0, 10.0, 10.0), imu)
    rc = _qc("--bag", str(bag), "--expected-duration", "9")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "imu.gap" in out


def test_imu_all_zero_header_fails(tmp_path, capsys):
    """IMU 帧头时间全 0（冻结）→ 必须 FAIL。"""
    times = _times(0.0, 6.0, 100.0)
    payloads = []
    for t in times:
        msg = _imu_msg(t)
        msg.header.stamp = Time(sec=0, nanosec=0)
        payloads.append(serialize_message(msg))
    bag = write_bag(
        tmp_path / "imu_zero", _times(0.0, 6.0, 10.0), times, imu_payloads=payloads
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "imu.header_monotonic" in out


def test_lidar_frozen_header_time_fails(tmp_path, capsys):
    """点云帧头时间全部相同（冻结但不为 0）→ 必须 FAIL。"""
    times = _times(0.0, 6.0, 10.0)
    payloads = _payloads_pc2([times[0]] * len(times))
    bag = write_bag(tmp_path / "frozen", times, _times(0.0, 6.0, 100.0), lidar_payloads=payloads)
    rc = _qc("--bag", str(bag), "--expected-duration", "5")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.header_monotonic" in out


def test_late_header_regression_detected_with_full_check(tmp_path, capsys):
    """第 150 条之后的帧头倒退也必须被检出（默认全量检查）。"""
    times = _times(0.0, 20.0, 10.0)
    stamps = list(times)
    for i in range(150, len(stamps)):
        stamps[i] -= int(1.0e9)
    bag = write_bag(
        tmp_path / "late_regress",
        times,
        _times(0.0, 20.0, 100.0),
        lidar_payloads=_payloads_pc2(stamps),
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "15")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.header_monotonic" in out


def test_header_sample_cannot_yield_pass(tmp_path, capsys):
    """显式采样会漏掉采样范围之后的异常 → 只能 INCONCLUSIVE，绝不能 PASS。"""
    times = _times(0.0, 20.0, 10.0)
    stamps = list(times)
    for i in range(150, len(stamps)):
        stamps[i] -= int(1.0e9)
    bag = write_bag(
        tmp_path / "sample_vs_full",
        times,
        _times(0.0, 20.0, 100.0),
        lidar_payloads=_payloads_pc2(stamps),
    )
    common = ["--expected-duration", "15", "--imu-hz-min", "95", "--imu-hz-max", "105"]

    full = _qc("--bag", str(bag), *common)
    out_full = capsys.readouterr().out
    assert full == bag_qc.EXIT_FAIL, out_full
    assert "lidar.header_monotonic" in out_full

    sampled = _qc("--bag", str(bag), *common, "--header-sample", "100")
    out_sampled = capsys.readouterr().out
    assert sampled == bag_qc.EXIT_INCONCLUSIVE, out_sampled
    assert "结论: PASS" not in out_sampled


@pytest.mark.parametrize("bad", ["0", "-1", "-100"])
def test_header_sample_bypass_values_rejected(tmp_path, capsys, bad):
    """header-sample 0/负数等绕过验收的取值必须被拒绝。"""
    bag = write_bag(tmp_path / f"bypass_{bad}", _times(0.0, 2.0, 10.0), _times(0.0, 2.0, 100.0))
    assert _qc("--bag", str(bag), "--header-sample", bad) == bag_qc.EXIT_USAGE
    assert "非法" in capsys.readouterr().err


def test_same_topic_for_both_streams_rejected(tmp_path, capsys):
    """lidar 与 imu 话题相同会把两条流的统计合并 → 必须拒绝。"""
    bag = write_bag(tmp_path / "same_topic", _times(0.0, 2.0, 10.0), _times(0.0, 2.0, 100.0))
    rc = _qc("--bag", str(bag), "--lidar-topic", LIDAR, "--imu-topic", LIDAR)
    assert rc == bag_qc.EXIT_USAGE
    assert "不能相同" in capsys.readouterr().err


# ==========================================================================
# RF-LIDAR-004 残留：中段冻结 / 源时间跳变
# ==========================================================================


def test_midstream_header_freeze_then_recovery_fails(tmp_path, capsys):
    """第 6–10 帧保持第 5 帧时间、之后恢复正常 → 必须 FAIL。

    只比较“整段唯一值集合”会漏掉这种中段冻结；必须逐相邻帧头检查。
    """
    times = _times(0.0, 10.0, 10.0)
    stamps = list(times)
    frozen_value = stamps[5]
    for i in range(6, 11):
        stamps[i] = frozen_value
    bag = write_bag(
        tmp_path / "mid_freeze",
        times,
        _times(0.0, 10.0, 100.0),
        lidar_payloads=_payloads_pc2(stamps),
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "9")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.header_monotonic" in out
    assert "重复/冻结 5 次" in out


def test_source_header_gap_detected_even_when_receive_times_contiguous(tmp_path, capsys):
    """接收时间连续、但源帧头跳变 2 s → 必须由 header_gap 抓住。"""
    times = _times(0.0, 10.0, 10.0)
    stamps = list(times)
    for i in range(50, len(stamps)):
        stamps[i] += int(2.0e9)
    bag = write_bag(
        tmp_path / "header_jump",
        times,
        _times(0.0, 10.0, 100.0),
        lidar_payloads=_payloads_pc2(stamps),
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "9")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.header_gap" in out
    assert "[PASS] lidar.gap" in out, "接收时间本身连续，应由源帧头间隔判负"


def test_imu_source_header_gap_fails(tmp_path, capsys):
    """IMU 源帧头中段跳变（接收时间连续）也必须 FAIL。"""
    itimes = _times(0.0, 10.0, 100.0)
    stamps = list(itimes)
    for i in range(400, len(stamps)):
        stamps[i] += int(2.0e9)
    payloads = [serialize_message(Imu(header=_header(s))) for s in stamps]
    bag = write_bag(
        tmp_path / "imu_header_jump",
        _times(0.0, 10.0, 10.0),
        itimes,
        imu_payloads=payloads,
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "9")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "imu.header_gap" in out


# ==========================================================================
# 阈值参数校验
# ==========================================================================


@pytest.mark.parametrize(
    "extra",
    [
        ["--expected-duration", "0"],
        ["--expected-duration", "-5"],
        ["--expected-duration", "nan"],
        ["--expected-duration", "inf"],
        ["--lidar-hz-min", "0"],
        ["--lidar-hz-min", "12", "--lidar-hz-max", "9"],
        ["--lidar-hz-max", "-1"],
        ["--max-gap", "0"],
        ["--max-gap", "nan"],
        ["--imu-hz-min", "100"],
        ["--imu-hz-max", "100"],
        ["--imu-hz-min", "200", "--imu-hz-max", "100"],
        ["--imu-hz-min", "0", "--imu-hz-max", "100"],
    ],
)
def test_invalid_thresholds_rejected(tmp_path, capsys, extra):
    bag = write_bag(tmp_path / "thr", _times(0.0, 2.0, 10.0), _times(0.0, 2.0, 100.0))
    assert _qc("--bag", str(bag), *extra) == bag_qc.EXIT_USAGE
    assert "错误" in capsys.readouterr().err


# ==========================================================================
# 空包 / 缺流 / 类型
# ==========================================================================


def test_short_bag_never_reported_as_30min_pass(good_bag, capsys):
    """30 s 的包在默认 1800 s 判据下必须 FAIL，绝不能判通过。"""
    rc = _qc("--bag", str(good_bag))
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL
    assert "1800" in out
    assert "结论: FAIL" in out
    assert "结论: PASS" not in out


def test_lidar_topic_with_zero_messages_fails(tmp_path, capsys):
    bag = write_bag(tmp_path / "empty_stream", [], _times(0.0, 1.0, 100.0))
    assert _qc("--bag", str(bag)) == bag_qc.EXIT_FAIL
    assert "消息数 0" in capsys.readouterr().out


def test_missing_lidar_topic_fails(tmp_path, capsys):
    """包里根本没有点云话题 → 缺流 FAIL。"""
    bag = tmp_path / "no_lidar"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(name=IMU, type=IMU_TYPE, serialization_format="cdr"))
    for t_ns in _times(0.0, 2.0, 100.0):
        writer.write(IMU, serialize_message(_imu_msg(t_ns)), t_ns)
    del writer

    assert _qc("--bag", str(bag)) == bag_qc.EXIT_FAIL
    out = capsys.readouterr().out
    assert "不存在" in out and "点云流缺失" in out


def test_missing_imu_topic_fails(tmp_path, capsys):
    """只录点云、缺 IMU 流：必须非零退出并说明原因。"""
    bag = tmp_path / "no_imu"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(name=LIDAR, type=POINTCLOUD2_TYPE, serialization_format="cdr"))
    for t_ns in _times(0.0, 10.0, 10.0):
        writer.write(LIDAR, serialize_message(_pc2_msg(t_ns)), t_ns)
    del writer

    assert _qc("--bag", str(bag), "--expected-duration", "5") == bag_qc.EXIT_FAIL
    assert "IMU" in capsys.readouterr().out


def test_wrong_lidar_type_fails(good_bag, capsys):
    """按 PointCloud2 期望，但包里是 CustomMsg 类型 → FAIL。"""
    rc = bag_qc.main(["--bag", str(good_bag), "--pointcloud-format", "custom"])
    assert rc == bag_qc.EXIT_FAIL
    assert "类型" in capsys.readouterr().out


def test_missing_bag_path_is_usage_error(tmp_path):
    assert _qc("--bag", str(tmp_path / "absent")) == bag_qc.EXIT_USAGE


# ==========================================================================
# 通过 / 不确定
# ==========================================================================


def test_inconclusive_when_imu_nominal_frequency_unknown(good_bag, capsys):
    """一切都对但没有 IMU 标称频率判据 → 不能判整体通过。"""
    rc = _qc("--bag", str(good_bag), "--expected-duration", "25", "--max-gap", "0.5")
    assert rc == bag_qc.EXIT_INCONCLUSIVE
    out = capsys.readouterr().out
    assert "结论: INCONCLUSIVE" in out
    assert "IMU 标称频率" in out


def test_pass_when_all_criteria_including_imu_are_met(good_bag, capsys):
    rc = _qc(
        "--bag", str(good_bag),
        "--expected-duration", "25",
        "--imu-hz-min", "95", "--imu-hz-max", "105",
    )
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_PASS, out
    assert "结论: PASS" in out


def test_imu_rate_out_of_range_fails(good_bag, capsys):
    rc = _qc("--bag", str(good_bag), "--expected-duration", "25", "--imu-hz-min", "150", "--imu-hz-max", "250")
    assert rc == bag_qc.EXIT_FAIL
    assert "IMU 平均频率" in capsys.readouterr().out


# ==========================================================================
# 断流 / 时间倒退（含 rosbag2 存储行为）
# ==========================================================================


def test_gap_over_threshold_fails(tmp_path, capsys):
    """在 10 Hz 点云中插入 2 s 空档 → 断流超限 FAIL。"""
    base = _times(0.0, 10.0, 10.0)
    split = len(base) // 2
    shifted = [t + int(2.0 * 1e9) for t in base[split:]]
    times = base[:split] + shifted
    bag = write_bag(tmp_path / "gap", times, _times(0.0, 12.0, 100.0))

    rc = _qc("--bag", str(bag), "--expected-duration", "5", "--max-gap", "0.5")
    assert rc == bag_qc.EXIT_FAIL
    assert "断流" in capsys.readouterr().out


def test_bag_storage_reorders_out_of_order_timestamps(tmp_path):
    """记录实测事实：rosbag2 sqlite3 读取时按时间戳排序。

    所以“接收时间倒退”对这类包几乎不可触发；发现时间基准异常必须靠消息内帧头。
    """
    times = _times(0.0, 5.0, 10.0)
    shuffled = list(times)
    shuffled[20], shuffled[21] = shuffled[21], shuffled[20]
    bag = write_bag(tmp_path / "shuffled", shuffled, [])

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    read_back = []
    while reader.has_next():
        topic, _, t_ns = reader.read_next()
        if topic == LIDAR:
            read_back.append(t_ns)
    assert read_back == sorted(read_back), "rosbag2 读取时应已按时间戳排序"
    assert read_back != shuffled, "前提失效：存储不再重排时间戳，需要重新评估该检查"


# ==========================================================================
# CustomMsg：结构判据（FAIL 级）与逐点时间统计（INFO 级）
# ==========================================================================

pytest.importorskip("livox_ros_driver2", reason="CustomMsg 用例需要 livox underlay")
from livox_ros_driver2.msg import CustomMsg, CustomPoint  # noqa: E402


def _custom_payload(t_ns: int, *, points: int = 1, point_num: int | None = None,
                    offsets: list[int] | None = None) -> bytes:
    msg = CustomMsg()
    msg.header = _header(t_ns)
    msg.timebase = t_ns
    built = []
    for idx in range(points):
        point = CustomPoint()
        point.offset_time = 0 if offsets is None else offsets[idx]
        point.x, point.y, point.z = 1.0, 2.0, 3.0
        built.append(point)
    msg.points = built
    msg.point_num = len(built) if point_num is None else point_num
    return serialize_message(msg)


def test_custommsg_offset_time_present(tmp_path, capsys):
    """CustomMsg 逐点 offset_time 存在 → 该项通过（其余仅剩 IMU 判据缺失）。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "custom_ok",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[_custom_payload(t, offsets=[1000]) for t in times],
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5", "--pointcloud-format", "custom")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_INCONCLUSIVE, out
    assert "lidar.custom_structure" in out
    assert "[INFO] lidar.custom_point_time" in out
    assert "1000 ns" in out


def test_custommsg_empty_points_fails(tmp_path, capsys):
    """CustomMsg 没有任何点 → FAIL。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "custom_empty",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[_custom_payload(t, points=0) for t in times],
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5", "--pointcloud-format", "custom")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "points 为空" in out


def test_custommsg_point_num_mismatch_fails(tmp_path, capsys):
    """point_num 与数组长度不一致 → FAIL；且该帧不得进入 INFO 统计（不允许半帧计数）。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "custom_mismatch",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[_custom_payload(t, points=3, point_num=99, offsets=[1, 2, 3]) for t in times],
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5", "--pointcloud-format", "custom")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "point_num" in out
    # 61 帧全部结构非法 → INFO 统计必须为空帧，不得只计前半帧
    assert "0 帧 / 0 点" in out


#: SYNTHETIC：模拟 MID-360S 的 UDP 包到达次序与包内递增。
#: driver 1.2.6 逐包追加到 points[]（pub_handler.cpp:388-390），不排序；
#: 这里刻意让第 2 个到达的包时间更早（包内间隔 5 us），复现"合法乱序到达"。
ARRIVAL_ORDER_OFFSETS = [2000, 7000, 12000, 0, 5000, 10000, 4000, 9000, 14000]


def test_custommsg_arrival_order_regressions_are_not_failures(tmp_path, capsys):
    """合法乱序 packet arrival（点不按时间排序）不得判 FAIL，只报 INFO 统计。

    依据 driver 1.2.6 冻结源码：points[] 顺序 = UDP 包到达顺序，全链路无排序
    （pub_handler.cpp:149/240/388-390；lddc.cpp:395）。SYNTHETIC 数据，不是硬件证据。
    """
    times = _times(0.0, 6.0, 10.0)  # 61 帧
    bag = write_bag(
        tmp_path / "custom_arrival_order",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[
            _custom_payload(t, points=len(ARRIVAL_ORDER_OFFSETS), offsets=ARRIVAL_ORDER_OFFSETS)
            for t in times
        ],
    )
    rc = _qc(
        "--bag", str(bag),
        "--expected-duration", "5",
        "--pointcloud-format", "custom",
        "--imu-hz-min", "95", "--imu-hz-max", "105",
    )
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_PASS, out
    assert "[PASS] lidar.custom_structure" in out
    assert "[INFO] lidar.custom_point_time" in out
    # 每帧 2 处逆序、单次最大回退 12000 ns，共 61 帧
    assert "相邻逆序 122 处" in out
    assert "涉及 61 帧" in out
    assert "12000 ns" in out
    assert "非单调" not in out


def test_custommsg_frozen_point_times_reported_as_info(tmp_path, capsys):
    """全帧 offset 相同只报 INFO（消息类型允许、point_interval==0 时 driver 合法产出），不判 FAIL。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "custom_frozen",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[_custom_payload(t, points=3, offsets=[7, 7, 7]) for t in times],
    )
    rc = _qc(
        "--bag", str(bag),
        "--expected-duration", "5",
        "--pointcloud-format", "custom",
        "--imu-hz-min", "95", "--imu-hz-max", "105",
    )
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_PASS, out
    assert "[INFO] lidar.custom_point_time" in out
    assert "冻结）61 帧" in out


def test_custommsg_frozen_header_stamp_still_fails(tmp_path, capsys):
    """逐点 offset 改为 INFO 后，CustomMsg 的**消息级**帧头判据必须仍然 FAIL。"""
    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "custom_frozen_header",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[
            _custom_payload(times[0], points=3, offsets=[0, 5000, 10000]) for _ in times
        ],
    )
    rc = _qc("--bag", str(bag), "--expected-duration", "5", "--pointcloud-format", "custom")
    out = capsys.readouterr().out
    assert rc == bag_qc.EXIT_FAIL, out
    assert "lidar.header_monotonic" in out


def test_custommsg_point_time_stats_recorded_in_json(tmp_path, capsys):
    """INFO 统计必须进入 --json-out 取证输出（真实袋复盘时要用）。"""
    import json

    times = _times(0.0, 6.0, 10.0)
    bag = write_bag(
        tmp_path / "custom_json",
        times,
        _times(0.0, 6.0, 100.0),
        lidar_type=CUSTOMMSG_TYPE,
        lidar_payloads=[
            _custom_payload(t, points=len(ARRIVAL_ORDER_OFFSETS), offsets=ARRIVAL_ORDER_OFFSETS)
            for t in times
        ],
    )
    out_json = tmp_path / "qc.json"
    _qc(
        "--bag", str(bag),
        "--expected-duration", "5",
        "--pointcloud-format", "custom",
        "--json-out", str(out_json),
    )
    capsys.readouterr()
    stats = json.loads(out_json.read_text(encoding="utf-8"))["streams"][LIDAR]["custom_point_time"]
    assert stats["frames"] == 61
    assert stats["inversions"] == 122
    assert stats["frames_with_inversions"] == 61
    assert stats["worst_regression_ns"] == 12000
    assert stats["min_offset_ns"] == 0
    assert stats["max_offset_ns"] == 14000
    assert stats["frozen_frames"] == 0


# --------------------------------------------------------------------------
# 结构桩：CustomPoint.offset_time 缺失/不可表示仍必须 FAIL
# --------------------------------------------------------------------------


class _StubPoint:
    def __init__(self, **fields):
        self.__dict__.update(fields)


class _StubMsg:
    def __init__(self, points, point_num=None):
        self.points = points
        self.point_num = len(points) if point_num is None else point_num


def test_custommsg_missing_offset_time_field_fails():
    """点缺少 offset_time 字段 → FAIL（结构桩，不依赖真机/真包）。"""
    problems = bag_qc.validate_custommsg(_StubMsg([_StubPoint(x=1.0, y=2.0, z=3.0)]))
    assert any("缺少 offset_time" in p for p in problems), problems


@pytest.mark.parametrize("bad", [2 ** 32, -1, 1.5, "1000", None])
def test_custommsg_offset_time_not_representable_fails(bad):
    """offset_time 超出 uint32 可表示范围/非整数 → FAIL（CustomPoint.msg 声明 uint32）。"""
    problems = bag_qc.validate_custommsg(_StubMsg([_StubPoint(offset_time=bad)]))
    assert any("uint32" in p for p in problems), problems


def test_custommsg_point_num_mismatch_still_fails_on_structure_stub():
    """结构桩同样覆盖 point_num 不一致（与 bag 级用例互为补充）。"""
    msg = _StubMsg([_StubPoint(offset_time=0), _StubPoint(offset_time=1)], point_num=99)
    problems = bag_qc.validate_custommsg(msg)
    assert any("point_num" in p for p in problems), problems


# --------------------------------------------------------------------------
# INFO 统计必须整帧原子提交（Codex review #1）
# --------------------------------------------------------------------------


def test_custommsg_partial_invalid_frame_not_committed_to_stats():
    """先有合法点、后有非法点 → 整帧不进 INFO 统计（结构桩，SYNTHETIC）。

    否则"前半帧合法、后半帧非法"会把前半帧计入 frames/points/min/max，
    形成不完整统计，误导真实袋复盘。
    """
    stats = bag_qc.PointTimeStats()
    msg = _StubMsg(
        [
            _StubPoint(offset_time=0),
            _StubPoint(offset_time=5000),
            _StubPoint(x=1.0),  # 缺少 offset_time
        ]
    )
    assert any("缺少 offset_time" in p for p in bag_qc.validate_custommsg(msg))
    assert bag_qc.collect_custommsg_point_time(stats, msg) is False
    assert stats == bag_qc.PointTimeStats(), stats
    assert (stats.frames, stats.points, stats.inversions) == (0, 0, 0)
    assert stats.min_offset_ns is None and stats.max_offset_ns is None


def test_custommsg_out_of_range_point_not_committed_to_stats():
    """前半帧合法、后一个点 offset_time 超出 uint32 → 整帧不进统计。"""
    stats = bag_qc.PointTimeStats()
    msg = _StubMsg([_StubPoint(offset_time=0), _StubPoint(offset_time=2 ** 32)])
    assert bag_qc.collect_custommsg_point_time(stats, msg) is False
    assert stats == bag_qc.PointTimeStats(), stats


def test_custommsg_all_valid_frame_is_committed():
    """全部点合法时正常提交（与上面两个"不提交"用例互为对照）。"""
    stats = bag_qc.PointTimeStats()
    msg = _StubMsg([_StubPoint(offset_time=0), _StubPoint(offset_time=5000)])
    assert bag_qc.collect_custommsg_point_time(stats, msg) is True
    assert (stats.frames, stats.points, stats.min_offset_ns, stats.max_offset_ns) == (
        1,
        2,
        0,
        5000,
    )


# ==========================================================================
# 结构化取证输出
# ==========================================================================


def test_reports_upstream_imu_units_and_absent_orientation(good_bag, capsys):
    """验收输出必须写明 IMU 原始单位是 g、orientation 未发布。"""
    _qc("--bag", str(good_bag), "--expected-duration", "25")
    out = capsys.readouterr().out
    assert "imu.units" in out
    assert "linear_acceleration 单位 g" in out
    assert "m/s²" in out and "不是" in out
    assert "orientation 未发布" in out
    assert "livox_frame" in out


def test_json_note_does_not_imply_si_imu(good_bag, tmp_path):
    import json

    out = tmp_path / "qc.json"
    _qc("--bag", str(good_bag), "--expected-duration", "25", "--json-out", str(out))
    note = json.loads(out.read_text(encoding="utf-8"))["note"]
    assert "非 m/s²" in note
    assert "orientation 未发布" in note


def test_json_out_records_verdict(good_bag, tmp_path):
    import json

    out = tmp_path / "qc.json"
    rc = _qc("--bag", str(good_bag), "--expected-duration", "25", "--json-out", str(out))
    assert rc == bag_qc.EXIT_INCONCLUSIVE
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["verdict"] == "INCONCLUSIVE"
    assert "不证明硬件时间同步精度" in payload["note"]
    assert payload["streams"][LIDAR]["count"] == 301
