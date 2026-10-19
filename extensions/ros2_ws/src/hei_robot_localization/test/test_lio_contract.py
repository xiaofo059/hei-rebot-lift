"""输入契约纯函数的回归测试（无 ROS、无硬件、无 bag）。

这些用例把本轮源码审计的**结论**固定下来：
  * 外参符号：手册给的是“芯片在点云坐标中的位置”，FAST-LIO 要的是 t_IL = -R_IL r_LI；
  * IMU 单位：FAST-LIO 自己归一化，原始 g 与已换算 m/s^2 落到同一量级，本包不换算；
  * 时间：帧尾取 max offset_time，而不是数组末元素。
"""

from __future__ import annotations

import math

import pytest

from hei_robot_localization import lio_contract as C


# --------------------------------------------------------------------- 外参
def test_manual_chip_position_is_the_manual_value():
    assert C.MID360S_IMU_CHIP_POSITION_IN_LIDAR_M == pytest.approx(
        (0.011, 0.02329, -0.04412), abs=1e-12
    )


def test_extrinsic_t_is_negated_chip_position():
    """t_IL = -R_IL r_LI，R_IL = I 时必须逐轴取反。"""
    want = (-0.011, -0.02329, 0.04412)
    got = C.nominal_t_il_from_chip_position()
    assert got == pytest.approx(want, abs=1e-12)
    assert C.FASTLIO_NOMINAL_EXTRINSIC_T == pytest.approx(want, abs=1e-12)


def test_imu_origin_maps_to_zero_in_imu_frame():
    """把“IMU 芯片位置”作为 LiDAR 系中的一个点，变换后必须落在 IMU 系原点。"""
    p_i = C.lidar_point_to_imu(
        C.MID360S_IMU_CHIP_POSITION_IN_LIDAR_M, C.R_IL_NOMINAL, C.FASTLIO_NOMINAL_EXTRINSIC_T
    )
    assert p_i == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_round_trip_t_il_to_chip_position():
    back = C.imu_origin_in_lidar(C.FASTLIO_NOMINAL_EXTRINSIC_T, C.R_IL_NOMINAL)
    assert back == pytest.approx(C.MID360S_IMU_CHIP_POSITION_IN_LIDAR_M, abs=1e-12)


def test_non_identity_rotation_uses_matrix_not_elementwise():
    """R_IL 非单位阵时 t_IL = -R r_LI 必须真的乘矩阵（防止写成逐元素取反）。"""
    r = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))  # 绕 Z 转 +90°
    chip = (0.001, 0.002, 0.003)
    t = C.nominal_t_il_from_chip_position(chip, r)
    # R*chip = (-0.002, 0.001, 0.003)
    assert t == pytest.approx((-(-0.002), -(0.001), -(0.003)), abs=1e-12)
    p_i = C.lidar_point_to_imu(chip, r, t)
    assert p_i == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


# --------------------------------------------------------------------- 单位
def test_acc_scale_for_raw_g_and_si_agree_after_fastlio():
    """原始 g（范数 0.995）与 SI（范数 9.81）经 FAST-LIO 归一化后应一致。

    这就是“不要在适配器里再乘 9.81”的依据：FAST-LIO 已做 G/|mean_acc| 归一化。
    """
    acc_g = (0.0291, -0.0432, 0.9936)
    norm_g = C.norm3(acc_g)
    si_from_g = C.fastlio_si_acc(acc_g, norm_g)

    acc_si = tuple(v * C.STANDARD_GRAVITY for v in acc_g)
    norm_si = C.norm3(acc_si)
    si_from_si = C.fastlio_si_acc(acc_si, norm_si)

    assert si_from_g == pytest.approx(si_from_si, rel=5e-3)
    # 归一化后模长约等于 FAST-LIO 的 G_m_s2，而不是标准重力
    assert C.norm3(si_from_g) == pytest.approx(C.G_M_S2_FASTLIO, rel=1e-6)


def test_scale_is_inverse_of_mean_norm():
    assert C.fastlio_acc_scale(1.0) == pytest.approx(C.G_M_S2_FASTLIO)
    assert C.fastlio_acc_scale(C.G_M_S2_FASTLIO) == pytest.approx(1.0)


def test_scale_rejects_non_positive_norm():
    for bad in (0.0, -1.0, float("nan")):
        with pytest.raises(ValueError):
            C.fastlio_acc_scale(bad)


def test_fastlio_gravity_vector_only_depends_on_direction():
    raw_g = (0.0291, -0.0432, 0.9936)
    si = tuple(v * C.STANDARD_GRAVITY for v in raw_g)
    g1 = C.fastlio_gravity_vector(raw_g)
    g2 = C.fastlio_gravity_vector(si)
    assert g1 == pytest.approx(g2, abs=1e-9)
    assert C.norm3(g1) == pytest.approx(C.G_M_S2_FASTLIO, rel=1e-9)


def test_describe_acc_unit():
    assert C.describe_acc_unit(0.995) == "g"
    assert C.describe_acc_unit(9.81) == "m/s^2"
    assert C.describe_acc_unit(42.0) == "unknown"


def test_unit_contract_states_no_conversion_and_covariance_caveat():
    notes = C.unit_contract_notes(0.9949)
    assert notes["conversion_in_this_package"] == "none"
    assert notes["fastlio_acc_scale"] == pytest.approx(C.G_M_S2_FASTLIO / 0.9949)
    text = notes["explanation"]
    # 不允许把“协方差随单位自动换算”写成结论（:359 的缩放在 :362 被覆盖）
    assert "cov_acc" in text
    assert "覆盖" in text


# --------------------------------------------------------------------- 时间
def test_frame_end_uses_max_not_last():
    offsets = [0, 10, 20, 15, 30, 25]  # 末元素 25 不是最大值
    assert C.frame_end_offset_ns(offsets) == 30
    assert C.inversion_stats(offsets)["last_equals_max"] is False
    assert C.inversion_stats(offsets)["inversions"] == 2


def test_scan_end_adds_base_time():
    beg = 1792224601250394712
    offsets = [0, 100000000]
    assert C.scan_end_ns(beg, offsets) == beg + 100000000


def test_frame_end_empty_is_none():
    assert C.frame_end_offset_ns([]) is None
    assert C.scan_end_ns(0, []) is None


def test_inversion_stats_on_sorted_sequence():
    st = C.inversion_stats([0, 5, 10, 20])
    assert st["inversions"] == 0
    assert st["worst_regression_ns"] == 0
    assert st["last_equals_max"] is True


def test_frame_end_does_not_mutate_input():
    offsets = [5, 1, 9, 2]
    before = list(offsets)
    C.frame_end_offset_ns(offsets)
    C.inversion_stats(offsets)
    assert offsets == before


# --------------------------------------------------------------------- 静止判定
def test_static_window_thresholds():
    assert C.is_static_window((0.003, 0.006, 0.005), 0.012) is True
    assert C.is_static_window((0.003, 0.006, 0.5), 0.012) is False
    assert C.is_static_window((0.003, 0.006, 0.005), 0.5) is False


def test_specific_force_direction_is_up_and_gravity_is_negative():
    """静止时加速度计读数是比力：方向约 +up；重力单位向量必须显式取负。"""
    mean_acc = (0.0291, -0.0432, 0.9936)
    f = C.specific_force_unit_direction(mean_acc)
    g = C.gravity_unit_direction(mean_acc)
    assert C.norm3(f) == pytest.approx(1.0, rel=1e-9)
    assert C.norm3(g) == pytest.approx(1.0, rel=1e-9)
    assert f[2] > 0.99, "比力方向应接近 +Z（up）"
    assert g[2] < -0.99, "重力方向应接近 -Z"
    # 两者严格反号（点积 = -1），防止把比力当重力用
    assert sum(f[i] * g[i] for i in range(3)) == pytest.approx(-1.0, rel=1e-12)
    assert all(g[i] == pytest.approx(-f[i], abs=1e-15) for i in range(3))


def test_fastlio_init_gravity_is_negative_specific_force_times_g():
    """交叉验证：FAST-LIO 初始化 grav = -mean_acc/|mean_acc|*G_m_s2
    （IMU_Processing.hpp:193）应与 gravity_unit_direction * G_m_s2 同向同值。"""
    mean_acc = (0.0289, -0.0430, 0.9986)
    g_fastlio = C.fastlio_gravity_vector(mean_acc)
    g_unit = C.gravity_unit_direction(mean_acc)
    assert g_fastlio == pytest.approx(
        tuple(c * C.G_M_S2_FASTLIO for c in g_unit), abs=1e-12
    )
    f_unit = C.specific_force_unit_direction(mean_acc)
    assert g_fastlio == pytest.approx(
        tuple(-c * C.G_M_S2_FASTLIO for c in f_unit), abs=1e-12
    )


def test_known_unknowns_are_explicit():
    assert len(C.KNOWN_UNKNOWNS) >= 4
    joined = " ".join(C.KNOWN_UNKNOWNS)
    assert "UNKNOWN" in joined
    assert "外参" in joined and "时间同步" in joined


def test_fastlio_frames_are_upstream_hardcoded():
    assert C.FASTLIO_WORLD_FRAME == "camera_init"
    assert C.FASTLIO_BODY_FRAME == "body"
    assert C.IMU_FRAME_ID_UPSTREAM == "livox_frame"
