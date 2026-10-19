"""MID-360S -> FAST-LIO2 输入契约的唯一事实来源（纯 Python，可离线单测）。

本模块只做算术与判定，不发 TF、不改单位、不持有 ROS 句柄；节点与测试共用同一份实现，
避免“文档一套、代码一套”。

本轮源码审计基线（2026-10-17，行号对应该 SHA）：

FAST_LIO_ROS2 @ 2fffc570a25d0df172720bac034fbdb6a13d2162（branch ros2）
  src/laserMapping.cpp:311-348   livox_pcl_cbk  —— CustomMsg 入口，只用 header.stamp
  src/laserMapping.cpp:383-435   sync_packages  —— 帧尾与 IMU 配对
  src/laserMapping.cpp:921-929   lidar_type==AVIA 时订阅 livox_ros_driver2/CustomMsg
  src/laserMapping.cpp:628-659   publish_odometry —— TF: camera_init -> body
  src/IMU_Processing.hpp:156-211 IMU_init：mean_acc 只用于重力方向
  src/IMU_Processing.hpp:260     acc_avr = acc_avr * G_m_s2 / mean_acc.norm()
  src/IMU_Processing.hpp:225     按 curvature(ms) 排序后才去畸变

livox_ros_driver2 @ 13eb05e4e6dd7a765b934d0c5fd6236676a57b49（1.2.6，冻结 + tracked patch）
  src/lddc.cpp:395                offset_time = 点绝对时间 - 帧 base_time（ns）
  src/lddc.cpp:481                IMU header.frame_id 硬编码 "livox_frame"
  src/lddc.cpp:489-494            IMU 字段原样写入（acc=g、gyro=rad/s、无 orientation）
  src/comm/lidar_imu_data_queue.h:37-42  acc 单位 g / gyro 单位 rad/s 的源码自述
  src/comm/pub_handler.cpp:404/432/467   逐点时间 = 包时间 + i*间隔；按 UDP 到达序追加

Livox Mid-360S User Manual (2026-06-01) §3.4 p.18 / §3.5 "IMU Data" p.22
  * 点云坐标 O-XYZ 定义于 p.18；
  * "The IMU data contains the 3-axis acceleration and 3-axis angular velocity,
    the directions of which are the same with that of the point cloud
    coordinates. The position of the IMU chip in the point cloud coordinates is
    x=11.0 mm, y=23.29 mm, z=-44.12 mm."
  * 默认 200 Hz 推 IMU。
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

#: FAST-LIO 源码里的重力常量（include/common_lib.h:18 G_m_s2 (9.81)）。
#: 它用于 IMU 初始化与加速度归一化，不是 CODATA 的 9.80665。
G_M_S2_FASTLIO = 9.81
#: 标准重力加速度（仅在需要把 g 换算成 SI 的工具里使用）。
STANDARD_GRAVITY = 9.80665

#: 原始话题的事实（driver 冻结源码，不是本包发明）
IMU_LINEAR_ACCELERATION_UNIT = "g"
IMU_ANGULAR_VELOCITY_UNIT = "rad/s"
IMU_ORIENTATION_PUBLISHED = False
CUSTOM_POINT_OFFSET_UNIT = "ns"
LIDAR_POINT_UNIT = "m"

#: 上游硬编码 frame（不因 launch 参数改变）
IMU_FRAME_ID_UPSTREAM = "livox_frame"
#: 本仓库 bringup 配置的点云 frame（见 hei_robot_bringup.mid360s_config）
LIDAR_FRAME_ID_DEFAULT = "lidar_link"

#: FAST-LIO 输出的 frame（源码硬编码，只能改话题名不能改 frame 名）
FASTLIO_WORLD_FRAME = "camera_init"
FASTLIO_BODY_FRAME = "body"

#: 厂商名义值：MID-360S 手册 §3.5 的 IMU 芯片位置（在点云坐标中表达），单位 m。
#: 这是 manufacturer nominal，不是本机现场标定结果。
MID360S_IMU_CHIP_POSITION_IN_LIDAR_M: tuple[float, float, float] = (
    11.0e-3,
    23.29e-3,
    -44.12e-3,
)

#: 手册原文 "directions ... are the same with that of the point cloud
#: coordinates" => R_IL = I（IMU 系到 LiDAR/点云系无旋转）。
R_IL_NOMINAL: tuple[tuple[float, float, float], ...] = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

IDENTITY3: tuple[tuple[float, float, float], ...] = R_IL_NOMINAL


# ---------------------------------------------------------------------------
# 极小的 3x3 矩阵算术（避免为纯函数引入 numpy 依赖）
# ---------------------------------------------------------------------------

def mat_vec(m: Sequence[Sequence[float]], v: Sequence[float]) -> tuple[float, float, float]:
    """3x3 矩阵乘向量。"""
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def mat_transpose(m: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def norm3(v: Sequence[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


# ---------------------------------------------------------------------------
# 外参：FAST-LIO 的 extrinsic_T / extrinsic_R 语义
# ---------------------------------------------------------------------------

def lidar_point_to_imu(
    p_l: Sequence[float],
    r_il: Sequence[Sequence[float]] = R_IL_NOMINAL,
    t_il: Sequence[float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float]:
    """p_I = R_IL * p_L + t_IL。

    FAST-LIO 对每个点正是这么用的（IMU_Processing.hpp:327 里的
    offset_R_L_I * P_i + offset_T_L_I；laserMapping.cpp:895-897 用
    extrinsic_T / extrinsic_R 初始化 Lidar_T_wrt_IMU / Lidar_R_wrt_IMU）。
    """
    rp = mat_vec(r_il, p_l)
    return (rp[0] + t_il[0], rp[1] + t_il[1], rp[2] + t_il[2])


def nominal_t_il_from_chip_position(
    chip_position: Sequence[float] = MID360S_IMU_CHIP_POSITION_IN_LIDAR_M,
    r_il: Sequence[Sequence[float]] = R_IL_NOMINAL,
) -> tuple[float, float, float]:
    """由“IMU 芯片在点云坐标中的位置” r_LI 求 FAST-LIO 的 extrinsic_T = t_IL。

    r_LI 是 IMU 原点在 LiDAR 系中的位置；t_IL 是 LiDAR 原点在 IMU 系中的位置：
        p_I = R_IL (p_L - r_LI)  =>  t_IL = -R_IL * r_LI
    因此把手册的 [+0.011, +0.02329, -0.04412] m 直接写进 extrinsic_T 是错的，
    R_IL = I 时必须取反（得到 [-0.011, -0.02329, +0.04412]）。
    """
    rr = mat_vec(r_il, chip_position)
    return (-rr[0], -rr[1], -rr[2])


def imu_origin_in_lidar(
    t_il: Sequence[float], r_il: Sequence[Sequence[float]] = R_IL_NOMINAL
) -> tuple[float, float, float]:
    """t_IL 的逆：r_LI = -R_IL^T * t_IL（用于回归测试方向是否写反）。"""
    rt = mat_transpose(r_il)
    rr = mat_vec(rt, t_il)
    return (-rr[0], -rr[1], -rr[2])


#: 由 S 型手册推导、与上游 config/mid360.yaml 一致的 FAST-LIO 外参（名义值）。
FASTLIO_NOMINAL_EXTRINSIC_T: tuple[float, float, float] = nominal_t_il_from_chip_position()
FASTLIO_NOMINAL_EXTRINSIC_R: tuple[float, ...] = (
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
)


# ---------------------------------------------------------------------------
# IMU 单位：FAST-LIO 自己把加速度归一到 1g，本包不做换算
# ---------------------------------------------------------------------------

def fastlio_acc_scale(mean_acc_norm: float) -> float:
    """FAST-LIO 对加速度施加的比例因子 G_m_s2 / |mean_acc|（IMU_Processing.hpp:260）。"""
    if not math.isfinite(mean_acc_norm) or not (mean_acc_norm > 0.0):
        raise ValueError("mean_acc_norm 必须为正有限数，实际 " + repr(mean_acc_norm))
    return G_M_S2_FASTLIO / mean_acc_norm


def fastlio_si_acc(
    acc_raw: Sequence[float], mean_acc_norm: float
) -> tuple[float, float, float]:
    """FAST-LIO 内部实际使用的加速度（SI, m/s^2）。

    mean_acc_norm 是初始化窗口里 |acc| 的均值，单位跟随输入话题。
    """
    s = fastlio_acc_scale(mean_acc_norm)
    return (acc_raw[0] * s, acc_raw[1] * s, acc_raw[2] * s)


def fastlio_gravity_vector(mean_acc: Sequence[float]) -> tuple[float, float, float]:
    """FAST-LIO 初始化重力：grav = -mean_acc/|mean_acc| * G_m_s2（IMU_Processing.hpp:193）。

    只依赖方向，因此输入是 g 还是 m/s^2 都不影响 gravity 的模。
    """
    n = norm3(mean_acc)
    if not (n > 0.0):
        raise ValueError("mean_acc 不能是零向量")
    return (
        -mean_acc[0] / n * G_M_S2_FASTLIO,
        -mean_acc[1] / n * G_M_S2_FASTLIO,
        -mean_acc[2] / n * G_M_S2_FASTLIO,
    )


def describe_acc_unit(acc_norm_mean: float) -> str:
    """按 |acc| 均值判断话题里加速度的量纲（仅供审计输出，不用于换算）。"""
    if not math.isfinite(acc_norm_mean):
        return "unknown"
    if 0.5 <= acc_norm_mean <= 1.5:
        return "g"
    if 5.0 <= acc_norm_mean <= 15.0:
        return "m/s^2"
    return "unknown"


def unit_contract_notes(acc_norm_mean: float) -> dict:
    """把“输入单位 + 算法如何处理”写成可机读的结论。"""
    unit = describe_acc_unit(acc_norm_mean)
    scale = fastlio_acc_scale(acc_norm_mean) if acc_norm_mean > 0 else None
    return {
        "input_acc_unit_detected": unit,
        "driver_declared_acc_unit": IMU_LINEAR_ACCELERATION_UNIT,
        "gyro_unit": IMU_ANGULAR_VELOCITY_UNIT,
        "orientation_valid": IMU_ORIENTATION_PUBLISHED,
        "fastlio_acc_scale": scale,
        "conversion_in_this_package": "none",
        "explanation": (
            "FAST-LIO 在 IMU 初始化后用 |mean_acc| 把加速度归一到 G_m_s2=9.81"
            "（IMU_Processing.hpp:260），重力方向也只取 mean_acc 的方向（:193）。"
            "因此原始 g 与已换算的 m/s^2 都会落到同一量级；本包不做 g->m/s^2 换算，"
            "避免重复转换，也不把原始消息标成 SI。注意 cov_acc 的 G/|mean_acc| 缩放"
            "随后被 cov_acc_scale 覆盖（:359 与 :362），所以不能声称协方差随单位自动换算。"
        ),
    }


# ---------------------------------------------------------------------------
# 时间：CustomMsg 逐点 offset_time 与帧尾
# ---------------------------------------------------------------------------

def frame_end_offset_ns(offsets: Iterable[int]) -> int | None:
    """帧尾参考：所有保留点 offset_time 的最大值（ns）。

    FAST-LIO 原本用 points.back().curvature（laserMapping.cpp:399/406/407），
    而 driver 只按 UDP 到达序追加点（pub_handler.cpp:404/432/467），末点不保证最大。
    本仓库的 patch 0002 改为对实际保留点求 max。
    """
    it = iter(offsets)
    try:
        best = int(next(it))
    except StopIteration:
        return None
    for v in it:
        iv = int(v)
        if iv > best:
            best = iv
    return best


def scan_end_ns(beg_ns: int, offsets: Iterable[int]) -> int | None:
    """扫描结束时刻 = 帧头(base) + max(offset_time)。"""
    off = frame_end_offset_ns(offsets)
    if off is None:
        return None
    return int(beg_ns) + off


def inversion_stats(offsets: Sequence[int]) -> dict:
    """逐点 offset_time 的乱序统计（相邻逆序次数、单次最大回退）。"""
    inv = 0
    worst = 0
    prev = None
    for v in offsets:
        iv = int(v)
        if prev is not None and iv < prev:
            inv += 1
            worst = max(worst, prev - iv)
        prev = iv
    mx = frame_end_offset_ns(offsets)
    return {
        "points": len(offsets),
        "inversions": inv,
        "worst_regression_ns": worst,
        "last_offset_ns": int(offsets[-1]) if offsets else None,
        "max_offset_ns": mx,
        "last_equals_max": bool(offsets) and int(offsets[-1]) == mx,
    }


# ---------------------------------------------------------------------------
# 静止窗口判定（只用于说明“单一姿态不能证明轴符号/偏航/外参")
# ---------------------------------------------------------------------------

def is_static_window(
    acc_std: Sequence[float],
    gyro_norm_mean: float,
    acc_std_max: float = 0.05,
    gyro_norm_max: float = 0.05,
) -> bool:
    """粗判静止（阈值是本仓库的实验口径，不是厂商规格）。"""
    return max(abs(s) for s in acc_std) <= acc_std_max and abs(gyro_norm_mean) <= gyro_norm_max


def specific_force_unit_direction(mean_acc: Sequence[float]) -> tuple[float, float, float]:
    """静止时加速度计读数的**单位方向**：这是比力（specific force），约指向 +up。

    符号约定（极易误用，故写死在函数名里）：
      * 加速度计测的是比力 f = a - g，静止时 f ≈ -g，方向指向 +up；
      * **重力单位向量是它的相反数**，见 gravity_unit_direction()；
      * FAST-LIO 初始化用的正是 "grav = -mean_acc/|mean_acc| * G"（IMU_Processing.hpp:193），
        也就是先取这里的比力方向再取负，本函数**不**代替那一步。
    """
    n = norm3(mean_acc)
    if not (n > 0.0):
        raise ValueError("mean_acc 不能是零向量")
    return (mean_acc[0] / n, mean_acc[1] / n, mean_acc[2] / n)


def gravity_unit_direction(mean_acc: Sequence[float]) -> tuple[float, float, float]:
    """重力单位向量 = -比力方向（显式取负，避免把比力方向当重力用）。"""
    d = specific_force_unit_direction(mean_acc)
    return (-d[0], -d[1], -d[2])


KNOWN_UNKNOWNS: tuple[str, ...] = (
    "IMU 与 LiDAR 的现场标定外参（R_IL 偏航与 t_IL 实测值）：UNKNOWN，需现场标定/多姿态数据",
    "IMU 轴符号与安装方向：名义同向，单一静止姿态不足以证明，UNKNOWN",
    "IMU 与 LiDAR 的硬件时间同步精度（NoSync 为主机 epoch+steady 映射，非 PTP）：UNKNOWN",
    "IMU 噪声/随机游走与 bias 稳定性：UNKNOWN（未做 Allan 方差）",
    "base_link 与 body/IMU 的安装外参：UNKNOWN（本轮不发布任何 base_link 变换）",
)
