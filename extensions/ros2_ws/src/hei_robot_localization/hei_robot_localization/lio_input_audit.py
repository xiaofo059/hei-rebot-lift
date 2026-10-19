"""只读审计节点：核对 MID-360S 原始话题是否满足 FAST-LIO 输入契约。

设计约束（任务 HEI-LIDAR-20261017-03-LIO）
  * 不发布 TF、不发布点云/位姿、不改写任何消息、不做单位换算；
  * 不订阅也绝不参与硬件：本节点只用于**离线回放**时的输入核对；
  * 统计结果同时写结构化 JSON（--report-path）和一行 LIO_AUDIT_JSON 日志，
    便于把“实测输入”与“静态结论”分开取证。

它回答的问题是：“这份原始数据交给 FAST-LIO，单位/时间/frame 的实际形态是什么？”
它**不**回答“LIO 结果是否准确”，也不证明外参或轴符号。
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from hei_robot_localization import lio_contract as C
from hei_robot_localization.mid360s_lio_params import EXPERIMENT_METADATA

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError as exc:  # pragma: no cover - 依赖 underlay
    CustomMsg = None
    _CUSTOMMSG_IMPORT_ERROR = exc
else:
    _CUSTOMMSG_IMPORT_ERROR = None


class LioInputAudit(Node):
    """累计一个时间窗口，输出一次输入契约报告。"""

    def __init__(self) -> None:
        super().__init__("lio_input_audit")
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("window_seconds", 10.0)
        self.declare_parameter("report_path", "")
        self.declare_parameter("min_imu_norm_g", 0.5)
        self.declare_parameter("max_imu_norm_g", 1.5)

        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.window_seconds = float(self.get_parameter("window_seconds").value)
        self.report_path = str(self.get_parameter("report_path").value or "")

        if CustomMsg is None:
            raise RuntimeError(
                "无法导入 livox_ros_driver2.msg.CustomMsg：请先 source 冻结的 livox underlay"
                f"（{_CUSTOMMSG_IMPORT_ERROR}）"
            )

        # 点云每帧 ~2 万点、Python 侧统计较慢；IMU 200Hz 需要更深的队列，
        # 否则 2 倍速回放时会因单线程执行器排不过来而丢样本。
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2000,
        )
        self.create_subscription(CustomMsg, self.lidar_topic, self._on_lidar, lidar_qos)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, imu_qos)

        self._t0 = None
        self._done = False
        self._imu_n = 0
        self._imu_frames: set[str] = set()
        self._imu_stamps: list[int] = []
        self._imu_acc_sum = [0.0, 0.0, 0.0]
        self._imu_acc_sq = [0.0, 0.0, 0.0]
        self._imu_gyr_sum = [0.0, 0.0, 0.0]
        self._imu_gyr_norm_sum = 0.0
        self._orientations: set[tuple] = set()
        self._imu_norm_sum = 0.0
        self._lidar_n = 0
        self._lidar_frames: set[str] = set()
        self._lidar_stamps: list[int] = []
        self._lidar_points = 0
        self._lidar_inversions = 0
        self._lidar_worst = 0
        self._lidar_last_eq_max = 0
        self._lidar_header_eq_timebase = 0
        self._lidar_max_offset = 0
        self.create_timer(1.0, self._tick)

    # ---------------------------------------------------------------- 回调
    def _on_imu(self, msg: Imu) -> None:
        if self._done:
            return
        if self._t0 is None:
            self._t0 = time.monotonic()
        self._imu_n += 1
        self._imu_frames.add(msg.header.frame_id)
        self._imu_stamps.append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        g = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        for i in range(3):
            self._imu_acc_sum[i] += a[i]
            self._imu_acc_sq[i] += a[i] * a[i]
            self._imu_gyr_sum[i] += g[i]
        self._imu_norm_sum += C.norm3(a)
        self._imu_gyr_norm_sum += C.norm3(g)
        self._orientations.add(
            (
                round(msg.orientation.x, 9),
                round(msg.orientation.y, 9),
                round(msg.orientation.z, 9),
                round(msg.orientation.w, 9),
            )
        )

    def _on_lidar(self, msg: CustomMsg) -> None:
        if self._done:
            return
        if self._t0 is None:
            self._t0 = time.monotonic()
        self._lidar_n += 1
        self._lidar_frames.add(msg.header.frame_id)
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._lidar_stamps.append(stamp)
        if stamp == int(msg.timebase):
            self._lidar_header_eq_timebase += 1
        offsets = [int(p.offset_time) for p in msg.points]
        self._lidar_points += len(offsets)
        st = C.inversion_stats(offsets)
        self._lidar_inversions += st["inversions"]
        self._lidar_worst = max(self._lidar_worst, st["worst_regression_ns"])
        if st["last_equals_max"]:
            self._lidar_last_eq_max += 1
        if st["max_offset_ns"] is not None:
            self._lidar_max_offset = max(self._lidar_max_offset, st["max_offset_ns"])

    # ---------------------------------------------------------------- 报告
    def _tick(self) -> None:
        if self._done or self._t0 is None:
            return
        if time.monotonic() - self._t0 < self.window_seconds:
            return
        self._done = True
        report = self._build_report()
        text = json.dumps(report, ensure_ascii=False, indent=2)
        if self.report_path:
            try:
                with open(self.report_path, "w", encoding="utf-8") as fh:
                    fh.write(text + "\n")
                self.get_logger().info("审计报告已写入 " + self.report_path)
            except OSError as exc:
                self.get_logger().error("审计报告写入失败: " + str(exc))
        self.get_logger().info("LIO_AUDIT_JSON=" + json.dumps(report, ensure_ascii=False))

    def _build_report(self) -> dict:
        n = self._imu_n
        acc_mean = [v / n for v in self._imu_acc_sum] if n else [0.0, 0.0, 0.0]
        acc_std = []
        for i in range(3):
            if n > 1:
                var = max(0.0, self._imu_acc_sq[i] / n - acc_mean[i] ** 2)
            else:
                var = 0.0
            acc_std.append(math.sqrt(var))
        gyro_mean = [v / n for v in self._imu_gyr_sum] if n else [0.0, 0.0, 0.0]
        acc_norm_mean = (self._imu_norm_sum / n) if n else float("nan")
        gyro_norm_mean = (self._imu_gyr_norm_sum / n) if n else float("nan")

        lidar_monotonic = all(
            b > a for a, b in zip(self._lidar_stamps, self._lidar_stamps[1:])
        )
        imu_monotonic = all(b > a for a, b in zip(self._imu_stamps, self._imu_stamps[1:]))

        return {
            "kind": "mid360s_lio_input_audit",
            "window_seconds_param": self.window_seconds,
            "topics": {"lidar": self.lidar_topic, "imu": self.imu_topic},
            "imu": {
                "count": n,
                "frame_ids": sorted(self._imu_frames),
                "acc_mean_raw": acc_mean,
                "acc_std_raw": acc_std,
                "acc_norm_mean_raw": acc_norm_mean,
                "gyro_mean_raw": gyro_mean,
                "gyro_norm_mean_raw": gyro_norm_mean,
                "orientation_unique_count": len(self._orientations),
                "orientation_all_default": self._orientations == {(0.0, 0.0, 0.0, 1.0)},
                "stamp_monotonic": imu_monotonic,
                "static_window": C.is_static_window(acc_std, gyro_norm_mean),
                # 比力方向（≈ +up）。名称写死，避免被当成重力向量用。
                "specific_force_unit_direction": (
                    C.specific_force_unit_direction(acc_mean) if n else None
                ),
                # 重力 = 负比力方向，显式取负后再报告
                "gravity_unit_direction": (
                    C.gravity_unit_direction(acc_mean) if n else None
                ),
            },
            "lidar": {
                "frames": self._lidar_n,
                "frame_ids": sorted(self._lidar_frames),
                "points": self._lidar_points,
                "header_equals_timebase_frames": self._lidar_header_eq_timebase,
                "stamp_monotonic": lidar_monotonic,
                "per_point_inversions": self._lidar_inversions,
                "worst_regression_ns": self._lidar_worst,
                "frames_last_point_equals_max": self._lidar_last_eq_max,
                "max_offset_ns": self._lidar_max_offset,
            },
            "direction_convention": {
                "specific_force_unit_direction": "加速度计读数的单位方向（比力），静止时约指向 +up",
                "gravity_unit_direction": "重力单位向量 = -specific_force_unit_direction（显式取负）",
                "why": "静止时 f = a - g ≈ -g，所以比力方向与重力方向相反；把比力当重力用会整体反号",
            },
            "unit_contract": C.unit_contract_notes(acc_norm_mean),
            "extrinsic_contract": EXPERIMENT_METADATA,
            "tf_contract": {
                "this_package_publishes_tf": False,
                "fastlio_publishes_tf": True,
                "fastlio_frames": [C.FASTLIO_WORLD_FRAME, C.FASTLIO_BODY_FRAME],
                "base_link_published": False,
                "map_published": False,
            },
            "count_interpretation": (
                "count 只表示本节点实际收到的样本数，受 rclpy 单线程调度与回放倍速影响，"
                "不是频率证据；话题频率以 02-LIVE 全量 QC 与 bag 元数据为准。"
            ),
            "known_unknowns": list(C.KNOWN_UNKNOWNS),
            "limitations": (
                "单一静止窗口只能给出量纲/方向/量级的粗核，不能证明全部轴符号、偏航或"
                "IMU-LiDAR 外参；也不代表任何运动里程计精度。"
            ),
        }


def spin_loop(node, spin_once, ok) -> None:
    """带关停语义的 spin 循环（spin_once / ok 可注入，便于离线单测）。

    规则（不允许无条件吞异常）：
      * KeyboardInterrupt 属于 SIGINT 停止路径，直接抛给调用方处理；
      * 其它异常：若 ROS context 仍在运行（ok() 为真）-> **必须传播**，
        否则运行期故障会被伪装成“正常退出”；
      * 只有 ok() 已为假（context 已被关停，典型是 SIGINT 关停窗口内
        take_message 失败）才允许降级返回，并记录一条警告。
    """
    while ok():
        try:
            spin_once(node)
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001 - 仅在关停窗口降级
            if ok():
                raise
            node.get_logger().warn(
                "上下文已关停，审计循环退出（不影响已写出的报告）: "
                + type(exc).__name__
                + ": "
                + str(exc)
            )
            return


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = LioInputAudit()
    try:
        spin_loop(
            node,
            lambda n: rclpy.spin_once(n, timeout_sec=0.2),
            rclpy.ok,
        )
    except KeyboardInterrupt:
        # 用户/launch 的 SIGINT：正常停止路径
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # SIGINT 打断销毁同样属于正常停止，不掩盖其它异常
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
