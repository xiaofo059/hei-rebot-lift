"""RTAB-Map XYZI 最薄字段适配节点（不发 TF、不 deskew、不累计、不降采样）。

订阅 FAST-LIO 的去畸变单帧 body 点云，投影为固定 XYZI 字段后原样转发：

* 输入：/cloud_registered_body（frame=body，stamp=lidar end time，与 /Odometry 同 stamp）
* 输出：/slam/lidar_cloud_xyzi（field=x,y,z,intensity，stamp/frame 原样）
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from hei_robot_slam.cloud_xyzi import XYZI_FIELD_SPEC, XYZI_POINT_STEP, convert


def _qos(reliability: str, depth: int = 10) -> QoSProfile:
    policy = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }.get(reliability, ReliabilityPolicy.SYSTEM_DEFAULT)
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=policy,
        durability=DurabilityPolicy.VOLATILE,
    )


class CloudAdapter(Node):
    """唯一职责：把点云字段收敛为 XYZI，其余语义（stamp/frame/点数）保持。"""

    def __init__(self) -> None:
        super().__init__("slam_cloud_adapter")
        self.declare_parameter("input_qos", "reliable")
        self.declare_parameter("report_period_s", 5.0)
        self._input_qos = str(self.get_parameter("input_qos").value)
        self._report_period = max(0.5, float(self.get_parameter("report_period_s").value))

        self._sub = self.create_subscription(
            PointCloud2, "input_cloud", self._callback, _qos(self._input_qos, 20)
        )
        self._pub = self.create_publisher(PointCloud2, "output_cloud", _qos("reliable", 20))

        self._frames = 0
        self._in_points = 0
        self._out_points = 0
        self._dropped = 0
        self._normal_frames = 0
        self._nonzero_normal_points = 0
        self._last_stamp_ns = None
        self._stamp_regressions = 0
        self._last_fields: list[str] = []
        self._timer = self.create_timer(self._report_period, self._report)
        self.get_logger().info(
            "slam_cloud_adapter 启动：input_cloud(任意字段) -> output_cloud(XYZI)；"
            f" 输入 QoS={self._input_qos}；不做 TF/deskew/累计/降采样"
        )

    def _callback(self, msg: PointCloud2) -> None:
        result = convert(msg)
        stats = result["stats"]

        out = PointCloud2()
        out.header = msg.header
        out.height = int(result["height"])
        out.width = int(result["width"])
        out.fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=count)
            for (name, offset, datatype, count) in XYZI_FIELD_SPEC
        ]
        out.is_bigendian = False
        out.point_step = XYZI_POINT_STEP
        out.row_step = XYZI_POINT_STEP * out.width
        out.data = result["data"]
        out.is_dense = bool(stats["dropped_nonfinite"] == 0 and getattr(msg, "is_dense", True))
        self._pub.publish(out)

        self._frames += 1
        self._in_points += int(stats["input_points"])
        self._out_points += int(stats["output_points"])
        self._dropped += int(stats["dropped_nonfinite"])
        if stats["input_has_normal_fields"]:
            self._normal_frames += 1
            self._nonzero_normal_points += int(stats["input_nonzero_normal_points"])
        self._last_fields = list(stats["source_fields"])

        stamp_ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            self._stamp_regressions += 1
        self._last_stamp_ns = stamp_ns

    def _report(self) -> None:
        normal_state = "有" if "normal_x" in self._last_fields else "无"
        self.get_logger().info(
            f"adapter frames={self._frames} in_points={self._in_points} "
            f"out_points={self._out_points} dropped_nonfinite={self._dropped} "
            f"frames_with_normal_fields={self._normal_frames} "
            f"nonzero_normal_points={self._nonzero_normal_points} "
            f"stamp_regressions={self._stamp_regressions} "
            f"source_fields=[{chr(44).join(self._last_fields)}] "
            f"source_normal_fields={normal_state}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
