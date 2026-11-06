#!/usr/bin/python3
"""A lightweight Trigger service around RTAB-Map's localization interfaces."""
import math
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener


class Relocalizer(Node):
    def __init__(self):
        super().__init__("relocalizer")
        defaults = {
            "database_path": "", "initial_pose": [0.0] * 6,
            "auto_relocalize": True, "service_timeout_sec": 15.0,
            "localization_timeout_sec": 60.0, "required_matches": 3,
            "data_timeout_sec": 3.0, "rtabmap_node": "/rtabmap/rtabmap",
            "map_frame": "map", "odom_frame": "odom", "body_frame": "body",
            "initialpose_topic": "/initialpose", "info_topic": "/rtabmap/info",
            "localization_pose_topic": "/rtabmap/localization_pose",
        }
        self.config = {name: self.declare_parameter(
            name, value, ParameterDescriptor(read_only=True)).value
            for name, value in defaults.items()}
        self.database = Path(self.config["database_path"]).expanduser().resolve()
        if not self.database.is_file():
            raise ValueError(f"Saved DB does not exist: {self.database}")
        pose = self.config["initial_pose"]
        if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
            raise ValueError("initial_pose needs six finite numbers (m, rad)")
        for key in ("service_timeout_sec", "localization_timeout_sec", "data_timeout_sec"):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if self.config["required_matches"] < 1:
            raise ValueError("required_matches must be positive")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, "localization_ready", qos)
        self.seed_pub = self.create_publisher(PoseWithCovarianceStamped,
                                               self.config["initialpose_topic"], 1)
        self.create_subscription(Info, self.config["info_topic"], self._info, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.config["localization_pose_topic"], self._pose, 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        target = self.config["rtabmap_node"].rstrip("/")
        self.parameters_client = self.create_client(GetParameters, target + "/get_parameters")
        self.mode_client = self.create_client(Empty, target + "/set_mode_localization")
        self.create_service(Trigger, "relocalize", self._request,
                            callback_group=ReentrantCallbackGroup())
        self.auto_client = self.create_client(Trigger, "relocalize")
        self.pending = None
        self.phase = "idle"
        self.generation = 0
        self.ready = False
        self.armed = False
        self.good = 0
        self.seed_stamp = 0
        self.last_stamp = 0
        self.last_match = 0.0
        self.infos = {}
        self.poses = {}
        self.auto_pending = self.config["auto_relocalize"]
        self._set_ready(False, force=True)
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Relocalize service ready; configured DB: {self.database}")

    def _set_ready(self, value, force=False):
        if force or value != self.ready:
            self.ready = value
            self.ready_pub.publish(Bool(data=value))

    async def _request(self, _request, response):
        if self.pending is not None:
            response.success = False
            response.message = "Relocalization is already in progress."
            return response
        self.generation += 1
        self.pending = Future()
        future = self.pending
        self.armed = False
        self.good = 0
        self.infos.clear()
        self.poses.clear()
        self._set_ready(False, force=True)
        self.phase = "backend"
        self.deadline = time.monotonic() + self.config["service_timeout_sec"]
        self.get_logger().info("Relocalization requested; checking read-only RTAB-Map backend.")
        response.success, response.message = await future
        return response

    def _finish(self, success, message):
        self.phase = "idle"
        future, self.pending = self.pending, None
        if not success:
            self.armed = False
            self._set_ready(False)
        if future is not None:
            future.set_result((success, message))
        logger = self.get_logger().info if success else self.get_logger().error
        logger(message)

    def _parameters_done(self, future, generation):
        if generation != self.generation or self.pending is None:
            return
        try:
            values = future.result().values
            database, incremental, read_only, body, world = [v.string_value for v in values]
            if (Path(database).expanduser().resolve() != self.database
                    or incremental != "false" or read_only != "true"
                    or body != self.config["body_frame"] or world != self.config["map_frame"]):
                raise ValueError("Backend DB/frames/read-only localization parameters do not match; "
                                 "stop mapping and use relocalize.launch.py.")
            self.phase = "mode"
            future = self.mode_client.call_async(Empty.Request())
            future.add_done_callback(lambda f: self._mode_done(f, generation))
        except Exception as error:
            self._finish(False, f"RTAB-Map backend check failed: {error}")

    def _mode_done(self, future, generation):
        if generation != self.generation or self.pending is None:
            return
        try:
            future.result()
            self.phase = "seed"
        except Exception as error:
            self._finish(False, f"Cannot select localization mode: {error}")

    def _tick(self):
        if self.auto_pending and self.auto_client.service_is_ready():
            self.auto_pending = False
            self.auto_client.call_async(Trigger.Request())
        now = time.monotonic()
        if self.good and now - self.last_match > self.config["data_timeout_sec"]:
            self.good = 0
            was_ready = self.ready
            self._set_ready(False)
            if was_ready:
                self.get_logger().warn("Localization matches are stale; localization_ready=false.")
        if self.pending is None:
            return
        if now > self.deadline:
            self._finish(False, "Relocalization timed out: no confirmed fresh map registration. "
                         "Check FAST-LIO2, initial pose and TF; correct the YAML and restart if needed.")
        elif self.phase == "backend" and self.mode_client.service_is_ready() and self.parameters_client.service_is_ready():
            self.phase = "parameters"
            request = GetParameters.Request(names=["database_path", "Mem/IncrementalMemory",
                        "Mem/LocalizationReadOnly", "frame_id", "map_frame_id"])
            generation = self.generation
            self.parameters_client.call_async(request).add_done_callback(
                lambda f: self._parameters_done(f, generation))
        elif self.phase == "seed" and self.seed_pub.get_subscription_count() > 0:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.config["map_frame"]
            msg.header.stamp = self.get_clock().now().to_msg()
            x, y, z, roll, pitch, yaw = self.config["initial_pose"]
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            q = msg.pose.pose.orientation
            q.x, q.y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
            q.z, q.w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
            self.seed_stamp = self._stamp(msg)
            self.last_stamp = self.seed_stamp
            self.seed_pub.publish(msg)
            self.armed = True
            self.phase = "matching"
            self.deadline = now + self.config["localization_timeout_sec"]
            self.get_logger().info("Initial pose sent; waiting for accepted registrations and map -> odom -> body TF.")

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _fresh(self, msg):
        stamp = self._stamp(msg)
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > self.seed_stamp and -0.5 <= age <= self.config["data_timeout_sec"]

    def _info(self, msg):
        if not self.armed or not self._fresh(msg):
            return
        rejected = dict(zip(msg.stats_keys, msg.stats_values)).get("Loop/RejectedHypothesis/", 0.0)
        if (msg.loop_closure_id <= 0 and msg.proximity_detection_id <= 0) or rejected > 0:
            self.good = 0
            self.last_stamp = max(self.last_stamp, self._stamp(msg))
            self._set_ready(False)
            return
        self.infos[self._stamp(msg)] = msg
        self._pair(self._stamp(msg))

    def _pose(self, msg):
        if self.armed and self._fresh(msg):
            self.poses[self._stamp(msg)] = msg
            self._pair(self._stamp(msg))

    def _pair(self, stamp):
        # Bound caches even if one topic stops, with no point-cloud subscription.
        for cache in (self.infos, self.poses):
            while len(cache) > 16:
                del cache[min(cache)]
        if stamp not in self.infos or stamp not in self.poses or stamp <= self.last_stamp:
            return
        info, pose = self.infos.pop(stamp), self.poses.pop(stamp)
        position, q = pose.pose.pose.position, pose.pose.pose.orientation
        values = [position.x, position.y, position.z, q.x, q.y, q.z, q.w]
        valid = (pose.header.frame_id == self.config["map_frame"]
                 and all(math.isfinite(v) for v in values + list(pose.pose.covariance))
                 and abs(sum(v * v for v in [q.x, q.y, q.z, q.w]) - 1.0) < 0.01
                 and all(0 <= pose.pose.covariance[i] < 9999 for i in (0, 7, 14, 21, 28, 35)))
        scan_time = Time.from_msg(info.header.stamp)
        valid = valid and self.tf.can_transform(self.config["map_frame"], self.config["odom_frame"], scan_time)
        valid = valid and self.tf.can_transform(self.config["odom_frame"], self.config["body_frame"], scan_time)
        self.last_stamp = stamp
        if not valid:
            self.good = 0
            self._set_ready(False)
            return
        self.good += 1
        self.last_match = time.monotonic()
        if self.good >= self.config["required_matches"]:
            self._set_ready(True)
            if self.pending is not None:
                self._finish(True, "RTAB-Map localization confirmed: accepted registrations and TF are fresh. "
                             "Verify scan/map alignment in RViz before enabling navigation.")


def main():
    rclpy.init()
    node = None
    try:
        node = Relocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
