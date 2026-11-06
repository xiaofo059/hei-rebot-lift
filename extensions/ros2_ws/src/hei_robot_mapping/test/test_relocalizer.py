"""Isolated ROS service/registration tests; no robot or point-cloud inputs."""
import importlib.util
import math
from pathlib import Path
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from std_srvs.srv import Empty, Trigger
from tf2_ros import TransformBroadcaster

_spec = importlib.util.spec_from_file_location(
    "rtabmap_relocalizer", Path(__file__).parents[1] / "scripts" / "rtabmap_relocalizer.py")
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)


class Backend(Node):
    def __init__(self, database):
        super().__init__("rtabmap", namespace="/rtabmap", use_global_arguments=False)
        for name, value in {"database_path": str(database), "Mem/IncrementalMemory": "false",
                            "Mem/LocalizationReadOnly": "true", "frame_id": "body",
                            "map_frame_id": "map"}.items():
            self.declare_parameter(name, value)
        self.create_service(Empty, "/rtabmap/rtabmap/set_mode_localization", lambda req, res: res)
        self.seeds = []
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.seeds.append, 10)
        self.info = self.create_publisher(Info, "/rtabmap/info", 10)
        self.pose = self.create_publisher(PoseWithCovarianceStamped, "/rtabmap/localization_pose", 10)
        self.transforms = TransformBroadcaster(self)


class Harness:
    def __init__(self, directory, monkeypatch):
        monkeypatch.setenv("ROS_DOMAIN_ID", "216")
        monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
        database = directory / "test map.db"
        database.touch()
        rclpy.init(args=["--ros-args", "-r", "__ns:=/hei_robot_mapping",
                        "-p", f"database_path:={database}", "-p", "auto_relocalize:=false",
                        "-p", "service_timeout_sec:=5.0", "-p", "localization_timeout_sec:=2.0",
                        "-p", "data_timeout_sec:=0.6"])
        self.backend = Backend(database)
        self.node = _module.Relocalizer()
        self.observer = Node("observer", use_global_arguments=False)
        self.client = self.observer.create_client(Trigger, "/hei_robot_mapping/relocalize")
        self.executor = SingleThreadedExecutor()
        for node in (self.backend, self.node, self.observer):
            self.executor.add_node(node)
        self.wait(lambda: self.client.service_is_ready())

    def wait(self, condition, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        assert condition(), "Timed out waiting for test ROS events"

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def request(self):
        generation = self.node.generation
        future = self.client.call_async(Trigger.Request())
        self.wait(lambda: future.done() or
                  (self.node.generation > generation and self.node.armed))
        return future

    def sample(self, accepted=True, covariance=0.01, old=False, tf=True, rejected=False):
        stamp = self.backend.get_clock().now().to_msg()
        if old:
            stamp.sec -= 10
        if tf:
            transforms = []
            for parent, child in (("map", "odom"), ("odom", "body")):
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.header.stamp = stamp
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            self.backend.transforms.sendTransform(transforms)
            self.spin(0.03)
        info = Info()
        info.header.stamp = stamp
        info.proximity_detection_id = 7 if accepted else 0
        if rejected:
            info.stats_keys = ["Loop/RejectedHypothesis/"]
            info.stats_values = [1.0]
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            pose.pose.covariance[index] = covariance
        self.backend.pose.publish(pose)
        self.backend.info.publish(info)
        self.spin(0.06)

    def close(self):
        self.executor.shutdown()
        for node in (self.node, self.backend, self.observer):
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def ros(tmp_path, monkeypatch):
    harness = Harness(tmp_path, monkeypatch)
    try:
        yield harness
    finally:
        harness.close()


def test_service_waits_for_registration_and_expires_readiness(ros):
    result = ros.request()
    ros.wait(lambda: len(ros.backend.seeds) == 1)
    assert not result.done()
    for _ in range(3):
        ros.sample(old=True)
    assert ros.node.good == 0 and not result.done()
    for _ in range(2):
        ros.sample()
    assert not result.done()
    ros.sample()
    ros.wait(result.done)
    assert result.result().success and ros.node.ready
    ros.wait(lambda: not ros.node.ready, timeout=2)


def test_backend_mismatch_is_rejected_without_sending_seed(ros):
    ros.backend.set_parameters([rclpy.parameter.Parameter("Mem/IncrementalMemory", value="true")])
    result = ros.request()
    ros.wait(result.done)
    assert not result.result().success
    assert not ros.backend.seeds


def test_missing_matches_timeout_and_busy_requests_fail(ros):
    first = ros.request()
    second = ros.client.call_async(Trigger.Request())
    ros.wait(second.done)
    assert not second.result().success
    ros.sample(accepted=False)
    ros.wait(first.done)
    assert not first.result().success and not ros.node.ready


def test_invalid_pose_rejected_graph_and_missing_tf_do_not_confirm(ros):
    result = ros.request()
    for _ in range(3):
        ros.sample(tf=False)
    assert not result.done()
    for _ in range(3):
        ros.sample(covariance=math.nan)
    assert not result.done()
    for _ in range(3):
        ros.sample(rejected=True)
    assert not result.done()
    for _ in range(3):
        ros.sample()
    ros.wait(result.done)
    assert result.result().success


def test_failed_match_resets_consecutive_count_and_service_can_retry(ros):
    result = ros.request()
    ros.sample()
    ros.sample(accepted=False)
    for _ in range(2):
        ros.sample()
    assert not result.done()
    ros.sample()
    ros.wait(result.done)
    assert result.result().success
    retry = ros.request()
    assert not ros.node.ready and not retry.done()
    for _ in range(3):
        ros.sample()
    ros.wait(retry.done)
    assert retry.result().success
