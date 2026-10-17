"""launch 文件的静态与功能检查（不启动任何真实设备）。

覆盖任务要求：
  * ``config_path`` 必填，缺失时明确失败并指出缺少哪个现场参数；
  * 只有**一个** driver 实例；
  * 不发布任何未经测量的 base/imu 外参（无 static_transform_publisher）。
"""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from hei_robot_bringup import mid360s_config as cfg

PKG_ROOT = Path(cfg.__file__).parents[1]
LAUNCH_PATH = PKG_ROOT / "launch" / "mid360s_bringup.launch.py"
LAUNCH_SOURCE = LAUNCH_PATH.read_text(encoding="utf-8")


@pytest.fixture(scope="module", autouse=True)
def _writable_ros_log_dir(tmp_path_factory):
    """launch 的 LogInfo 构造时会初始化 launch 日志目录。

    默认位置是 ~/.ros/log；在只读沙箱/受限环境里会直接抛 OSError。
    把它重定向到临时目录，让测试考察 launch 逻辑本身而不是宿主写权限。
    """
    log_dir = tmp_path_factory.mktemp("roslog")
    previous = os.environ.get("ROS_LOG_DIR")
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    yield log_dir
    if previous is None:
        os.environ.pop("ROS_LOG_DIR", None)
    else:
        os.environ["ROS_LOG_DIR"] = previous


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("mid360s_launch_under_test", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def launch_module():
    return _load_launch_module()


def _context(config_path: str = "", **overrides) -> SimpleNamespace:
    configs = {
        "config_path": config_path,
        "publish_freq": "10.0",
        "pointcloud_format": "pointcloud2",
        "frame_id": "lidar_link",
        "launch_rviz": "false",
        "rviz_config": "/tmp/does-not-matter.rviz",
    }
    configs.update(overrides)
    return SimpleNamespace(launch_configurations=configs)


# --------------------------------------------------------------------------
# 静态检查
# --------------------------------------------------------------------------


def test_no_tf_publisher_for_unmeasured_extrinsics():
    """不得用 static_transform_publisher 伪造 base_link/IMU 外参。"""
    assert "static_transform_publisher" not in LAUNCH_SOURCE
    assert "StaticTransformBroadcaster" not in LAUNCH_SOURCE
    assert "TransformBroadcaster" not in LAUNCH_SOURCE


def test_single_driver_instance():
    assert LAUNCH_SOURCE.count('executable="livox_ros_driver2_node"') == 1
    assert LAUNCH_SOURCE.count('package="livox_ros_driver2"') == 1


def test_config_path_argument_has_no_default_value():
    """config_path 必须是必填项：现场 IP 不进仓库，也不写死默认路径。"""
    marker = 'DeclareLaunchArgument(\n                "config_path"'
    start = LAUNCH_SOURCE.index(marker)
    end = LAUNCH_SOURCE.index("DeclareLaunchArgument(", start + len(marker))
    block = LAUNCH_SOURCE[start:end]
    assert "config_path" in block
    assert "default_value" not in block, f"config_path 不得有默认值：{block!r}"


def test_rviz_is_optional_and_defaults_off():
    assert 'default_value="false"' in LAUNCH_SOURCE
    assert "launch_rviz" in LAUNCH_SOURCE


def test_default_frame_is_lidar_link():
    assert cfg.DEFAULT_FRAME_ID == "lidar_link"
    assert 'DEFAULT_FRAME_ID' in LAUNCH_SOURCE


def test_documents_upstream_imu_frame_hardcode():
    """上游 IMU 帧硬编码必须在 launch 内被显式记录，避免下游误判。"""
    assert cfg.IMU_FRAME_ID_UPSTREAM == "livox_frame"
    assert "lddc.cpp:481" in LAUNCH_SOURCE


def test_generate_launch_description_without_context(launch_module):
    """`ros2 launch ... --show-args` 在没有 launch context 时调用本函数。

    回归保护：曾因 FindPackageShare(...).perform(None) 在此路径抛
    AttributeError: 'NoneType' object has no attribute 'perform_substitution'。
    """
    description = launch_module.generate_launch_description()
    assert description is not None
    names = [getattr(entity, "name", None) for entity in description.entities]
    assert "config_path" in names
    assert "pointcloud_format" in names
    assert "launch_rviz" in names
    assert "rviz_config" in names


# --------------------------------------------------------------------------
# 功能检查：缺失/非法现场参数
# --------------------------------------------------------------------------


def test_missing_config_path_fails_with_actionable_message(launch_module):
    with pytest.raises(RuntimeError) as excinfo:
        launch_module._launch_setup(_context(""))
    message = str(excinfo.value)
    assert "config_path" in message
    assert "mid360s_make_config" in message, "必须告诉操作者如何生成现场配置"


def test_nonexistent_config_path_fails(launch_module, tmp_path):
    with pytest.raises(RuntimeError) as excinfo:
        launch_module._launch_setup(_context(str(tmp_path / "absent.json")))
    assert "不存在" in str(excinfo.value)


def test_non_s_config_rejected_at_launch(launch_module, tmp_path):
    path = tmp_path / "mid360.json"
    path.write_text(json.dumps({"MID360": {}, "lidar_configs": []}), encoding="utf-8")
    with pytest.raises(RuntimeError) as excinfo:
        launch_module._launch_setup(_context(str(path)))
    assert "Mid360s" in str(excinfo.value)


def test_invalid_pointcloud_format_rejected(launch_module, tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps(cfg.build_config("10.20.30.40", "10.20.30.50")), encoding="utf-8")
    with pytest.raises(RuntimeError) as excinfo:
        launch_module._launch_setup(_context(str(path), pointcloud_format="pcl"))
    assert "pointcloud_format" in str(excinfo.value)


@pytest.mark.parametrize("freq", ["0", "200", "abc"])
def test_invalid_publish_freq_rejected(launch_module, tmp_path, freq):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps(cfg.build_config("10.20.30.40", "10.20.30.50")), encoding="utf-8")
    with pytest.raises(RuntimeError):
        launch_module._launch_setup(_context(str(path), publish_freq=freq))


# --------------------------------------------------------------------------
# 功能检查：正常路径
# --------------------------------------------------------------------------


def _driver_nodes(actions):
    from launch_ros.actions import Node

    found = []
    for action in actions:
        if isinstance(action, Node):
            package = getattr(action, "node_package", None) or action.__dict__.get("_Node__package")
            found.append(package)
    return found


@pytest.mark.parametrize(
    "fmt,expected_xfer",
    [("pointcloud2", 0), ("custom", 1)],
)
def test_valid_config_builds_exactly_one_driver(launch_module, tmp_path, fmt, expected_xfer):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps(cfg.build_config("10.20.30.40", "10.20.30.50")), encoding="utf-8")

    actions = launch_module._launch_setup(_context(str(path), pointcloud_format=fmt))
    packages = _driver_nodes(actions)
    assert packages.count("livox_ros_driver2") == 1, f"driver 实例数应为 1，实际 {packages}"
    assert packages.count("rviz2") == 1
    assert expected_xfer in (0, 1)
    # 点云格式只影响 xfer_format；两种取值都不得额外引入节点
    assert len([p for p in packages if p not in ("livox_ros_driver2", "rviz2")]) == 0
