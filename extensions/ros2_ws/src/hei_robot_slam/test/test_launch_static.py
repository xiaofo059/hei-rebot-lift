"""launch/config 静态护栏：没有 ROS 环境也能跑。"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml

from hei_robot_slam import slam_contract as C

PKG_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PKG_ROOT / "launch"
CONFIG = PKG_ROOT / "config"

LAUNCH_FILES = sorted(LAUNCH.glob("*.launch.py"))
CONFIG_FILES = sorted(CONFIG.glob("*.yaml"))


def _src(path: Path) -> str:
    assert path.is_file(), f"缺少文件 {path}"
    return path.read_text(encoding="utf-8")


def _calls(src: str) -> list[tuple[str, dict]]:
    result: list[tuple[str, dict]] = []
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Name):
            name = func.id
        elif isinstance(func, ast.Attribute):
            name = func.attr
        else:
            name = ""
        result.append((name, {keyword.arg: keyword.value for keyword in node.keywords if keyword.arg}))
    return result


def _const(value) -> object:
    return value.value if isinstance(value, ast.Constant) else None


def _node_packages_and_executables(src: str) -> list[tuple[str, str]]:
    pairs: list[tuple[str, str]] = []
    for name, kwargs in _calls(src):
        if name != "Node":
            continue
        pairs.append((_const(kwargs.get("package")), _const(kwargs.get("executable"))))
    return pairs


def _has_database_delete_flag(src: str) -> list[str]:
    """只有把 -d 传给 rtabmap 节点/进程才算自动删除数据库；rviz 的 -d 配置参数不算。"""
    findings: list[str] = []
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.id if isinstance(func, ast.Name) else (func.attr if isinstance(func, ast.Attribute) else "")
        kwargs = {keyword.arg: keyword.value for keyword in node.keywords if keyword.arg}
        targets_rtabmap = "rtabmap" in str(_const(kwargs.get("package")) or "")
        if name == "ExecuteProcess":
            targets_rtabmap = True
        if not targets_rtabmap:
            continue
        for child in ast.walk(node):
            if isinstance(child, ast.Constant) and child.value in ("-d", "--delete_db_on_start", "delete_db_on_start"):
                findings.append(f"line {child.lineno}: {child.value}")
    return findings


# ---- 所有 launch 文件共同护栏 ---------------------------------------------
def test_launch_files_exist_and_compile():
    assert [path.name for path in LAUNCH_FILES] == [
        "slam_live_localization.launch.py",
        "slam_live_mapping.launch.py",
        "slam_localization.launch.py",
        "slam_offline_mapping.launch.py",
    ]
    for path in LAUNCH_FILES:
        ast.parse(_src(path))


def test_no_launch_file_passes_database_delete_flag():
    for path in LAUNCH_FILES:
        assert _has_database_delete_flag(_src(path)) == [], f"{path.name} 把 -d 传给了 RTAB-Map"


def test_no_config_file_enables_auto_delete():
    for path in CONFIG_FILES:
        data = yaml.safe_load(_src(path))
        params = data.get("/**", {}).get("ros__parameters", {})
        assert "delete_db_on_start" not in params, f"{path.name} 不得包含 delete_db_on_start"
        assert "database_path" not in params, f"{path.name} 不得写死 database_path（由 launch 决定）"


def test_package_never_publishes_tf():
    for path in sorted((PKG_ROOT / "hei_robot_slam").glob("*.py")):
        src = _src(path)
        for token in ("TransformBroadcaster", "StaticTransformBroadcaster"):
            assert token not in src, f"{path.name} 不得创建 TF 发布器（{token}）"


def test_bag_play_uses_clock_for_offline_sim_time():
    support = _src(PKG_ROOT / "hei_robot_slam" / "launch_support.py")
    assert "--clock" in support
    assert "SIM_CLOCK_HZ" in support


# ---- 离线建图 launch -------------------------------------------------------
def test_offline_mapping_launch_has_all_safety_gates():
    src = _src(LAUNCH / "slam_offline_mapping.launch.py")
    for token in (
        "require_uncalibrated_optin",
        "require_offline_isolation",
        "resolve_bag_and_run_dir",
        "resolve_param_file",
        "C.MODE_MAPPING",
    ):
        assert token in src, f"离线建图 launch 缺少护栏 {token}"


def test_offline_mapping_launch_uses_expected_nodes_and_topics():
    src = _src(LAUNCH / "slam_offline_mapping.launch.py")
    nodes = _node_packages_and_executables(src)
    assert nodes.count(("fast_lio", "fastlio_mapping")) == 1, "只允许一个 FAST-LIO 实例"
    assert ("hei_robot_slam", "slam_cloud_adapter") in nodes
    assert ("rtabmap_slam", "rtabmap") in nodes
    packages = [package for package, _ in nodes]
    for forbidden in ("rtabmap_odom", "rtabmap_util", "livox_ros_driver2"):
        assert forbidden not in packages, f"离线建图 launch 不得启动 {forbidden}"
    assert "scan_cloud" in src
    assert "LIO_BODY_CLOUD_TOPIC" in src
    assert "SLAM_CLOUD_TOPIC" in src


def test_offline_mapping_launch_enables_sim_time_for_frontend_and_backend():
    src = _src(LAUNCH / "slam_offline_mapping.launch.py")
    assert src.count("use_sim_time") >= 4
    assert "bag_clock_hz" in src, "离线 launch_info 必须记录 bag clock 与 sim time"


# ---- 定位 launch -----------------------------------------------------------
def test_localization_launch_is_read_only_and_requires_existing_db():
    src = _src(LAUNCH / "slam_localization.launch.py")
    for token in (
        "resolve_localization_inputs",
        "C.MODE_LOCALIZATION",
        "database_expected_unmodified",
        "initialpose",
    ):
        assert token in src, f"定位 launch 缺少 {token}"


def test_live_localization_uses_live_sensor_and_read_only_database():
    src = _src(LAUNCH / "slam_live_localization.launch.py")
    for token in (
        "allow_live_hardware",
        "require_uncalibrated_optin",
        "resolve_live_localization_inputs",
        "C.MODE_LOCALIZATION",
        "resolve_param_file",
        "database_path",
        "database_expected_unmodified",
        "database_sha256_before",
        "pointcloud_format",
        "custom",
        "use_sim_time",
    ):
        assert token in src, f"实时定位 launch 缺少护栏/配置 {token}"
    assert "bag_path" not in src, "实时定位不应依赖 rosbag 输入"
    assert "load_lio_params" in src
    for forbidden in ("hei_rebot_lift", "chassis", "lift_io", "lerobot"):
        assert forbidden not in src, f"实时定位不得启动机器人本体（{forbidden}）"


# ---- live launch -----------------------------------------------------------
def test_live_launch_requires_explicit_hardware_and_uncalibrated_optin():
    src = _src(LAUNCH / "slam_live_mapping.launch.py")
    for token in (
        "allow_live_hardware",
        "require_uncalibrated_optin",
        "mid360s_bringup",
        "no_robot_base_arm_lift",
        "pointcloud_format",
        "custom",
        "load_lio_params",
    ):
        assert token in src, f"live launch 缺少 {token}"
    assert "executed_in_this_round" not in src, "live launch_info 不应再写运行元数据"


def test_live_launch_only_starts_lidar_related_nodes():
    src = _src(LAUNCH / "slam_live_mapping.launch.py")
    allowed = {("fast_lio", "fastlio_mapping"), ("hei_robot_slam", "slam_cloud_adapter"), ("hei_robot_slam", "slam_cloud_probe"), ("rtabmap_slam", "rtabmap"), ("rviz2", "rviz2")}
    for pair in _node_packages_and_executables(src):
        assert pair in allowed, f"live launch 启动了未授权的节点 {pair}"
    for forbidden in ("hei_rebot_lift", "chassis", "lift_io", "lerobot"):
        assert forbidden not in src, f"live launch 不得涉及机器人本体（{forbidden}）"


# ---- 配置与 RViz ----------------------------------------------------------
def test_configs_validate_against_contract():
    C.validate_mapping_parameters(C.load_param_yaml(CONFIG / "rtabmap_3d_lidar_mapping.yaml"))
    C.validate_localization_parameters(C.load_param_yaml(CONFIG / "rtabmap_3d_lidar_localization.yaml"))


def test_rviz_configs_show_map_and_hide_mapping_history_during_localization():
    for name in ("slam_mapping.rviz", "slam_localization.rviz"):
        data = yaml.safe_load(_src(CONFIG / name))
        manager = data["Visualization Manager"]
        assert manager["Global Options"]["Fixed Frame"] == C.MAP_FRAME
        topics = {
            display.get("Topic", {}).get("Value"): display
            for display in manager["Displays"]
            if isinstance(display.get("Topic"), dict)
        }
        assert "/cloud_map" in topics, f"{name} 必须显示后端优化地图 /cloud_map"
        classes = [display.get("Class") for display in manager["Displays"]]
        assert "rviz_default_plugins/PointCloud2" in classes
        assert "/mapPath" in topics, f"{name} 保留可选历史轨迹 display"
        expected_path_enabled = name == "slam_mapping.rviz"
        assert topics["/mapPath"]["Enabled"] is expected_path_enabled
