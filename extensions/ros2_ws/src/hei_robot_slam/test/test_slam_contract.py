"""契约回归：frame/模式/DB 路径保护/离线隔离/参数校验（纯 Python，无 ROS 依赖）。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from hei_robot_slam import slam_contract as C

PKG_ROOT = Path(__file__).resolve().parents[1]
CONFIG_MAPPING = PKG_ROOT / "config" / "rtabmap_3d_lidar_mapping.yaml"
CONFIG_LOCALIZATION = PKG_ROOT / "config" / "rtabmap_3d_lidar_localization.yaml"


def _mapping_params() -> dict:
    return C.load_param_yaml(CONFIG_MAPPING)


def _localization_params() -> dict:
    return C.load_param_yaml(CONFIG_LOCALIZATION)


# ---- frame / topic 契约 ----------------------------------------------------
def test_frame_contract_is_exact():
    assert C.MAP_FRAME == "map"
    assert C.ODOM_FRAME == "camera_init"
    assert C.BASE_FRAME == "body"
    assert C.FORBIDDEN_FRAMES == ("base_link", "odom", "base_footprint")
    tf = C.tf_contract()
    assert tf["frontend_tf"] == ["camera_init", "body"]
    assert tf["backend_tf"] == ["map", "camera_init"]
    assert tf["no_base_link_no_odom"] is True


def test_cloud_contract_keeps_stamp_frame_and_drops_unestimated_normals():
    contract = C.cloud_contract()
    assert contract["preserve"] == ["header.stamp", "header.frame_id", "point xyz", "intensity"]
    assert "normal_x" in contract["dropped_unestimated_fields"]
    assert contract["required_fields"] == ["x", "y", "z", "intensity"]
    assert contract["no_deskew"] is True
    assert contract["no_accumulated_map"] is True


# ---- 输出目录/DB 保护 ------------------------------------------------------
def test_resolve_run_dir_is_unique_and_never_reuses(tmp_path: Path):
    run_a = C.resolve_run_dir(tmp_path, "session_a")
    assert run_a.is_dir()
    with pytest.raises(C.SlamContractError):
        C.resolve_run_dir(tmp_path, "session_a")


def test_resolve_run_dir_rejects_bad_session(tmp_path: Path):
    for bad in ("a/b", "..", "."):
        with pytest.raises(C.SlamContractError):
            C.resolve_run_dir(tmp_path, bad)


def test_mapping_database_must_not_exist(tmp_path: Path):
    db = tmp_path / "rtabmap_mapping.db"
    db.write_bytes(b"old database")
    with pytest.raises(C.SlamContractError):
        C.validate_mapping_database_target(db)
    fresh = tmp_path / "new.db"
    assert C.validate_mapping_database_target(fresh) == fresh.resolve()


def test_localization_database_must_exist_non_empty(tmp_path: Path):
    missing = tmp_path / "missing.db"
    with pytest.raises(C.SlamContractError):
        C.validate_localization_database(missing)
    empty = tmp_path / "empty.db"
    empty.write_bytes(b"")
    with pytest.raises(C.SlamContractError):
        C.validate_localization_database(empty)
    valid = tmp_path / "valid.db"
    valid.write_bytes(b"database")
    assert C.validate_localization_database(valid) == valid.resolve()


def test_sha256_file():
    target = PKG_ROOT / "package.xml"
    digest = C.sha256_file(target)
    assert len(digest) == 64 and int(digest, 16) >= 0


# ---- 离线隔离 --------------------------------------------------------------
def test_offline_isolation_requires_domain_89_and_localhost():
    C.check_offline_isolation({"ROS_DOMAIN_ID": "89", "ROS_LOCALHOST_ONLY": "1"})
    for env in (
        {"ROS_DOMAIN_ID": "88", "ROS_LOCALHOST_ONLY": "1"},
        {"ROS_DOMAIN_ID": "89", "ROS_LOCALHOST_ONLY": "0"},
        {},
    ):
        with pytest.raises(C.SlamContractError):
            C.check_offline_isolation(env)


# ---- 参数契约 --------------------------------------------------------------
def test_mapping_and_localization_configs_follow_contract():
    C.validate_mapping_parameters(_mapping_params())
    C.validate_localization_parameters(_localization_params())


def test_config_overrides_are_exactly_the_three_mode_switches():
    mapping = _mapping_params()
    localization = _localization_params()
    diff = {key for key in set(mapping) & set(localization) if mapping[key] != localization[key]}
    assert diff == {"Mem/IncrementalMemory", "Mem/InitWMWithAllNodes", "Mem/LocalizationReadOnly"}


def test_mapping_rejects_localization_switch():
    params = dict(_mapping_params())
    params["Mem/IncrementalMemory"] = "false"
    with pytest.raises(C.SlamContractError):
        C.validate_mapping_parameters(params)


def test_localization_rejects_writable_database():
    params = dict(_localization_params())
    params["Mem/LocalizationReadOnly"] = "false"
    with pytest.raises(C.SlamContractError):
        C.validate_localization_parameters(params)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("frame_id", "base_link"),
        ("odom_frame_id", "odom"),
        ("map_frame_id", "world"),
        ("subscribe_odom", True),
        ("subscribe_scan", True),
        ("subscribe_scan_cloud", False),
        ("Reg/Force3DoF", "true"),
        ("Icp/Force4DoF", "true"),
        ("Reg/Strategy", "0"),
        ("Icp/PointToPlane", "false"),
        ("RGBD/ProximityMaxGraphDepth", "50"),
        ("RGBD/ProximityAngle", "45"),
        ("RGBD/ProximityOdomGuess", "false"),
        ("delete_db_on_start", True),
    ],
)
def test_mapping_rejects_contract_violations(key: str, value) -> None:
    params = dict(_mapping_params())
    params[key] = value
    with pytest.raises(C.SlamContractError):
        C.validate_mapping_parameters(params)


def test_rejects_auto_delete_in_args():
    params = _mapping_params()
    params["args"] = "--delete_db_on_start"
    with pytest.raises(C.SlamContractError):
        C.validate_mapping_parameters(params)
    params2 = _mapping_params()
    params2["args"] = "-d"
    with pytest.raises(C.SlamContractError):
        C.validate_mapping_parameters(params2)


def test_missing_required_parameter_is_rejected():
    params = _mapping_params()
    del params["Icp/Strategy"]
    with pytest.raises(C.SlamContractError):
        C.validate_mapping_parameters(params)


def test_parameter_yaml_requires_wildcard_section(tmp_path: Path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("foo: 1\n", encoding="utf-8")
    with pytest.raises(C.SlamContractError):
        C.load_param_yaml(bad)


def test_versions_are_pinned():
    versions = C.versions()
    assert versions["rtabmap_version"] == "0.23.13"
    assert versions["rtabmap_ros_deb_pin"].startswith("0.23.13-1jammy.")