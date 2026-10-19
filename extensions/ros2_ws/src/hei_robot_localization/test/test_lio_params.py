"""参数集与“未标定”门控的回归测试。

固定住本实验的三个不可协商项：
  1. lidar_type 必须是 1（否则 FAST-LIO 不订阅 CustomMsg）；
  2. 外参必须是手册名义值推导出的 t_IL = [-0.011, -0.02329, +0.04412]，且禁止在线估计；
  3. time_sync_en 必须为 false、time_offset_lidar_to_imu 必须为 0（NoSync 不能靠软件猜）。
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest

from hei_robot_localization import lio_contract as C
from hei_robot_localization import mid360s_lio_params as M

PKG_ROOT = Path(__file__).resolve().parents[1]
SHIPPED_YAML = PKG_ROOT / "config" / "mid360s_fastlio_uncalibrated.yaml"


def test_shipped_yaml_exists():
    assert SHIPPED_YAML.is_file(), f"缺少参数文件 {SHIPPED_YAML}"


def test_fastlio_params_are_valid():
    assert M.validate(M.FASTLIO_PARAMS) == []


def test_shipped_yaml_matches_module_and_validates():
    loaded = M.load_and_validate(SHIPPED_YAML)
    assert M.validate(loaded) == []
    assert loaded["preprocess"]["lidar_type"] == 1
    assert loaded["preprocess"]["scan_line"] == 4
    assert loaded["mapping"]["extrinsic_T"] == pytest.approx(
        list(C.FASTLIO_NOMINAL_EXTRINSIC_T), abs=1e-12
    )
    assert loaded["mapping"]["extrinsic_est_en"] is False
    assert loaded["common"]["time_sync_en"] is False
    assert loaded["common"]["time_offset_lidar_to_imu"] == 0.0


def _mutated(**overrides):
    params = copy.deepcopy(M.FASTLIO_PARAMS)
    for dotted, value in overrides.items():
        cur = params
        keys = dotted.split(".")
        for k in keys[:-1]:
            cur = cur[k]
        cur[keys[-1]] = value
    return params


@pytest.mark.parametrize(
    "dotted,value,needle",
    [
        ("preprocess.lidar_type", 4, "CustomMsg"),
        ("preprocess.scan_line", 16, "line"),
        ("common.time_sync_en", True, "NoSync"),
        ("common.time_offset_lidar_to_imu", 0.1, "时间偏置"),
        ("mapping.extrinsic_est_en", True, "在线估计"),
        ("common.lid_topic", "/livox/lidar2", "话题"),
    ],
)
def test_validate_rejects_contract_violations(dotted, value, needle):
    problems = M.validate(_mutated(**{dotted: value}))
    assert problems, f"{dotted} 改成 {value!r} 后必须报错"
    assert any(needle in p for p in problems)


def test_validate_rejects_copied_chip_position_without_sign_flip():
    """把手册芯片坐标直接当 extrinsic_T（不取反）必须被拒绝。"""
    bad = list(C.MID360S_IMU_CHIP_POSITION_IN_LIDAR_M)
    problems = M.validate(_mutated(**{"mapping.extrinsic_T": bad}))
    assert problems
    assert any("符号" in p for p in problems)


def test_validate_rejects_non_identity_rotation():
    bad = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0]
    problems = M.validate(_mutated(**{"mapping.extrinsic_R": bad}))
    assert problems
    assert any("extrinsic_R" in p for p in problems)


def test_validate_reports_missing_key():
    params = copy.deepcopy(M.FASTLIO_PARAMS)
    del params["common"]["time_sync_en"]
    problems = M.validate(params)
    assert any("common.time_sync_en" in p for p in problems)


def test_load_and_validate_rejects_missing_file(tmp_path):
    with pytest.raises(ValueError):
        M.load_and_validate(tmp_path / "nope.yaml")


def test_load_and_validate_rejects_wrong_structure(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("foo: 1\n", encoding="utf-8")
    with pytest.raises(ValueError):
        M.load_and_validate(bad)


def test_load_and_validate_rejects_violating_file(tmp_path):
    text = SHIPPED_YAML.read_text(encoding="utf-8")
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        text.replace("time_sync_en: false", "time_sync_en: true"), encoding="utf-8"
    )
    with pytest.raises(ValueError):
        M.load_and_validate(bad)


def test_metadata_marks_uncalibrated_and_no_conversion():
    md = M.EXPERIMENT_METADATA
    assert md["extrinsic_status"] == "UNCALIBRATED_NOMINAL"
    assert "User Manual" in md["extrinsic_source"]
    assert "§3.5" in md["extrinsic_source"]
    assert "g" in md["imu_acc_unit"]
    assert "无" in md["acc_conversion"]
    notice = M.launch_notice()
    assert "UNCALIBRATED_NOMINAL" in notice
    assert "camera_init" in notice
