"""mid360s_config 的无硬件单元测试：结构、端口、IPv4 校验、现场配置校验。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from hei_robot_bringup import mid360s_config as cfg

MODULE_PATH = Path(cfg.__file__)
LAUNCH_PATH = MODULE_PATH.parents[1] / "launch" / "mid360s_bringup.launch.py"


def test_upstream_imu_facts_are_pinned():
    """上游 IMU 单位/字段事实固定在常量里，防止被顺手改成“看起来更合理”的值。

    依据 driver 1.2.6 冻结源码：
      * src/comm/pub_handler.cpp:119-124 原样拷贝，无换算；
      * src/lddc.cpp:489-494 原样写入，不设 orientation/covariance；
      * src/comm/lidar_imu_data_queue.h:37-42 自述 acc 单位 g、gyro 单位 rad/s。
    """
    assert cfg.IMU_ANGULAR_VELOCITY_UNIT == "rad/s"
    assert cfg.IMU_LINEAR_ACCELERATION_UNIT == "g"
    assert cfg.IMU_LINEAR_ACCELERATION_IS_SI is False
    assert cfg.IMU_ORIENTATION_PUBLISHED is False
    assert cfg.IMU_FRAME_ID_UPSTREAM == "livox_frame"
    assert "livox_eth_protocol_mid360" in cfg.IMU_PROTOCOL_REFERENCE


def test_imu_frame_comment_does_not_claim_topic_mismatch():
    """frame_id 与 topic 名称本就无需同名；注释不得写成“frame 与 topic 不一致”。"""
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "与其发布话题不一致" not in source
    assert "与**配置的点云 frame** 不同" in source


# --------------------------------------------------------------------------
# 结构 / 官方一致性
# --------------------------------------------------------------------------


def test_build_config_uses_mid360s_key_and_array_host_info():
    """S 型必须用 'Mid360s' 键与数组形式 host_net_info（不是 MID360 的对象形式）。"""
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    assert set(data) == {"lidar_summary_info", "Mid360s", "lidar_configs"}
    block = data["Mid360s"]
    assert "MID360" not in data, "不得生成 MID360（非 S）顶层键"
    assert isinstance(block["host_net_info"], list)
    assert block["host_net_info"][0]["host_ip"] == "10.20.30.40"
    assert data["lidar_configs"][0]["ip"] == "10.20.30.50"


def test_ports_match_upstream_mid360s_config():
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    assert data["Mid360s"]["lidar_net_info"] == {
        "cmd_data_port": 56100,
        "push_msg_port": 56200,
        "point_data_port": 56300,
        "imu_data_port": 56400,
        "log_data_port": 56500,
    }
    host = data["Mid360s"]["host_net_info"][0]
    assert {k: v for k, v in host.items() if k != "host_ip"} == {
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501,
    }


def test_summary_type_is_driver_flag_not_sdk_enum():
    """lidar_type 是 driver 内部位标志 8；S 型 SDK 枚举 35 不能写进配置。"""
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    assert data["lidar_summary_info"]["lidar_type"] == 8
    assert cfg.LIDAR_SUMMARY_TYPE == 8


def test_raw_pointcloud_extrinsic_is_identity():
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    assert data["lidar_configs"][0]["extrinsic_parameter"] == {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    }


# --------------------------------------------------------------------------
# IPv4 校验
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value,reason",
    [
        ("", "空"),
        ("   ", "空"),
        ("not-an-ip", "非法"),
        ("999.1.1.1", "非法"),
        ("192.168.1", "非法"),
        ("::1", "IPv6 不是 IPv4"),
        ("224.0.0.1", "组播"),
        ("239.255.255.250", "组播"),
        ("0.0.0.0", "未指定"),
        ("255.255.255.255", "广播"),
        ("127.0.0.1", "环回"),
    ],
)
def test_validate_ipv4_rejects_bad_addresses(value, reason):
    with pytest.raises(cfg.ConfigError):
        cfg.validate_ipv4(value, field="host_ip")


@pytest.mark.parametrize("value", ["10.20.30.40", "192.168.50.1", "172.16.0.9", "1.2.3.4"])
def test_validate_ipv4_accepts_unicast(value):
    assert cfg.validate_ipv4(value, field="host_ip") == value


def test_validate_ipv4_strips_whitespace():
    assert cfg.validate_ipv4("  10.20.30.40  ", field="host_ip") == "10.20.30.40"


def test_validate_pair_rejects_identical_addresses():
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.validate_pair("10.20.30.40", "10.20.30.40")
    assert "不能相同" in str(excinfo.value)


def test_validate_pair_rejects_multicast_even_if_other_side_ok():
    with pytest.raises(cfg.ConfigError):
        cfg.validate_pair("10.20.30.40", "224.0.0.5")
    with pytest.raises(cfg.ConfigError):
        cfg.validate_pair("0.0.0.0", "10.20.30.50")


# --------------------------------------------------------------------------
# 没有示例 IP 默认值
# --------------------------------------------------------------------------


def test_module_and_launch_contain_no_example_ip_literals():
    """生成器不得内置示例 IP：任何 '192.168.' 之类的字面量都是隐患。"""
    for path in (MODULE_PATH, LAUNCH_PATH):
        source = path.read_text(encoding="utf-8")
        for needle in ("192.168.", "10.0.0.", "172.16."):
            assert needle not in source, f"{path.name} 含示例 IP 字面量 {needle}"


def test_build_config_requires_explicit_addresses():
    """不给地址就无法构造配置（没有默认值兜底）。"""
    with pytest.raises(cfg.ConfigError):
        cfg.build_config("", "")
    with pytest.raises(cfg.ConfigError):
        cfg.build_config(None, None)  # type: ignore[arg-type]


# --------------------------------------------------------------------------
# 现场配置读取 / 校验
# --------------------------------------------------------------------------


def _write(path: Path, data) -> Path:
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return path


def test_roundtrip_dumps_then_load(tmp_path):
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    path = _write(tmp_path / "cfg.json", data)
    field = cfg.load_and_validate(path)
    assert field.host_ip == "10.20.30.40"
    assert field.lidar_ip == "10.20.30.50"
    assert field.lidar_type == 8


def test_dumps_is_stable_json(tmp_path):
    text = cfg.dumps_config(cfg.build_config("10.20.30.40", "10.20.30.50"))
    assert text.endswith("\n")
    assert json.loads(text)["Mid360s"]["host_net_info"][0]["host_ip"] == "10.20.30.40"


def test_load_missing_file_points_to_generator(tmp_path):
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_and_validate(tmp_path / "nope.json")
    assert "mid360s_make_config" in str(excinfo.value)


def test_load_rejects_bad_json(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_and_validate(path)
    assert "JSON" in str(excinfo.value)


def test_load_rejects_non_object(tmp_path):
    path = _write(tmp_path / "list.json", [1, 2, 3])
    with pytest.raises(cfg.ConfigError):
        cfg.load_and_validate(path)


def test_load_rejects_mid360_non_s_config(tmp_path):
    """MID360（非 S）配置的顶层键是 'MID360'，必须被明确拒绝而不是静默使用。"""
    path = _write(tmp_path / "mid360.json", {"MID360": {}, "lidar_configs": []})
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_and_validate(path)
    assert "Mid360s" in str(excinfo.value)


def test_load_rejects_object_host_net_info(tmp_path):
    """S 型要求数组 host_net_info；对象形式（MID360）会静默收不到点云，必须拦。"""
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    data["Mid360s"]["host_net_info"] = {"cmd_data_ip": "10.20.30.40"}
    path = _write(tmp_path / "obj.json", data)
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_and_validate(path)
    assert "数组" in str(excinfo.value)


def test_load_rejects_sdk_enum_lidar_type(tmp_path):
    """写成 SDK 的 S 型枚举 35 会破坏 driver 位标志语义。"""
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    data["lidar_summary_info"]["lidar_type"] = 35
    path = _write(tmp_path / "enum.json", data)
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_and_validate(path)
    assert "35" in str(excinfo.value)


def test_load_rejects_missing_host_ip(tmp_path):
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    del data["Mid360s"]["host_net_info"][0]["host_ip"]
    path = _write(tmp_path / "nohost.json", data)
    with pytest.raises(cfg.ConfigError):
        cfg.load_and_validate(path)


def test_load_rejects_missing_lidar_ip(tmp_path):
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    data["lidar_configs"][0].pop("ip")
    path = _write(tmp_path / "nolidar.json", data)
    with pytest.raises(cfg.ConfigError):
        cfg.load_and_validate(path)


def test_load_revalidates_hand_edited_addresses(tmp_path):
    """现场手改过的配置同样要过 IPv4 校验。"""
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    data["Mid360s"]["host_net_info"][0]["host_ip"] = "0.0.0.0"
    path = _write(tmp_path / "edited.json", data)
    with pytest.raises(cfg.ConfigError):
        cfg.load_and_validate(path)


def test_load_rejects_identical_host_and_lidar(tmp_path):
    data = cfg.build_config("10.20.30.40", "10.20.30.50")
    data["lidar_configs"][0]["ip"] = "10.20.30.40"
    path = _write(tmp_path / "same.json", data)
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_and_validate(path)
    assert "不能相同" in str(excinfo.value)
