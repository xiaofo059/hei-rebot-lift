"""launch 与配置的静态护栏（不需要 ROS 环境即可跑）。

目的：防止后续改动悄悄破坏三条安全属性：
  1. 未标定必须显式 opt-in（uncalibrated:=true），默认不得启动；
  2. 必须离线隔离（ROS_DOMAIN_ID=88 且 ROS_LOCALHOST_ONLY=1）；
  3. 本包不得发布任何 TF / base_link / map 变换。
"""

from __future__ import annotations

from pathlib import Path

import yaml

from hei_robot_localization import lio_contract as C

PKG_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PKG_ROOT / "launch" / "fast_lio_mid360s_offline.launch.py"
CONFIG = PKG_ROOT / "config" / "mid360s_fastlio_uncalibrated.yaml"


def _launch_src() -> str:
    assert LAUNCH.is_file(), f"缺少 launch 文件 {LAUNCH}"
    return LAUNCH.read_text(encoding="utf-8")


def test_launch_requires_explicit_uncalibrated_optin():
    src = _launch_src()
    assert 'uncalibrated' in src
    assert 'default_value="false"' in src, "uncalibrated 默认必须是 false（需显式 opt-in）"
    assert "未标定" in src
    # 拒绝路径必须真实存在：默认值不是 true 时会走 _fail
    assert 'uncal not in ("true", "1", "yes")' in src


def test_launch_enforces_offline_isolation():
    src = _launch_src()
    assert 'OFFLINE_DOMAIN_ID = "88"' in src
    assert 'os.environ.get("ROS_DOMAIN_ID"' in src
    assert 'os.environ.get("ROS_LOCALHOST_ONLY"' in src
    assert "ROS_LOCALHOST_ONLY=1" in src


def test_launch_requires_existing_bag_and_metadata():
    src = _launch_src()
    assert "metadata.yaml" in src, "必须校验 bag 目录结构，避免误开空目录"
    assert "is_dir()" in src


def test_launch_does_not_publish_tf_or_fake_frames():
    src = _launch_src()
    for forbidden in ("TransformBroadcaster", "StaticTransformBroadcaster", "tf2_ros"):
        assert forbidden not in src, f"本包不得创建 TF 发布器（发现 {forbidden}）"
    # 只允许 fast_lio 自己发 camera_init -> body；不得出现 base_link / map 变换
    # （说明性文字允许提到这些 frame，因此检查的是被当作 frame 名使用的字面量）
    for forbidden_frame in ('"base_link"', "'base_link'", '"map"', "'map'", '"odom"', "'odom'"):
        assert forbidden_frame not in src, f"不得发布/引用伪造 frame {forbidden_frame}"


def test_launch_starts_single_fastlio_node():
    src = _launch_src()
    assert src.count('executable="fastlio_mapping"') == 1
    assert '"fast_lio"' in src


def test_launch_has_no_hardware_driver():
    src = _launch_src()
    for forbidden in ("livox_ros_driver2_node", "heitor", "serial", "can0"):
        assert forbidden not in src, f"离线 launch 不得包含硬件入口（{forbidden}）"


def test_config_yaml_frames_and_units_are_consistent():
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    p = data["/**"]["ros__parameters"]
    assert p["common"]["lid_topic"] == "/livox/lidar"
    assert p["common"]["imu_topic"] == "/livox/imu"
    assert p["preprocess"]["lidar_type"] == 1
    assert p["preprocess"]["scan_line"] == 4
    assert p["mapping"]["extrinsic_T"] == [C.FASTLIO_NOMINAL_EXTRINSIC_T[i] for i in range(3)]
    assert p["pcd_save"]["pcd_save_en"] is False, "离线冒烟不应写点云大文件"
    # 参数文件里不得出现任何 g->SI 换算键（FAST-LIO 没有该参数；出现即误导）
    flat = yaml.safe_dump(p)
    assert "acc_scale" not in flat
    assert "gravity_scale" not in flat
