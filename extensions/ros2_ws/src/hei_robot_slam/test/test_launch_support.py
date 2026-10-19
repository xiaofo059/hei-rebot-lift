"""launch_support 回归：失败横幅 / 参数解析 / 目录创建 / bag 回放命令。"""

from __future__ import annotations

import pytest

from hei_robot_slam import launch_support as S
from hei_robot_slam import slam_contract as C


def test_require_uncalibrated_optin():
    S.require_uncalibrated_optin({"uncalibrated": "true"})
    for value in ("", "false", "0"):
        with pytest.raises(RuntimeError):
            S.require_uncalibrated_optin({"uncalibrated": value})


def test_parse_replay_arguments():
    rate, offset = S.parse_replay_arguments({"rate": "2.0", "start_offset": "1.5"})
    assert rate == 2.0 and offset == 1.5
    for bad in ({"rate": "-1"}, {"rate": "nan"}, {"start_offset": "-1"}, {"rate": "abc"}):
        with pytest.raises(RuntimeError):
            S.parse_replay_arguments(bad)


def test_resolve_bag_and_run_dir(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    cfg = {"bag_path": str(bag), "output_root": str(tmp_path), "session": "run1"}
    bag_dir, run_dir, db_path = S.resolve_bag_and_run_dir(cfg)
    assert bag_dir == bag.resolve()
    assert run_dir == (tmp_path / "run1").resolve()
    assert db_path.name == C.MAPPING_DB_NAME
    with pytest.raises(RuntimeError):
        S.resolve_bag_and_run_dir(cfg)


def test_resolve_bag_requires_metadata(tmp_path):
    bag = tmp_path / "empty"
    bag.mkdir()
    with pytest.raises(RuntimeError):
        S.resolve_bag_and_run_dir({"bag_path": str(bag), "output_root": str(tmp_path)})


def test_resolve_localization_requires_existing_db(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("x: 1\n", encoding="utf-8")
    cfg = {"bag_path": str(bag), "output_root": str(tmp_path), "database_path": str(tmp_path / "missing.db")}
    with pytest.raises(RuntimeError):
        S.resolve_localization_inputs(cfg)
    database = tmp_path / "map.db"
    database.write_bytes(b"db")
    cfg["database_path"] = str(database)
    bag_dir, db_path, run_dir = S.resolve_localization_inputs(cfg)
    assert db_path == database.resolve()
    assert run_dir.is_dir()


def test_resolve_live_localization_requires_existing_db_and_unique_log_dir(tmp_path):
    database = tmp_path / "map.db"
    with pytest.raises(RuntimeError):
        S.resolve_live_localization_inputs({"output_root": str(tmp_path)})
    with pytest.raises(RuntimeError):
        S.resolve_live_localization_inputs(
            {"database_path": str(tmp_path / "missing.db"), "output_root": str(tmp_path)}
        )

    database.write_bytes(b"readonly-map")
    cfg = {"database_path": str(database), "output_root": str(tmp_path), "session": "live-localization"}
    db_path, run_dir = S.resolve_live_localization_inputs(cfg)
    assert db_path == database.resolve()
    assert run_dir == (tmp_path / "live-localization").resolve()
    assert run_dir.is_dir()
    with pytest.raises(RuntimeError):
        S.resolve_live_localization_inputs(cfg)


def test_resolve_param_file_rejects_missing(tmp_path):
    with pytest.raises(RuntimeError):
        S.resolve_param_file({"rtabmap_params_file": str(tmp_path / "missing.yaml")}, str(tmp_path), C.MODE_MAPPING)


def test_write_launch_info(tmp_path):
    target = S.write_launch_info(tmp_path, {"kind": "test", "mode": C.MODE_MAPPING})
    assert target.is_file()
    import json

    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload["kind"] == "test"


def test_bag_play_command_has_clock_and_delay():
    command = S.build_bag_play_command("/tmp/bag", 1.5, 2.0)
    assert "--clock" in command
    assert command[command.index("--clock") + 1] == C.SIM_CLOCK_HZ
    assert "--rate" in command and "1.5" in command
    assert "--start-offset" in command and "2" in command
    assert "--delay" in command
    assert "--disable-keyboard-controls" in command
