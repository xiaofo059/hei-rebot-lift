"""db_tools 回归：Link 枚举、Data.scan 云统计、导出产物校验（无 ROS 依赖）。"""

from __future__ import annotations

import sqlite3
from pathlib import Path
from types import SimpleNamespace

import pytest

from hei_robot_slam import db_tools


def _build_database(path: Path) -> None:
    connection = sqlite3.connect(path)
    connection.execute("CREATE TABLE Node(id INTEGER PRIMARY KEY, map_id INTEGER, stamp REAL)")
    connection.execute("CREATE TABLE Data(id INTEGER PRIMARY KEY, scan BLOB)")
    connection.execute("CREATE TABLE Link(from_id INTEGER, to_id INTEGER, type INTEGER)")
    for index in range(1, 7):
        connection.execute("INSERT INTO Node VALUES(?,?,?)", (index, 0, float(index)))
    for index in range(1, 6):
        connection.execute("INSERT INTO Data VALUES(?,?)", (index, b"cloud-bytes"))
    connection.execute("INSERT INTO Data VALUES(6, NULL)")
    for index in range(1, 5):
        connection.execute("INSERT INTO Link VALUES(?,?,?)", (index, index + 1, 0))
    connection.execute("INSERT INTO Link VALUES(1,6,1)")
    connection.execute("INSERT INTO Link VALUES(2,5,2)")
    connection.execute("INSERT INTO Link VALUES(3,4,3)")
    connection.execute("INSERT INTO Link VALUES(4,6,4)")
    connection.execute("INSERT INTO Link VALUES(5,5,9)")
    connection.commit()
    connection.close()


def test_link_type_enum_matches_installed_header():
    assert db_tools.LINK_TYPES == {
        0: "neighbor",
        1: "global_loop_closure",
        2: "local_space_closure",
        3: "local_time_closure",
        4: "user_closure",
        5: "virtual_closure",
        6: "neighbor_merged",
        7: "pose_prior",
        8: "landmark",
        9: "gravity",
    }
    assert db_tools.PRIMARY_AUTO_LOOP_CLOSURE_TYPE == 2
    assert 4 not in db_tools.AUTO_LOOP_CLOSURE_TYPES


def test_inspect_database_counts_nodes_data_and_links(tmp_path: Path):
    database = tmp_path / "map.db"
    _build_database(database)
    result = db_tools.inspect_database(database, run_rtabmap_info=False)
    assert result["nodes"] == 6
    assert result["data_rows"] == 6
    assert result["nodes_with_scan_data"] == 5
    assert result["link_type_counts"] == {"0": 4, "1": 1, "2": 1, "3": 1, "4": 1, "9": 1}
    assert result["primary_auto_loop_closure_links"] == 1
    assert result["auto_loop_closure_links"] == {
        "global_loop_closure": 1,
        "local_space_closure": 1,
        "local_time_closure": 1,
    }
    assert result["user_manual_closure_links"] == 1
    verdict = result["verdict_inputs"]
    assert verdict["has_stored_cloud"] is True
    assert verdict["stored_cloud_source"] == "Data.scan"
    assert verdict["honesty_note"].find("user_manual") < 0


def test_read_ply_vertex_count(tmp_path: Path):
    ply = tmp_path / "map.ply"
    ply.write_text(
        "ply\nformat ascii 1.0\nelement vertex 42\nproperty float x\nend_header\n0 0 0\n",
        encoding="utf-8",
    )
    assert db_tools.read_ply_vertex_count(ply) == 42


def test_count_pose_lines_ignores_comments(tmp_path: Path):
    poses = tmp_path / "map_poses.txt"
    poses.write_text(
        "# stamp x y z qx qy qz qw id\n1.0 0 0 0 0 0 0 1 1\n2.0 0 0 0 0 0 0 1 2\n",
        encoding="utf-8",
    )
    assert db_tools.count_pose_lines(poses) == 2


PLY_HEADER = (
    "ply\nformat ascii 1.0\n"
    "element vertex 4\n"
    "property float x\nproperty float y\nproperty float z\n"
    "end_header\n"
)


def _fake_export(command, **kwargs):
    output_dir = Path(command[command.index("--output_dir") + 1])
    (output_dir / "map.ply").write_text(
        PLY_HEADER + "0 0 0\n1 1 1\n2 2 2\n3 3 3\n",
        encoding="utf-8",
    )
    (output_dir / "map_poses.txt").write_text(
        "1.0 0 0 0 0 0 0 1 1\n2.0 0 0 0 0 0 0 1 2\n3.0 0 0 0 0 0 0 1 3\n",
        encoding="utf-8",
    )
    return SimpleNamespace(returncode=0, stdout="exported", stderr="")


def test_export_database_validates_non_empty_outputs(tmp_path, monkeypatch):
    database = tmp_path / "map.db"
    database.write_bytes(b"database-bytes")
    monkeypatch.setattr(db_tools.shutil, "which", lambda name: "/fake/rtabmap-export")
    monkeypatch.setattr(db_tools.subprocess, "run", _fake_export)
    result = db_tools.export_database(database, tmp_path / "export")
    assert result["passed"] is True
    assert result["checks"]["cloud_vertices"] == 4
    assert result["checks"]["cloud_geometry"]["valid"] is True
    assert result["checks"]["cloud_geometry"]["nonfinite_xyz"] == 0
    assert result["checks"]["pose_count"] == 3
    assert result["checks"]["poses_validation"]["valid"] is True
    assert result["checks"]["database_unmodified"] is True


def test_export_rejects_non_empty_output_dir(tmp_path: Path):
    database = tmp_path / "map.db"
    database.write_bytes(b"database-bytes")
    output = tmp_path / "export"
    output.mkdir()
    (output / "existing.txt").write_text("x", encoding="utf-8")
    with pytest.raises(FileExistsError):
        db_tools.export_database(database, output)


def test_export_fails_when_tool_returns_error(tmp_path, monkeypatch):
    database = tmp_path / "map.db"
    database.write_bytes(b"database-bytes")
    monkeypatch.setattr(db_tools.shutil, "which", lambda name: "/fake/rtabmap-export")
    monkeypatch.setattr(
        db_tools.subprocess,
        "run",
        lambda command, **kwargs: SimpleNamespace(returncode=1, stdout="", stderr="boom"),
    )
    result = db_tools.export_database(database, tmp_path / "export")
    assert result["passed"] is False
    assert result["checks"]["returncode_zero"] is False