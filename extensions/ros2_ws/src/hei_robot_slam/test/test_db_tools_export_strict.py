"""P2 强化回归：导出实体校验（PLY body/有限 XYZ/pose 9 列有限值）。

历史问题：旧实现只读 PLY 头的 element vertex 计数，header 声称 1000 顶点但 body 为空
也会 passed；poses 也不校验列数/有限性。现在 db_tools 必须验证实体数据。
"""

from __future__ import annotations

import struct
from pathlib import Path
from types import SimpleNamespace

from hei_robot_slam import db_tools

GOOD_POSES = "1.0 0 0 0 0 0 0 1 1\n2.0 0.1 0 0 0 0 0 1 2\n"
GOOD_PLY = (
    "ply\nformat ascii 1.0\n"
    "element vertex 2\n"
    "property float x\nproperty float y\nproperty float z\n"
    "end_header\n0 0 0\n1 1 1\n"
)


def _run_fake_export(tmp_path: Path, monkeypatch, ply_text: str, poses_text: str) -> dict:
    database = tmp_path / "map.db"
    database.write_bytes(b"database-bytes")

    def fake_run(command, **kwargs):
        output_dir = Path(command[command.index("--output_dir") + 1])
        (output_dir / "map.ply").write_text(ply_text, encoding="utf-8")
        (output_dir / "map_poses.txt").write_text(poses_text, encoding="utf-8")
        fake_run.command = list(command)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(db_tools.shutil, "which", lambda name: "/fake/rtabmap-export")
    monkeypatch.setattr(db_tools.subprocess, "run", fake_run)
    result = db_tools.export_database(database, tmp_path / "export")
    result["_command"] = fake_run.command
    return result


def test_export_accepts_complete_cloud_and_pose(tmp_path, monkeypatch):
    result = _run_fake_export(tmp_path, monkeypatch, GOOD_PLY, GOOD_POSES)
    assert result["passed"] is True
    assert result["checks"]["cloud_geometry"]["valid"] is True
    assert result["checks"]["poses_validation"]["valid"] is True
    # 实测 0.23.13 必须显式传 --voxel，否则 assembled cloud 为空。
    assert "--voxel" in result["_command"]
    assert result["_command"][result["_command"].index("--voxel") + 1] == "0.05"
    assert "--max_range" in result["_command"]


def test_export_rejects_ply_header_claiming_vertices_without_body(tmp_path, monkeypatch):
    result = _run_fake_export(
        tmp_path,
        monkeypatch,
        "ply\nformat ascii 1.0\nelement vertex 1000\nproperty float x\nproperty float y\nproperty float z\nend_header\n",
        GOOD_POSES,
    )
    assert result["passed"] is False
    geometry = result["checks"]["cloud_geometry"]
    assert geometry["valid"] is False
    assert geometry["parsed_vertices"] == 0


def test_export_rejects_truncated_ply_body(tmp_path, monkeypatch):
    result = _run_fake_export(
        tmp_path,
        monkeypatch,
        "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 1 1\n",
        GOOD_POSES,
    )
    assert result["passed"] is False
    assert result["checks"]["cloud_geometry"]["valid"] is False


def test_export_rejects_nonfinite_ply_xyz(tmp_path, monkeypatch):
    result = _run_fake_export(
        tmp_path,
        monkeypatch,
        "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nend_header\nnan 0 0\n1 1 1\n",
        GOOD_POSES,
    )
    assert result["passed"] is False
    assert result["checks"]["cloud_geometry"]["nonfinite_xyz"] == 1


def test_export_rejects_bad_pose_rows(tmp_path, monkeypatch):
    result = _run_fake_export(
        tmp_path,
        monkeypatch,
        GOOD_PLY,
        "1.0 nan 0 0 0 0 0 1 1\n2.0 0.1 0 0 0 0 0 1\n",
    )
    assert result["passed"] is False
    validation = result["checks"]["poses_validation"]
    assert validation["valid"] is False
    assert validation["nonfinite_rows"] == 1


def test_validate_pose_file_accepts_9_columns(tmp_path: Path):
    poses = tmp_path / "poses.txt"
    poses.write_text(GOOD_POSES, encoding="utf-8")
    result = db_tools.validate_pose_file(poses)
    assert result["valid"] is True and result["rows"] == 2


def test_binary_ply_geometry_validation(tmp_path: Path):
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        "element vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n"
    ).encode("ascii")
    body = struct.pack("<6f", 0.0, 0.0, 0.0, 1.0, 1.0, 1.0)
    good = tmp_path / "good.ply"
    good.write_bytes(header + body)
    result = db_tools.validate_ply_geometry(good)
    assert result["valid"] is True
    assert result["actual_vertex_count"] == 2
    truncated = tmp_path / "truncated.ply"
    truncated.write_bytes(header + body[:-4])
    bad = db_tools.validate_ply_geometry(truncated)
    assert bad["valid"] is False


def test_ply_header_only_is_not_valid_geometry(tmp_path: Path):
    path = tmp_path / "header_only.ply"
    path.write_text("ply\nformat ascii 1.0\nelement vertex 5\nproperty float x\nend_header\n", encoding="utf-8")
    result = db_tools.validate_ply_geometry(path)
    assert result["valid"] is False


def test_ply_rejects_vertex_not_first_element(tmp_path: Path):
    path = tmp_path / "face_first.ply"
    path.write_text(
        "ply\nformat ascii 1.0\n"
        "element face 1\nproperty list uchar int vertex_indices\n"
        "element vertex 1\nproperty float x\nproperty float y\nproperty float z\n"
        "end_header\n3 0 0 0\n0 0 0\n",
        encoding="utf-8",
    )
    result = db_tools.validate_ply_geometry(path)
    assert result["valid"] is False
    assert any("vertex 非首元素" in reason for reason in result["reasons"])


def test_ply_rejects_integer_xyz_properties(tmp_path: Path):
    path = tmp_path / "int_xyz.ply"
    path.write_text(
        "ply\nformat ascii 1.0\nelement vertex 1\n"
        "property int x\nproperty int y\nproperty int z\nend_header\n0 0 0\n",
        encoding="utf-8",
    )
    result = db_tools.validate_ply_geometry(path)
    assert result["valid"] is False
    assert any("类型不支持" in reason for reason in result["reasons"])