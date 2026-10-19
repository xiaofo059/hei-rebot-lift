"""RTAB-Map 数据库检查 / 导出 / SHA256 取证工具（只读数据库，绝不 -d）。

用法（source 好 ROS 环境与本地 RTAB-Map overlay 之后）：

    slam_db_tools sha256 DB
    slam_db_tools inspect DB --json OUT.json
    slam_db_tools export DB --output-dir OUT_DIR --json OUT.json

导出调用 `rtabmap-export` 的**本机 0.23.13 实测接口**：
`--cloud --scan --poses --poses_format 11 --opt 2`（`--opt 2` = 直接用数据库中
已优化位姿，不重新全局优化；导出产物另行校验非空、PLY 顶点数、位姿行数）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import struct
import sqlite3
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

#: RTAB-Map 0.23.13 DB Link.type -> 语义（本机安装头文件
#: extensions/third_party/rtabmap_local/prefix/opt/ros/humble/include/rtabmap-0.23/
#: rtabmap/core/Link.h 的 Link::Type 枚举，逐项核对）。
#: 关键：type=2 kLocalSpaceClosure 才是**由当前位姿附近空间检测产生的自动几何闭环**；
#: type=1 是全局描述子闭环（无图像/无全局描述子时通常为 0）；type=3 是时间邻近闭环；
#: type=4 是**用户手工**闭环，不能计入自动回环成功。
LINK_TYPES = {
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
AUTO_LOOP_CLOSURE_TYPES = (1, 2, 3)
PRIMARY_AUTO_LOOP_CLOSURE_TYPE = 2


def _sha256(path: str | os.PathLike[str]) -> str:
    import hashlib

    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _db_connect_readonly(path: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{path}?mode=ro", uri=True)


#: PLY 属性类型 -> 字节数（用于 binary PLY 顶点 body 长度校验）
_PLY_TYPE_SIZES = {
    "char": 1, "uchar": 1, "int8": 1, "uint8": 1,
    "short": 2, "ushort": 2, "int16": 2, "uint16": 2,
    "int": 4, "uint": 4, "int32": 4, "uint32": 4,
    "float": 4, "float32": 4, "double": 8, "float64": 8,
}
_PLY_FORMATS = ("ascii", "binary_little_endian", "binary_big_endian")
#: rtabmap-export --poses_format 11 的输出列数：stamp x y z qx qy qz qw id
POSE_COLUMNS_EXPECTED = 9


def read_ply_header(path: str | os.PathLike[str]) -> dict[str, Any]:
    """解析 PLY 头（ascii/binary），返回 format/elements/data_offset/file_size。"""
    with open(path, "rb") as handle:
        elements: list[dict[str, Any]] = []
        current: dict[str, Any] | None = None
        file_format: str | None = None
        header_lines = 0
        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError(f"PLY 头未以 end_header 结束：{path}")
            header_lines += 1
            if header_lines > 2000:
                raise ValueError(f"PLY 头过长（>2000 行）：{path}")
            text = raw.decode("ascii", errors="replace").strip()
            if text == "end_header":
                break
            if not text:
                continue
            parts = text.split()
            keyword = parts[0]
            if keyword in ("ply", "comment", "obj_info"):
                continue
            if keyword == "format":
                if len(parts) != 3 or parts[1] not in _PLY_FORMATS:
                    raise ValueError(f"PLY format 非法：{text!r}")
                file_format = parts[1]
            elif keyword == "element":
                if len(parts) != 3:
                    raise ValueError(f"PLY element 行非法：{text!r}")
                current = {"name": parts[1], "count": int(parts[2]), "properties": []}
                elements.append(current)
            elif keyword == "property":
                if current is None:
                    raise ValueError("PLY property 出现在 element 之前")
                if len(parts) == 5 and parts[1] == "list":
                    current["properties"].append(
                        {"is_list": True, "count_type": parts[2], "item_type": parts[3], "name": parts[4]}
                    )
                elif len(parts) == 3:
                    if parts[1] not in _PLY_TYPE_SIZES:
                        raise ValueError(f"未知 PLY 属性类型：{parts[1]!r}")
                    current["properties"].append({"is_list": False, "type": parts[1], "name": parts[2]})
                else:
                    raise ValueError(f"PLY property 行非法：{text!r}")
            else:
                raise ValueError(f"未知 PLY 头关键字：{keyword!r}")
        data_offset = handle.tell()
    if file_format is None:
        raise ValueError(f"PLY 缺 format 行：{path}")
    return {
        "path": str(path),
        "format": file_format,
        "elements": elements,
        "data_offset": data_offset,
        "file_size": os.path.getsize(path),
    }


def _vertex_element(header: dict[str, Any]) -> dict[str, Any]:
    for element in header["elements"]:
        if element["name"] == "vertex":
            return element
    raise ValueError(f"PLY 缺 element vertex：{header['path']}")


def validate_ply_geometry(
    path: str | os.PathLike[str], expected_min_vertices: int = 1
) -> dict[str, Any]:
    """校验 PLY **实体数据**：vertex 数与 body 实际长度一致、xyz 有限。

    P2 强化：只看头部 element vertex 声明不算数；截断 body、非有限 XYZ 必须判失败。
    """
    result: dict[str, Any] = {"valid": False, "reasons": []}
    header = read_ply_header(path)
    result["format"] = header["format"]
    result["data_offset"] = header["data_offset"]
    result["file_size"] = header["file_size"]
    element_names = [element["name"] for element in header["elements"]]
    # 本解析器只支持本工具 rtabmap-export 的输出布局：vertex 必须是第一个元素。
    if not element_names or element_names[0] != "vertex":
        result["reasons"].append(f"不支持 vertex 非首元素布局：elements={element_names}")
        return result
    vertex = _vertex_element(header)
    count = int(vertex["count"])
    result["header_vertex_count"] = count
    if count < expected_min_vertices:
        result["reasons"].append(f"header vertex count {count} < {expected_min_vertices}")
        return result
    if any(prop["is_list"] for prop in vertex["properties"]):
        result["reasons"].append("vertex 元素含 list 属性，不支持")
        return result
    property_names = [prop["name"] for prop in vertex["properties"]]
    result["vertex_properties"] = property_names
    missing = [name for name in ("x", "y", "z") if name not in property_names]
    if missing:
        result["reasons"].append(f"vertex 缺 xyz 属性：{missing}")
        return result
    xyz_indexes = [property_names.index(name) for name in ("x", "y", "z")]
    # 只支持 float32/float64 的 xyz；整数坐标显式拒绝，避免按字节数误判类型。
    for name in ("x", "y", "z"):
        prop_type = vertex["properties"][property_names.index(name)]["type"]
        if prop_type not in ("float", "float32", "double", "float64"):
            result["reasons"].append(f"xyz 属性 {name} 类型不支持：{prop_type}（只支持 float/double）")
            return result

    if header["format"] == "ascii":
        parsed = 0
        nonfinite = 0
        bad_lines = 0
        with open(path, "rb") as handle:
            handle.seek(header["data_offset"])
            for _ in range(count):
                line = handle.readline()
                if not line:
                    result["reasons"].append(f"ASCII PLY body 截断：仅 {parsed}/{count} 行")
                    break
                parts = line.decode("ascii", errors="replace").split()
                if len(parts) < len(property_names):
                    bad_lines += 1
                    continue
                try:
                    xyz = [float(parts[index]) for index in xyz_indexes]
                except ValueError:
                    bad_lines += 1
                    continue
                parsed += 1
                if not all(math.isfinite(value) for value in xyz):
                    nonfinite += 1
        result["parsed_vertices"] = parsed
        result["bad_vertex_lines"] = bad_lines
        result["nonfinite_xyz"] = nonfinite
        if bad_lines:
            result["reasons"].append(f"ASCII PLY 有 {bad_lines} 行无法解析 xyz")
        if parsed != count:
            result["reasons"].append(f"ASCII PLY body 顶点数 {parsed} != header {count}")
        if nonfinite:
            result["reasons"].append(f"ASCII PLY 有 {nonfinite} 个非有限 XYZ")
    else:
        endian = "<" if header["format"] == "binary_little_endian" else ">"
        offsets: list[int] = []
        offset = 0
        for prop in vertex["properties"]:
            offsets.append(offset)
            offset += _PLY_TYPE_SIZES[prop["type"]]
        vertex_size = offset
        body_available = header["file_size"] - header["data_offset"]
        body_needed = count * vertex_size
        result["vertex_size_bytes"] = vertex_size
        result["body_bytes_needed"] = body_needed
        result["body_bytes_available"] = body_available
        if body_available < body_needed:
            result["reasons"].append(
                f"binary PLY body 截断：{body_available} < {body_needed} 字节（vertex {count} x {vertex_size}）"
            )
            return result
        nonfinite = 0
        xyz_structs = []
        for name in ("x", "y", "z"):
            index = property_names.index(name)
            prop_type = vertex["properties"][index]["type"]
            struct_code = "f" if prop_type in ("float", "float32") else "d"
            xyz_structs.append((offsets[index], struct_code))
        with open(path, "rb") as handle:
            handle.seek(header["data_offset"])
            body = handle.read(body_needed)
        for start in range(0, body_needed, vertex_size):
            values = [struct.unpack_from(endian + code, body, start + prop_offset)[0] for prop_offset, code in xyz_structs]
            if not all(math.isfinite(value) for value in values):
                nonfinite += 1
        result["parsed_vertices"] = count
        result["nonfinite_xyz"] = nonfinite
        if nonfinite:
            result["reasons"].append(f"binary PLY 有 {nonfinite} 个非有限 XYZ")

    result["actual_vertex_count"] = result.get("parsed_vertices", 0)
    result["valid"] = not result["reasons"] and result["actual_vertex_count"] == count > 0
    return result


def read_ply_vertex_count(path: str | os.PathLike[str]) -> int:
    """只读 PLY 头中的 element vertex 数量（实体校验见 validate_ply_geometry）。"""
    return int(_vertex_element(read_ply_header(path))["count"])


def validate_pose_file(
    path: str | os.PathLike[str], expected_columns: int = POSE_COLUMNS_EXPECTED
) -> dict[str, Any]:
    """校验 pose 文本：每行 9 列（--poses_format 11）、全部有限数值。"""
    result: dict[str, Any] = {
        "valid": False,
        "expected_columns": expected_columns,
        "rows": 0,
        "invalid_rows": [],
        "nonfinite_rows": 0,
        "column_counts": {},
    }
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line_number, line in enumerate(handle, start=1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            columns = len(parts)
            result["column_counts"][str(columns)] = result["column_counts"].get(str(columns), 0) + 1
            if columns != expected_columns:
                result["invalid_rows"].append({"line": line_number, "reason": f"columns={columns}"})
                continue
            try:
                values = [float(part) for part in parts]
            except ValueError:
                result["invalid_rows"].append({"line": line_number, "reason": "non-numeric"})
                continue
            if not all(math.isfinite(value) for value in values):
                result["nonfinite_rows"] += 1
                result["invalid_rows"].append({"line": line_number, "reason": "non-finite"})
                continue
            result["rows"] += 1
    result["valid"] = result["rows"] > 0 and not result["invalid_rows"]
    return result


def count_pose_lines(path: str | os.PathLike[str]) -> int:
    """统计 pose 文件有效数据行（兼容旧接口；严格校验见 validate_pose_file）。"""
    return int(validate_pose_file(path)["rows"])


def inspect_database(path: str | os.PathLike[str], run_rtabmap_info: bool = True) -> dict[str, Any]:
    """只读检查数据库：节点数、点云数据、link 类型分布、回环证据。"""
    db = Path(path).expanduser().resolve()
    if not db.is_file():
        raise FileNotFoundError(f"数据库不存在：{db}")
    result: dict[str, Any] = {
        "kind": "rtabmap_database_inspection",
        "database": str(db),
        "size_bytes": db.stat().st_size,
        "sha256": _sha256(db),
        "tool_version": None,
    }

    connection = _db_connect_readonly(db)
    try:
        cursor = connection.cursor()
        result["integrity_check"] = str(cursor.execute("PRAGMA integrity_check").fetchone()[0])
        tables = [row[0] for row in cursor.execute("SELECT name FROM sqlite_master WHERE type = \"table\" ORDER BY name")]
        result["tables"] = tables
        if "Node" not in tables:
            raise ValueError(f"数据库缺少 Node 表：{tables}")

        node_columns = [row[1] for row in cursor.execute("PRAGMA table_info(Node)")]
        result["node_columns"] = node_columns
        result["nodes"] = int(cursor.execute("SELECT COUNT(*) FROM Node").fetchone()[0])
        # 传感器数据实际存放在 Data 表（scan/depth/image 列），不在 Node 表；
        # 必须先 PRAGMA 核验列名再统计，避免 has_stored_cloud 误判。
        result["data_table_columns"] = (
            [row[1] for row in cursor.execute("PRAGMA table_info(Data)")] if "Data" in tables else None
        )
        result["data_rows"] = (
            int(cursor.execute("SELECT COUNT(*) FROM Data").fetchone()[0]) if "Data" in tables else 0
        )
        result["nodes_with_scan_data"] = None
        if "Data" in tables and "scan" in (result["data_table_columns"] or []):
            result["nodes_with_scan_data"] = int(
                cursor.execute("SELECT COUNT(*) FROM Data WHERE scan IS NOT NULL AND length(scan) > 0").fetchone()[0]
            )
        if "data" in node_columns:
            result["nodes_with_data_column"] = int(
                cursor.execute("SELECT COUNT(*) FROM Node WHERE data IS NOT NULL AND length(data) > 0").fetchone()[0]
            )
        else:
            result["nodes_with_data_column"] = None
        if "map_id" in node_columns:
            result["map_ids"] = [int(row[0]) for row in cursor.execute("SELECT DISTINCT map_id FROM Node ORDER BY map_id")]
        if "stamp" in node_columns:
            row = cursor.execute("SELECT MIN(stamp), MAX(stamp) FROM Node").fetchone()
            result["node_stamp_range"] = [row[0], row[1]]

        result["has_link_table"] = "Link" in tables
        if "Link" in tables:
            link_columns = [row[1] for row in cursor.execute("PRAGMA table_info(Link)")]
            result["link_columns"] = link_columns
            type_rows = cursor.execute("SELECT type, COUNT(*) FROM Link GROUP BY type ORDER BY type").fetchall()
            link_type_counts = {str(int(row[0])): int(row[1]) for row in type_rows}
            result["link_type_counts"] = link_type_counts
            result["link_type_names"] = {
                str(int(row[0])): LINK_TYPES.get(int(row[0]), f"unknown_{int(row[0])}") for row in type_rows
            }
            result["link_type_name_counts"] = {
                LINK_TYPES.get(int(row[0]), f"unknown_{int(row[0])}"): int(row[1]) for row in type_rows
            }
            result["links_total"] = int(sum(link_type_counts.values()))
            result["auto_loop_closure_links"] = {
                LINK_TYPES[link_type]: int(link_type_counts.get(str(link_type), 0))
                for link_type in AUTO_LOOP_CLOSURE_TYPES
            }
            result["primary_auto_loop_closure_type"] = LINK_TYPES[PRIMARY_AUTO_LOOP_CLOSURE_TYPE]
            result["primary_auto_loop_closure_links"] = int(
                link_type_counts.get(str(PRIMARY_AUTO_LOOP_CLOSURE_TYPE), 0)
            )
            result["user_manual_closure_links"] = int(link_type_counts.get("4", 0))
            result["neighbor_links"] = int(link_type_counts.get("0", 0))
            result["neighbor_chain_ratio"] = round(
                result["neighbor_links"] / max(1, result["nodes"] - 1), 3
            )
            if "from_id" in link_columns and "to_id" in link_columns and result["primary_auto_loop_closure_links"]:
                rows = cursor.execute(
                    "SELECT from_id, to_id, type FROM Link WHERE type IN (1, 2, 3) "
                    "ORDER BY ABS(from_id-to_id) DESC LIMIT 20"
                ).fetchall()
                result["loop_closure_samples"] = [
                    {
                        "from_id": int(row[0]),
                        "to_id": int(row[1]),
                        "type": int(row[2]),
                        "type_name": LINK_TYPES.get(int(row[2]), "unknown"),
                        "id_gap": int(abs(row[0] - row[1])),
                    }
                    for row in rows
                ]
    finally:
        connection.close()

    if run_rtabmap_info:
        binary = shutil.which("rtabmap-info")
        if binary:
            completed = subprocess.run(
                [binary, str(db)], capture_output=True, text=True, timeout=300, check=False
            )
            result["rtabmap_info"] = {
                "returncode": int(completed.returncode),
                "stdout_tail": completed.stdout[-4000:],
                "stderr_tail": completed.stderr[-1000:],
            }
        else:
            result["rtabmap_info"] = {"returncode": None, "note": "rtabmap-info 不在 PATH（需 source 本地 overlay）"}

    stored_cloud = result.get("nodes_with_scan_data") or result.get("nodes_with_data_column")
    result["verdict_inputs"] = {
        "database_non_empty": result["nodes"] > 0,
        "has_stored_cloud": bool(stored_cloud),
        "stored_cloud_source": (
            "Data.scan"
            if result.get("nodes_with_scan_data")
            else ("Node.data" if result.get("nodes_with_data_column") else "none")
        ),
        "has_graph_links": bool(result.get("links_total")),
        "primary_auto_loop_closure_links": int(result.get("primary_auto_loop_closure_links", 0)),
        "auto_loop_closure_links": dict(result.get("auto_loop_closure_links", {})),
        "user_manual_closure_links": int(result.get("user_manual_closure_links", 0)),
        "honesty_note": (
            "自动回环只统计 Link::Type=1/2/3，其中 2=LocalSpaceClosure 为主要几何闭环；"
            "4=UserClosure 为用户手工闭环，绝不计入自动成功；0=neighbor、7=posePrior、9=gravity 均不算回环"
        ),
    }
    return result


def export_database(
    database: str | os.PathLike[str],
    output_dir: str | os.PathLike[str],
    timeout_s: float = 1800.0,
    voxel: str = "0.05",
    max_range: str = "0",
) -> dict[str, Any]:
    """调用 rtabmap-export 导出优化点云+轨迹；校验产物非空并记录 SHA256。

    实测（本机 0.23.13，Codex 完整回放 DB）：只传 --cloud --scan 时，
    "Create and assemble the clouds... 0 points / Export failed! The cloud is empty."；
    显式传 --voxel 0.05（--max_range 0 不裁剪）后可得到非空 PLY
    （本 DB：assemble 154829 点 -> 保存 48986 点）。因此这里固定显式传 voxel，
    并把实际命令与产物写进结果 JSON；不作为 CLI 猜参解释。
    """
    db = Path(database).expanduser().resolve()
    if not db.is_file():
        raise FileNotFoundError(f"数据库不存在：{db}")
    out = Path(output_dir).expanduser().resolve()
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"导出目录非空，拒绝覆盖：{out}")
    out.mkdir(parents=True, exist_ok=True)

    binary = shutil.which("rtabmap-export")
    if not binary:
        raise RuntimeError(
            "找不到 rtabmap-export：请先 source /opt/ros/humble/setup.bash 与 "
            "extensions/third_party/rtabmap_local/env.sh（见 extensions/tools/install_rtabmap_laptop.sh）"
        )

    command = [
        binary,
        "--cloud",
        "--scan",
        "--poses",
        "--poses_format",
        "11",
        "--opt",
        "2",
        "--voxel",
        str(voxel),
        "--max_range",
        str(max_range),
        "--output_dir",
        str(out),
        str(db),
    ]
    sha_before = _sha256(db)
    started = time.time()
    completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout_s, check=False)
    duration = time.time() - started

    log_path = out / "rtabmap_export.log"
    log_path.write_text(
        "COMMAND: " + " ".join(command) + "\n" + completed.stdout + "\n" + completed.stderr,
        encoding="utf-8",
    )

    result: dict[str, Any] = {
        "kind": "rtabmap_export",
        "database": str(db),
        "database_sha256_before": sha_before,
        "database_sha256_after": _sha256(db),
        "output_dir": str(out),
        "command": command,
        "returncode": int(completed.returncode),
        "duration_s": round(duration, 3),
        "log": str(log_path),
        "output_files": [],
        "checks": {},
    }
    result["checks"]["database_unmodified"] = result["database_sha256_before"] == result["database_sha256_after"]

    files = sorted(path for path in out.iterdir() if path.is_file() and path.name != "rtabmap_export.log")
    for path in files:
        result["output_files"].append(
            {"name": path.name, "size": path.stat().st_size, "sha256": _sha256(path)}
        )

    cloud_candidates = [out / item["name"] for item in result["output_files"] if item["name"].lower().endswith(".ply")]
    pose_candidates = [
        out / item["name"]
        for item in result["output_files"]
        if "pose" in item["name"].lower() and item["name"].lower().endswith((".txt", ".csv"))
    ]
    cloud_geometry: dict[str, Any] | None = None
    if cloud_candidates:
        try:
            cloud_geometry = validate_ply_geometry(cloud_candidates[0])
        except (ValueError, OSError) as exc:
            result["checks"]["cloud_parse_error"] = str(exc)
    poses_validation: dict[str, Any] | None = None
    if pose_candidates:
        try:
            poses_validation = validate_pose_file(pose_candidates[0])
        except (ValueError, OSError) as exc:
            result["checks"]["poses_parse_error"] = str(exc)

    # P2 强化：不只看 PLY 头声明，必须验证实体顶点数/body 长度/有限 XYZ；
    # pose 必须每行 9 列（--poses_format 11）且全部有限。
    result["checks"].update(
        {
            "returncode_zero": int(completed.returncode) == 0,
            "cloud_file_present": bool(cloud_candidates),
            "cloud_geometry": cloud_geometry,
            "cloud_vertices": (cloud_geometry or {}).get("actual_vertex_count"),
            "cloud_non_empty": bool(cloud_geometry and cloud_geometry.get("valid")),
            "poses_file_present": bool(pose_candidates),
            "poses_validation": poses_validation,
            "pose_count": (poses_validation or {}).get("rows"),
            "poses_non_empty": bool(poses_validation and poses_validation.get("valid")),
        }
    )
    result["passed"] = all(
        [
            result["checks"]["returncode_zero"],
            result["checks"]["database_unmodified"],
            result["checks"]["cloud_non_empty"],
            result["checks"]["poses_non_empty"],
        ]
    )
    return result


def sha256_command(path: str) -> dict[str, Any]:
    target = Path(path).expanduser().resolve()
    if not target.is_file():
        raise FileNotFoundError(f"文件不存在：{target}")
    return {
        "kind": "sha256",
        "path": str(target),
        "size_bytes": target.stat().st_size,
        "sha256": _sha256(target),
    }


def _write_json(path: str, payload: dict[str, Any]) -> None:
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="RTAB-Map 数据库只读取证/导出工具")
    subparsers = parser.add_subparsers(dest="command", required=True)

    sha_parser = subparsers.add_parser("sha256", help="输出文件 SHA256")
    sha_parser.add_argument("path")

    inspect_parser = subparsers.add_parser("inspect", help="只读检查数据库节点/链接/回环")
    inspect_parser.add_argument("database")
    inspect_parser.add_argument("--json", default="", help="结果 JSON 输出路径")
    inspect_parser.add_argument("--no-rtabmap-info", action="store_true")

    export_parser = subparsers.add_parser("export", help="导出优化点云+轨迹并校验")
    export_parser.add_argument("database")
    export_parser.add_argument("--output-dir", required=True, help="全新输出目录（非空则拒绝）")
    export_parser.add_argument("--voxel", default="0.05", help="导出点云体素（m）；0.23.13 实测必须显式给值，否则 cloud 为空")
    export_parser.add_argument("--max-range", default="0", help="导出点云最大距离（0=不裁剪）")
    export_parser.add_argument("--json", default="", help="结果 JSON 输出路径")

    args = parser.parse_args(argv)
    if args.command == "sha256":
        payload = sha256_command(args.path)
    elif args.command == "inspect":
        payload = inspect_database(args.database, run_rtabmap_info=not args.no_rtabmap_info)
    else:
        payload = export_database(args.database, args.output_dir, voxel=args.voxel, max_range=args.max_range)

    text = json.dumps(payload, ensure_ascii=False, indent=1)
    print(text)
    if getattr(args, "json", ""):
        _write_json(args.json, payload)
    if args.command == "export" and not payload.get("passed", False):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())