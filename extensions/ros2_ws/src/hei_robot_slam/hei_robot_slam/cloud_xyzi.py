"""PointCloud2 -> XYZI 最薄投影（纯 numpy，无 ROS import，便于无 ROS 环境回归）。

为什么需要它：FAST-LIO publish_frame_body() 用 pcl::PointXYZINormal 打包
/cloud_registered_body，但源码只赋值 x/y/z/intensity；normal_x/normal_y/normal_z
与 curvature 字段**不是估计出来的法线**。RTAB-Map 的 3D 点云入口若把零法线当有效
信息，会污染 PointToPlane 注册。本模块只把点云显式投影成固定 XYZI 布局：

* 保留：header.stamp、header.frame_id、逐点 xyz、intensity（MID-360S 反射率）；
* 丢弃：normal_*/curvature 等未被估计的字段；
* 不做去畸变、不降采样、不累计多帧；非有限 xyz 点会被计数后剔除。
"""

from __future__ import annotations

from typing import Any

import numpy as np

#: ROS sensor_msgs/PointField 常量 -> numpy dtype（PointCloud2 固定小端）
NUMPY_BY_DATATYPE = {
    1: "<i1",
    2: "<u1",
    3: "<i2",
    4: "<u2",
    5: "<i4",
    6: "<u4",
    7: "<f4",
    8: "<f8",
}

#: 输出点云字段（名称, 字节偏移, PointField datatype=7 FLOAT32, count=1）
XYZI_FIELD_SPEC = (
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("intensity", 12, 7, 1),
)
XYZI_POINT_STEP = 16
XYZI_FIELD_NAMES = tuple(spec[0] for spec in XYZI_FIELD_SPEC)

#: 本适配层只接受 FAST-LIO 的当前布局：小端、未组织（height<=1）、紧凑无行填充、
#: x/y/z（以及存在时的 intensity）为 float32、count=1。其他布局一律显式拒绝，
#: 绝不静默按错误 offset 读数据。



def field_names(msg: Any) -> list[str]:
    """PointCloud2 的字段名列表（保持消息顺序）。"""
    return [str(field.name) for field in msg.fields]


def _structured_dtype(msg: Any) -> np.dtype:
    formats: list[Any] = []
    names: list[str] = []
    offsets: list[int] = []
    for field in msg.fields:
        if int(field.datatype) not in NUMPY_BY_DATATYPE:
            raise ValueError(f"不支持的 PointField datatype={field.datatype}（字段 {field.name}）")
        base = np.dtype(NUMPY_BY_DATATYPE[int(field.datatype)])
        count = int(field.count)
        formats.append(base if count == 1 else (base, (count,)))
        names.append(str(field.name))
        offsets.append(int(field.offset))
    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(msg.point_step),
        }
    )


def structured_points(msg: Any) -> np.ndarray:
    """把 PointCloud2.data 视图化为结构化数组（只读视图，不复制）。"""
    n = int(msg.width) * int(msg.height)
    dtype = _structured_dtype(msg)
    return np.frombuffer(msg.data, dtype=dtype, count=n)


def validate_layout(msg: Any) -> dict[str, Any]:
    """校验 PointCloud2 布局是否属于本适配层支持的 FAST-LIO 当前布局。

    只支持：小端（is_bigendian=False）、未组织（height<=1）、紧凑无行填充
    （len(data) == width*height*point_step 且 row_step == point_step*width）、
    x/y/z 为 float32 count=1、可选 intensity 为 float32 count=1。
    其他布局（大端、行填充、畸形 data、非 float32 xyz）**显式拒绝**，不静默错读。
    """
    if bool(getattr(msg, "is_bigendian", False)):
        raise ValueError(
            "不支持大端 PointCloud2（is_bigendian=True）：本适配层只处理 FAST-LIO 的小端布局"
        )
    width = int(msg.width)
    height = int(msg.height)
    point_step = int(msg.point_step)
    row_step = int(getattr(msg, "row_step", 0))
    if width < 0 or height < 0:
        raise ValueError(f"非法点云尺寸 width={width} height={height}")
    if height > 1:
        raise ValueError(
            f"不支持组织化/行填充点云（height={height}）：本适配层只处理 FAST-LIO 非组织云"
        )
    if point_step <= 0:
        raise ValueError(f"非法 point_step={point_step}")
    expected = point_step * width * height
    if len(msg.data) != expected:
        raise ValueError(
            f"点云 data 长度 {len(msg.data)} 与 width*height*point_step={expected} 不一致"
            "（行填充/畸形数据不支持）"
        )
    if row_step and row_step != point_step * width:
        raise ValueError(
            f"不支持行填充：row_step={row_step} != point_step*width={point_step * width}"
        )

    required = ("x", "y", "z")
    seen: dict[str, Any] = {}
    for field in msg.fields:
        name = str(field.name)
        if name in seen:
            raise ValueError(f"PointCloud2 字段重名：{name}")
        seen[name] = field
        offset = int(field.offset)
        count = int(field.count)
        if count <= 0:
            raise ValueError(f"字段 {name} count 非法：{count}")
        if int(field.datatype) not in NUMPY_BY_DATATYPE:
            raise ValueError(f"字段 {name} datatype={field.datatype} 不受支持")
        size = np.dtype(NUMPY_BY_DATATYPE[int(field.datatype)]).itemsize * count
        if offset < 0 or offset + size > point_step:
            raise ValueError(f"字段 {name} 越界：offset={offset} size={size} point_step={point_step}")
    for name in required:
        if name not in seen:
            raise ValueError(f"输入点云缺少必需字段 {name!r}，实际字段 {sorted(seen)}")
        field = seen[name]
        if int(field.datatype) != 7 or int(field.count) != 1:
            raise ValueError(
                f"字段 {name} 必须是 float32 count=1，实际 datatype={field.datatype} count={field.count}"
            )
    if "intensity" in seen:
        field = seen["intensity"]
        if int(field.datatype) != 7 or int(field.count) != 1:
            raise ValueError(
                "intensity 必须是 float32 count=1，实际 "
                f"datatype={field.datatype} count={field.count}"
            )
    return {
        "width": width,
        "height": height,
        "point_step": point_step,
        "row_step": row_step,
        "fields": [str(field.name) for field in msg.fields],
    }

def build_xyzi_data(x: np.ndarray, y: np.ndarray, z: np.ndarray, intensity: np.ndarray) -> bytes:
    """把 4 列 float32 组成固定 XYZI 布局的字节串。"""
    n = int(x.shape[0])
    out = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    out["x"] = np.asarray(x, dtype=np.float32)
    out["y"] = np.asarray(y, dtype=np.float32)
    out["z"] = np.asarray(z, dtype=np.float32)
    out["intensity"] = np.asarray(intensity, dtype=np.float32)
    return out.tobytes()


def convert(msg: Any) -> dict[str, Any]:
    """PointCloud2 -> XYZI 投影；返回输出字节、宽高与统计（纯函数，无 ROS 依赖）。"""
    layout = validate_layout(msg)
    names = layout["fields"]
    missing = [name for name in ("x", "y", "z") if name not in names]
    if missing:
        raise ValueError(f"输入点云缺少字段 {missing}，实际字段 {names}")

    points = structured_points(msg)
    x = np.asarray(points["x"], dtype=np.float32)
    y = np.asarray(points["y"], dtype=np.float32)
    z = np.asarray(points["z"], dtype=np.float32)
    if "intensity" in names:
        intensity = np.asarray(points["intensity"], dtype=np.float32)
    else:
        intensity = np.zeros_like(x)

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    dropped_nonfinite = int(points.shape[0] - int(finite.sum()))

    # 取证：输入里是否带有未被估计的法线字段；若有，统计非零法线点数。
    normal_fields = [name for name in ("normal_x", "normal_y", "normal_z") if name in names]
    has_normals = len(normal_fields) == 3
    nonzero_normals = 0
    if has_normals:
        nx = np.asarray(points["normal_x"], dtype=np.float32)
        ny = np.asarray(points["normal_y"], dtype=np.float32)
        nz = np.asarray(points["normal_z"], dtype=np.float32)
        norm2 = nx * nx + ny * ny + nz * nz
        nonzero_normals = int(np.count_nonzero(np.isfinite(norm2) & (norm2 > 0.0)))

    out_x = x[finite]
    out_y = y[finite]
    out_z = z[finite]
    out_i = intensity[finite]
    return {
        "data": build_xyzi_data(out_x, out_y, out_z, out_i),
        "width": int(out_x.shape[0]),
        "height": 1,
        "fields": [spec[0] for spec in XYZI_FIELD_SPEC],
        "stats": {
            "input_points": int(points.shape[0]),
            "output_points": int(out_x.shape[0]),
            "dropped_nonfinite": dropped_nonfinite,
            "source_fields": names,
            "layout": layout,
            "input_has_normal_fields": has_normals,
            "input_nonzero_normal_points": nonzero_normals,
        },
    }