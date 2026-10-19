"""cloud_xyzi 回归：布局拒绝、字段投影、stamp/frame 语义保持（纯 numpy，无 ROS）。"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from hei_robot_slam import slam_contract as C
from hei_robot_slam.cloud_xyzi import (
    NUMPY_BY_DATATYPE,
    XYZI_FIELD_SPEC,
    XYZI_POINT_STEP,
    build_xyzi_data,
    convert,
)

ALL_FIELDS = ("x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z", "curvature")


def make_cloud(
    width: int = 3,
    height: int = 1,
    is_bigendian: bool = False,
    fields: tuple = ALL_FIELDS,
    row_step: int | None = None,
    data: bytes | None = None,
    datatypes: tuple | None = None,
    extra: int = 0,
):
    dtypes = datatypes or tuple(7 for _ in fields)
    offsets = []
    types = []
    items = []
    offset = 0
    for name, datatype in zip(fields, dtypes):
        offsets.append(offset)
        np_type = np.dtype(NUMPY_BY_DATATYPE[datatype])
        types.append(np_type)
        offset += np_type.itemsize
    point_step = offset
    point_fields = [
        SimpleNamespace(name=name, offset=field_offset, datatype=datatype, count=1)
        for name, field_offset, datatype in zip(fields, offsets, dtypes)
    ]
    n = width * height
    array = np.zeros(n, dtype=[(name, np_type) for name, np_type in zip(fields, types)])
    for index, name in enumerate(fields):
        if name in ("x", "y", "z", "intensity"):
            array[name] = np.arange(n, dtype=np.float32) + index
    if data is None:
        data = array.tobytes()
    if row_step is None:
        row_step = point_step * width
    return SimpleNamespace(
        fields=point_fields,
        width=width,
        height=height,
        point_step=point_step,
        row_step=row_step,
        is_bigendian=is_bigendian,
        data=data,
    )


def test_convert_projects_to_fixed_xyzi_and_keeps_count():
    result = convert(make_cloud(width=3))
    assert result["width"] == 3
    assert result["height"] == 1
    assert result["fields"] == list(C.CLOUD_ADAPTED_FIELDS)
    assert len(result["data"]) == 3 * XYZI_POINT_STEP
    layout = result["stats"]["layout"]
    assert layout["height"] == 1
    assert layout["point_step"] == 32
    assert layout["fields"] == list(ALL_FIELDS)


def test_convert_drops_nonfinite_xyz_and_counts_them():
    cloud = make_cloud(width=3)
    array = np.frombuffer(cloud.data, dtype=np.float32).reshape(3, 8).copy()
    array[1, 0] = np.nan
    cloud.data = array.tobytes()
    result = convert(cloud)
    assert result["width"] == 2
    assert result["stats"]["dropped_nonfinite"] == 1


def test_convert_reports_zero_unestimated_normals():
    result = convert(make_cloud(width=2))
    assert result["stats"]["input_has_normal_fields"] is True
    assert result["stats"]["input_nonzero_normal_points"] == 0


def test_convert_without_intensity_sets_zeros():
    result = convert(make_cloud(width=2, fields=("x", "y", "z")))
    assert result["width"] == 2
    assert len(result["data"]) == 2 * XYZI_POINT_STEP


def test_rejects_big_endian():
    with pytest.raises(ValueError):
        convert(make_cloud(width=2, is_bigendian=True))


def test_rejects_organized_cloud():
    with pytest.raises(ValueError):
        convert(make_cloud(width=2, height=2))


def test_rejects_row_padding():
    cloud = make_cloud(width=2)
    cloud.row_step = cloud.point_step * cloud.width + 8
    with pytest.raises(ValueError):
        convert(cloud)


def test_rejects_malformed_data_length():
    cloud = make_cloud(width=2)
    cloud.data = cloud.data[:-4]
    with pytest.raises(ValueError):
        convert(cloud)


def test_rejects_missing_xyz_fields():
    with pytest.raises(ValueError):
        convert(make_cloud(width=2, fields=("x", "y", "intensity")))


def test_rejects_non_float32_xyz():
    with pytest.raises(ValueError):
        convert(make_cloud(width=2, datatypes=(8, 7, 7, 7, 7, 7, 7, 7)))


def test_build_xyzi_data_layout():
    data = build_xyzi_data(
        np.array([1.0], dtype=np.float32),
        np.array([2.0], dtype=np.float32),
        np.array([3.0], dtype=np.float32),
        np.array([4.0], dtype=np.float32),
    )
    assert len(data) == XYZI_POINT_STEP
    values = np.frombuffer(data, dtype=np.float32)
    assert list(values) == [1.0, 2.0, 3.0, 4.0]


def test_field_spec_offsets_are_contiguous_float32():
    names = [spec[0] for spec in XYZI_FIELD_SPEC]
    offsets = [spec[1] for spec in XYZI_FIELD_SPEC]
    assert names == ["x", "y", "z", "intensity"]
    assert offsets == [0, 4, 8, 12]
    assert all(spec[2] == 7 and spec[3] == 1 for spec in XYZI_FIELD_SPEC)
    assert XYZI_POINT_STEP == 16
