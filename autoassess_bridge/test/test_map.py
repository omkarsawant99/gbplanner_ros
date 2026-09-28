# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the plan's reference map (autoassess_bridge.map)."""

from __future__ import annotations

import os
import struct
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional

import numpy as np
import pytest
from conftest import SPACE, FakeClient, FakeNode, RecordingLog

from autoassess_bridge import map as refmap
from autoassess_bridge.map import MapFile, MapSource, MapUpdater, PointCloud

RESULT_VIEW = ("InspectionResultView", "1")
FLOAT32 = refmap.FLOAT32
UINT32 = refmap.UINT32


# --- PCD ------------------------------------------------------------------------------------


def test_binary_pcd_keeps_every_field_and_the_raw_points(tmp_path: Any) -> None:
    points = [(1.0, 2.0, 3.0, 0.0, 0.0, 1.0), (4.0, 5.0, 6.0, 1.0, 0.0, 0.0)]
    body = b"".join(struct.pack("<6f", *p) for p in points)
    path = _write(tmp_path / "m.pcd", _pcd_header("x y z normal_x normal_y normal_z", "4 4 4 4 4 4", "F F F F F F", "1 1 1 1 1 1", 2, "binary") + body)

    cloud = refmap.load_pcd(path)

    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        ("x", 0, FLOAT32, 1),
        ("y", 4, FLOAT32, 1),
        ("z", 8, FLOAT32, 1),
        ("normal_x", 12, FLOAT32, 1),
        ("normal_y", 16, FLOAT32, 1),
        ("normal_z", 20, FLOAT32, 1),
    ]
    assert cloud.point_step == 24
    assert (cloud.width, cloud.height) == (2, 1)
    assert cloud.data == body
    assert cloud.is_dense


def test_binary_pcd_with_rgba_and_label(tmp_path: Any) -> None:
    body = struct.pack("<3fII", 1.0, 2.0, 3.0, 0xFF102030, 7)
    path = _write(tmp_path / "l.pcd", _pcd_header("x y z rgba label", "4 4 4 4 4", "F F F U U", "1 1 1 1 1", 1, "binary") + body)

    cloud = refmap.load_pcd(path)

    assert [(f.name, f.datatype) for f in cloud.fields][3:] == [("rgba", UINT32), ("label", UINT32)]
    assert cloud.data == body


def test_ascii_pcd_gives_the_same_cloud_as_binary(tmp_path: Any) -> None:
    fields = ("x y z label", "4 4 4 4", "F F F U", "1 1 1 1")
    binary = _write(tmp_path / "b.pcd", _pcd_header(*fields, 2, "binary") + struct.pack("<3fI3fI", 1.5, -2.0, 3.25, 4, 0.5, 0.0, -1.0, 9))
    ascii_ = _write(tmp_path / "a.pcd", _pcd_header(*fields, 2, "ascii") + b"1.5 -2 3.25 4\n0.5 0 -1 9\n")

    assert refmap.load_pcd(ascii_) == refmap.load_pcd(binary)


def test_pcd_padding_fields_are_dropped_but_keep_their_bytes(tmp_path: Any) -> None:
    body = struct.pack("<3f4x", 1.0, 2.0, 3.0)
    path = _write(tmp_path / "p.pcd", _pcd_header("x y z _", "4 4 4 1", "F F F U", "1 1 1 4", 1, "binary") + body)

    cloud = refmap.load_pcd(path)

    assert [f.name for f in cloud.fields] == ["x", "y", "z"]
    assert cloud.point_step == 16
    assert cloud.data == body


def test_pcd_with_nan_points_is_not_dense(tmp_path: Any) -> None:
    body = struct.pack("<3f", float("nan"), 0.0, 0.0)
    path = _write(tmp_path / "n.pcd", _pcd_header("x y z", "4 4 4", "F F F", "1 1 1", 1, "binary") + body)

    assert not refmap.load_pcd(path).is_dense


def test_compressed_pcd_is_rejected_with_a_hint(tmp_path: Any) -> None:
    path = _write(tmp_path / "c.pcd", _pcd_header("x y z", "4 4 4", "F F F", "1 1 1", 1, "binary_compressed") + b"\0" * 16)

    with pytest.raises(ValueError) as excinfo:
        refmap.load_pcd(path)

    assert "binary_compressed" in str(excinfo.value)
    assert "ascii or binary" in str(excinfo.value)


def test_truncated_binary_pcd_is_rejected(tmp_path: Any) -> None:
    path = _write(tmp_path / "t.pcd", _pcd_header("x y z", "4 4 4", "F F F", "1 1 1", 2, "binary") + b"\0" * 12)

    with pytest.raises(ValueError):
        refmap.load_pcd(path)


# --- PLY fallback ---------------------------------------------------------------------------


def test_ascii_ply_vertices_become_xyz_and_packed_rgb(tmp_path: Any) -> None:
    path = _write(
        tmp_path / "m.ply",
        b"ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\n"
        b"property uchar red\nproperty uchar green\nproperty uchar blue\nelement face 1\n"
        b"property list uchar int vertex_indices\nend_header\n"
        b"1 2 3 255 0 16\n4 5 6 1 2 3\n3 0 1 1\n",
    )

    cloud = refmap.load_ply_vertices(path)

    assert [(f.name, f.offset, f.datatype) for f in cloud.fields] == [
        ("x", 0, FLOAT32),
        ("y", 4, FLOAT32),
        ("z", 8, FLOAT32),
        ("rgb", 12, FLOAT32),
    ]
    assert (cloud.width, cloud.height, cloud.point_step) == (2, 1, 16)
    rows = np.frombuffer(cloud.data, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    assert rows["x"].tolist() == [1.0, 4.0]
    assert rows["rgb"].tolist() == [0xFF0010, 0x010203]


def test_binary_ply_without_colours_gives_xyz_only(tmp_path: Any) -> None:
    body = struct.pack("<3d", 1.0, 2.0, 3.0)
    path = _write(
        tmp_path / "b.ply",
        b"ply\nformat binary_little_endian 1.0\nelement vertex 1\nproperty double x\n"
        b"property double y\nproperty double z\nend_header\n" + body,
    )

    cloud = refmap.load_ply_vertices(path)

    assert [f.name for f in cloud.fields] == ["x", "y", "z"]
    assert np.frombuffer(cloud.data, dtype="<f4").tolist() == [1.0, 2.0, 3.0]


def test_ply_whose_first_element_is_not_vertex_is_rejected(tmp_path: Any) -> None:
    path = _write(
        tmp_path / "f.ply",
        b"ply\nformat ascii 1.0\nelement face 0\nproperty list uchar int vertex_indices\n"
        b"element vertex 0\nproperty float x\nproperty float y\nproperty float z\nend_header\n",
    )

    with pytest.raises(ValueError):
        refmap.load_ply_vertices(path)


# --- choosing the map file ------------------------------------------------------------------


def test_choose_map_file_prefers_the_pcd_with_the_label() -> None:
    chosen = refmap.choose_map_file([1], [2, 3], ["Pointcloud", "Labeled cloud"], "labeled CLOUD")

    assert chosen == (3, "pcd", "Labeled cloud", True)


def test_choose_map_file_takes_the_first_pcd_without_a_label() -> None:
    assert refmap.choose_map_file([1], [2, 3], ["Pointcloud", "Labeled cloud"], None) == (2, "pcd", "Pointcloud", True)


def test_choose_map_file_falls_back_to_the_first_pcd_for_an_unknown_label() -> None:
    assert refmap.choose_map_file([1], [2], ["Pointcloud"], "nope") == (2, "pcd", "Pointcloud", False)


def test_choose_map_file_uses_the_mesh_when_there_is_no_pcd() -> None:
    assert refmap.choose_map_file([1, 5], [], [], None) == (1, "ply", None, True)


def test_choose_map_file_returns_none_for_an_empty_campaign() -> None:
    assert refmap.choose_map_file([], [], [], None) is None


# --- MapSource ------------------------------------------------------------------------------


def test_resolve_reads_the_campaign_files() -> None:
    client = _map_client()
    _add_campaign(client, "result-1", cdf=[10], pcd=[20, 21], labels=["Pointcloud", "Labeled"])

    chosen = _source(client, "/unused").resolve("result-1", None)

    assert chosen == MapFile(20, "mesh_984.pcd", "pcd", "Pointcloud")
    (call,) = client.instances.calls_to("retrieve")
    assert call["nodes"] == [(SPACE, "result-1")]
    assert call["sources"][0].external_id == "InspectionResultView"


def test_resolve_warns_when_the_label_is_not_found() -> None:
    client = _map_client()
    log = RecordingLog()
    _add_campaign(client, "result-1", cdf=[], pcd=[20], labels=["Pointcloud"])

    MapSource(client, "/unused", log=log).resolve("result-1", "Other")

    assert any("Other" in m for m in log.messages("warning"))


def test_resolve_fails_for_a_missing_campaign() -> None:
    with pytest.raises(LookupError):
        _source(_map_client(), "/unused").resolve("result-x", None)


def test_resolve_fails_for_a_campaign_without_files() -> None:
    client = _map_client()
    _add_campaign(client, "result-1", cdf=[], pcd=[], labels=[])

    with pytest.raises(LookupError):
        _source(client, "/unused").resolve("result-1", None)


def test_fetch_downloads_into_the_cache_once(tmp_path: Any) -> None:
    client = _map_client()
    source = _source(client, str(tmp_path))
    map_file = MapFile(20, "mesh_984.pcd", "pcd", "Pointcloud")

    first = source.fetch(map_file)
    second = source.fetch(map_file)

    assert first == second == os.path.join(str(tmp_path), "20", "mesh_984.pcd")
    assert open(first, "rb").read() == b"content of 20"
    assert client.files.downloads == [20]
    assert os.listdir(os.path.join(str(tmp_path), "20")) == ["mesh_984.pcd"]


def test_fetch_leaves_no_file_behind_when_the_download_fails(tmp_path: Any) -> None:
    client = _map_client()
    client.files.fail = True

    with pytest.raises(RuntimeError):
        _source(client, str(tmp_path)).fetch(MapFile(20, "mesh_984.pcd", "pcd", None))

    assert os.listdir(os.path.join(str(tmp_path), "20")) == []


def test_load_dispatches_on_the_kind(tmp_path: Any) -> None:
    pcd = _write(tmp_path / "m.pcd", _pcd_header("x y z", "4 4 4", "F F F", "1 1 1", 1, "binary") + struct.pack("<3f", 1, 2, 3))

    cloud = _source(_map_client(), str(tmp_path)).load(MapFile(1, "m.pcd", "pcd", None), pcd)

    assert cloud.width == 1


# --- MapUpdater -----------------------------------------------------------------------------


def test_updater_publishes_the_map_once_per_map_id() -> None:
    source = _FakeSource()
    published: List[Any] = []
    updater = _updater(source, published)

    updater.want("result-1")
    updater.tick(0.0)
    updater.want("result-1")
    updater.tick(1.0)

    assert source.loaded == ["result-1"]
    assert [(m, f.file_id) for m, f, _ in published] == [("result-1", 20)]


def test_updater_loads_a_new_map_when_the_plan_changes_it() -> None:
    source = _FakeSource()
    published: List[Any] = []
    updater = _updater(source, published)
    updater.want("result-1")
    updater.tick(0.0)

    updater.want("result-2")
    updater.tick(1.0)

    assert [m for m, _, _ in published] == ["result-1", "result-2"]


def test_updater_retries_a_failed_map_after_the_interval() -> None:
    source = _FakeSource(fail_times=1)
    published: List[Any] = []
    log = RecordingLog()
    updater = _updater(source, published, log=log, retry_s=30.0)
    updater.want("result-1")

    updater.tick(0.0)
    updater.tick(10.0)
    assert published == []
    updater.tick(31.0)

    assert [m for m, _, _ in published] == ["result-1"]
    assert any("result-1" in m for m in log.messages("error"))


def test_updater_ignores_a_plan_without_a_map() -> None:
    source = _FakeSource()
    published: List[Any] = []
    log = RecordingLog()
    updater = _updater(source, published, log=log)

    updater.want(None)
    updater.tick(0.0)
    updater.want(None)
    updater.tick(1.0)

    assert source.loaded == []
    assert len([m for m in log.messages("info") if "no reference map" in m]) == 1


def test_updater_does_not_start_a_second_load_while_one_runs() -> None:
    source = _FakeSource()
    started: List[Callable[[], None]] = []
    updater = MapUpdater(source, None, lambda *a: None, RecordingLog(), start=started.append)
    updater.want("result-1")

    updater.tick(0.0)
    updater.tick(1.0)

    assert len(started) == 1


def test_updater_logs_the_source_file_and_size() -> None:
    log = RecordingLog()
    updater = _updater(_FakeSource(), [], log=log)
    updater.want("result-1")

    updater.tick(0.0)

    assert any("mesh_984.pcd" in m and "3 points" in m for m in log.messages("info"))


# --- helpers --------------------------------------------------------------------------------


class _FakeFiles:
    def __init__(self) -> None:
        self.downloads: List[int] = []
        self.fail = False

    def retrieve(self, id: Optional[int] = None, **_: Any) -> Any:
        return SimpleNamespace(id=id, name="mesh_984.pcd")

    def download_to_path(self, path: str, id: Optional[int] = None, **_: Any) -> None:
        if self.fail:
            raise RuntimeError("download failed")
        self.downloads.append(int(id))
        with open(path, "wb") as handle:
            handle.write("content of {}".format(id).encode())


def _map_client() -> FakeClient:
    client = FakeClient()
    client.files = _FakeFiles()  # type: ignore[attr-defined]
    return client


def _add_campaign(client: FakeClient, xid: str, cdf: List[int], pcd: List[int], labels: List[str]) -> None:
    client.instances.add(
        RESULT_VIEW,
        FakeNode(xid, RESULT_VIEW, {"cdfFileIds": cdf, "pcdFileIds": pcd, "pcdFileLabels": labels}),
        listable=False,
    )


def _source(client: FakeClient, cache_dir: str) -> MapSource:
    return MapSource(client, cache_dir, log=RecordingLog())


class _FakeSource:
    def __init__(self, fail_times: int = 0) -> None:
        self.loaded: List[str] = []
        self._fail_times = fail_times

    def resolve(self, map_external_id: str, label: Optional[str]) -> MapFile:
        if self._fail_times:
            self._fail_times -= 1
            raise RuntimeError("CDF down")
        self.loaded.append(map_external_id)
        return MapFile(20, "mesh_984.pcd", "pcd", "Pointcloud")

    def fetch(self, map_file: MapFile) -> str:
        return "/cache/20/mesh_984.pcd"

    def load(self, map_file: MapFile, path: str) -> PointCloud:
        return PointCloud(fields=[], point_step=12, width=3, height=1, data=b"\0" * 36, is_dense=True)


def _updater(source: Any, published: List[Any], log: Any = None, retry_s: float = 30.0) -> MapUpdater:
    return MapUpdater(
        source,
        None,
        lambda map_id, map_file, cloud: published.append((map_id, map_file, cloud)),
        log or RecordingLog(),
        retry_s=retry_s,
        start=lambda work: work(),
    )


def _pcd_header(fields: str, size: str, type_: str, count: str, points: int, data: str) -> bytes:
    return (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {}\nSIZE {}\nTYPE {}\n"
        "COUNT {}\nWIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA {}\n".format(
            fields, size, type_, count, points, points, data
        )
    ).encode("ascii")


def _write(path: Any, content: bytes) -> str:
    with open(str(path), "wb") as handle:
        handle.write(content)
    return str(path)
