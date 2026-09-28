# SPDX-License-Identifier: BSD-3-Clause
"""The plan's reference map: find it in CDF, cache it locally, load it as a point cloud. ROS-free.

A plan's `mapExternalId` names an AutoAssess campaign. Its files are listed on the campaign
node: `pcdFileIds` / `pcdFileLabels` (point clouds) and `cdfFileIds` (PLY meshes), the same
files `dss plan download-map` downloads. The bridge picks one point cloud: the one whose label
matches `~map_file_label`, otherwise the first; if the campaign has no point cloud, it uses the
vertices of its first mesh. Reads only (instances.retrieve, files.retrieve, files download).
"""

from __future__ import annotations

import os
import re
import threading
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Sequence, Tuple

import numpy as np
from cognite.client.data_classes.data_modeling import ViewId

from autoassess_bridge.cdf import SPACE

INSPECTION_RESULT_VIEW = ("InspectionResultView", "1")
DEFAULT_CACHE_DIR = os.path.join("~", ".ros", "autoassess_maps")

# sensor_msgs/PointField datatypes
INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = range(1, 9)
_PCD_TYPES = {
    ("I", 1): (INT8, "i1"),
    ("U", 1): (UINT8, "u1"),
    ("I", 2): (INT16, "<i2"),
    ("U", 2): (UINT16, "<u2"),
    ("I", 4): (INT32, "<i4"),
    ("U", 4): (UINT32, "<u4"),
    ("F", 4): (FLOAT32, "<f4"),
    ("F", 8): (FLOAT64, "<f8"),
}
_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
    "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
}


@dataclass(frozen=True)
class Field:
    name: str
    offset: int
    datatype: int  # sensor_msgs/PointField datatype
    count: int = 1


@dataclass(frozen=True)
class PointCloud:
    """Everything a sensor_msgs/PointCloud2 needs (little endian)."""

    fields: List[Field]
    point_step: int
    width: int
    height: int
    data: bytes
    is_dense: bool

    @property
    def num_points(self) -> int:
        return self.width * self.height


@dataclass(frozen=True)
class MapFile:
    file_id: int
    name: str
    kind: str  # "pcd" or "ply"
    label: Optional[str]


# --- file formats ---------------------------------------------------------------------------


def load_pcd(path: str) -> PointCloud:
    """All fields of an ascii or binary PCD, as pcl_ros pcd_to_pointcloud would publish them."""
    with open(path, "rb") as handle:
        raw = handle.read()
    header, body = _split_pcd(raw, path)
    names = header["FIELDS"]
    sizes = [int(s) for s in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(c) for c in header.get("COUNT", ["1"] * len(names))]
    width = int(header["WIDTH"][0])
    height = int(header.get("HEIGHT", ["1"])[0])
    points = int(header.get("POINTS", [str(width * height)])[0])
    encoding = header["DATA"][0]

    fields: List[Field] = []
    dtype: List[Tuple[str, str, Tuple[int, ...]]] = []
    offset = 0
    for index, (name, size, kind, count) in enumerate(zip(names, sizes, types, counts)):
        if (kind, size) not in _PCD_TYPES:
            raise ValueError("{}: unsupported PCD field type {}{} for {}".format(path, kind, size, name))
        datatype, code = _PCD_TYPES[(kind, size)]
        dtype.append(("f{}".format(index), code, (count,) if count > 1 else ()))
        if name != "_":  # PCL padding: keep the bytes, not the field
            fields.append(Field(name, offset, datatype, count))
        offset += size * count
    point_step = offset
    structured = np.dtype([(n, c, s) if s else (n, c) for n, c, s in dtype])

    if encoding == "binary":
        expected = points * point_step
        if len(body) < expected:
            raise ValueError("{}: truncated PCD, {} of {} data bytes".format(path, len(body), expected))
        data = bytes(body[:expected])
        rows = np.frombuffer(data, dtype=structured)
    elif encoding == "ascii":
        values = np.fromstring(body.decode("ascii"), sep=" ") if body.strip() else np.zeros(0)
        per_point = sum(counts)
        if values.size != points * per_point:
            raise ValueError("{}: PCD has {} values, expected {}".format(path, values.size, points * per_point))
        values = values.reshape(points, per_point)
        rows = np.zeros(points, dtype=structured)
        column = 0
        for index, count in enumerate(counts):
            key = "f{}".format(index)
            if count == 1:
                rows[key] = values[:, column]
            else:
                rows[key] = values[:, column:column + count]
            column += count
        data = rows.tobytes()
    else:
        raise ValueError(
            "{}: PCD DATA {} is not supported; save the point cloud as ascii or binary "
            "(e.g. pcl_convert_pcd_ascii_binary <in> <out> 1)".format(path, encoding)
        )
    return PointCloud(fields, point_step, width, height, data, _is_dense(rows, fields, names))


def load_ply_vertices(path: str) -> PointCloud:
    """The vertices of an ascii or binary little-endian PLY as xyz (+ PCL-style packed rgb)."""
    with open(path, "rb") as handle:
        raw = handle.read()
    end = raw.find(b"end_header")
    if not raw.startswith(b"ply") or end < 0:
        raise ValueError("{}: not a PLY file".format(path))
    body_start = raw.index(b"\n", end) + 1
    fmt, elements = _ply_header(raw[:end].decode("ascii", "replace"))
    if not elements or elements[0][0] != "vertex":
        raise ValueError("{}: the first PLY element must be 'vertex'".format(path))
    _, count, props = elements[0]
    if any(code is None for _, code in props):
        raise ValueError("{}: list properties on PLY vertices are not supported".format(path))
    names = [name for name, _ in props]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError("{}: PLY vertices have no {} property".format(path, axis))

    if fmt == "binary_little_endian":
        vertex_dtype = np.dtype([(name, code) for name, code in props])
        rows = np.frombuffer(raw, dtype=vertex_dtype, count=count, offset=body_start)
        columns = {name: rows[name] for name in names}
    elif fmt == "ascii":
        body = np.frombuffer(raw, dtype=np.uint8, offset=body_start)
        newlines = np.flatnonzero(body == ord("\n"))
        stop = int(newlines[count - 1]) if count and newlines.size >= count else body.size
        values = np.fromstring(bytes(body[:stop]).decode("ascii"), sep=" ") if count else np.zeros(0)
        if values.size != count * len(names):
            raise ValueError("{}: PLY vertex rows have an unexpected width".format(path))
        values = values.reshape(count, len(names))
        columns = {name: values[:, i] for i, name in enumerate(names)}
    else:
        raise ValueError("{}: unsupported PLY format {}".format(path, fmt))

    has_rgb = all(c in columns for c in ("red", "green", "blue"))
    out_dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")] + ([("rgb", "<u4")] if has_rgb else [])
    out = np.zeros(count, dtype=out_dtype)
    for axis in ("x", "y", "z"):
        out[axis] = columns[axis]
    fields = [Field("x", 0, FLOAT32), Field("y", 4, FLOAT32), Field("z", 8, FLOAT32)]
    if has_rgb:
        r, g, b = (np.asarray(columns[c]).astype(np.uint32) & 0xFF for c in ("red", "green", "blue"))
        out["rgb"] = (r << 16) | (g << 8) | b
        fields.append(Field("rgb", 12, FLOAT32))  # PCL convention: packed uint32 read as float
    xyz = np.stack([out["x"], out["y"], out["z"]])
    return PointCloud(fields, out.dtype.itemsize, count, 1, out.tobytes(), bool(np.isfinite(xyz).all()))


def _split_pcd(raw: bytes, path: str) -> Tuple[dict, bytes]:
    header: dict = {}
    position = 0
    while position < len(raw):
        end = raw.find(b"\n", position)
        if end < 0:
            end = len(raw)
        line = raw[position:end].decode("ascii", "replace").strip()
        position = end + 1
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            break
    for key in ("FIELDS", "SIZE", "TYPE", "WIDTH", "DATA"):
        if key not in header:
            raise ValueError("{}: PCD header has no {} line".format(path, key))
    return header, raw[position:]


def _is_dense(rows: Any, fields: Sequence[Field], names: Sequence[str]) -> bool:
    for axis in ("x", "y", "z"):
        if axis in names:
            column = rows["f{}".format(list(names).index(axis))]
            if column.dtype.kind == "f" and not np.isfinite(column).all():
                return False
    return True


def _ply_header(text: str) -> Tuple[str, List[Tuple[str, int, List[Tuple[str, Optional[str]]]]]]:
    fmt = ""
    elements: List[Tuple[str, int, List[Tuple[str, Optional[str]]]]] = []
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            elements.append((parts[1], int(parts[2]), []))
        elif parts[0] == "property" and elements:
            if parts[1] == "list":
                elements[-1][2].append((parts[-1], None))
            else:
                elements[-1][2].append((parts[2], _PLY_TYPES.get(parts[1])))
    return fmt, elements


# --- choosing and fetching ------------------------------------------------------------------


def choose_map_file(
    cdf_file_ids: Sequence[int],
    pcd_file_ids: Sequence[int],
    pcd_file_labels: Sequence[str],
    label: Optional[str],
) -> Optional[Tuple[int, str, Optional[str], bool]]:
    """(file id, kind, label, label matched) of the file to publish, or None."""
    labels = list(pcd_file_labels) + [""] * max(0, len(pcd_file_ids) - len(pcd_file_labels))
    if pcd_file_ids:
        if label:
            for file_id, file_label in zip(pcd_file_ids, labels):
                if file_label.strip().lower() == label.strip().lower():
                    return int(file_id), "pcd", file_label, True
            return int(pcd_file_ids[0]), "pcd", labels[0] or None, False
        return int(pcd_file_ids[0]), "pcd", labels[0] or None, True
    if cdf_file_ids:
        return int(cdf_file_ids[0]), "ply", None, True
    return None


class MapSource:
    """Resolves, downloads (cached by file id) and loads a map campaign's point cloud.

    `log` needs info() and warning().
    """

    def __init__(self, client: Any, cache_dir: str, log: Any, space: str = SPACE) -> None:
        self._client = client
        self._cache_dir = os.path.expanduser(cache_dir)
        self._log = log
        self._space = space

    def resolve(self, map_external_id: str, label: Optional[str]) -> MapFile:
        view = ViewId(self._space, *INSPECTION_RESULT_VIEW)
        result = self._client.data_modeling.instances.retrieve(
            nodes=[(self._space, map_external_id)], sources=[view]
        )
        if not result.nodes:
            raise LookupError("Map campaign {} not found".format(map_external_id))
        props = result.nodes[0].properties.get(view) or {}
        chosen = choose_map_file(
            [i for i in props.get("cdfFileIds") or [] if i is not None],
            [i for i in props.get("pcdFileIds") or [] if i is not None],
            [str(s) for s in props.get("pcdFileLabels") or [] if s is not None],
            label,
        )
        if chosen is None:
            raise LookupError("Map campaign {} has no point cloud or mesh files".format(map_external_id))
        file_id, kind, file_label, matched = chosen
        if not matched:
            self._log.warning(
                "Map campaign {} has no point cloud labelled '{}'; using '{}'".format(
                    map_external_id, label, file_label
                )
            )
        metadata = self._client.files.retrieve(id=file_id)
        if metadata is None:
            raise LookupError("Map file {} of campaign {} not found".format(file_id, map_external_id))
        name = _safe_name(metadata.name or "{}.{}".format(file_id, kind))
        return MapFile(file_id, name, kind, file_label)

    def fetch(self, map_file: MapFile) -> str:
        """Local path of the file, downloading it unless it is already in the cache."""
        folder = os.path.join(self._cache_dir, str(map_file.file_id))
        path = os.path.join(folder, map_file.name)
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return path
        os.makedirs(folder, exist_ok=True)
        partial = path + ".part"
        self._log.info("Downloading map file {} (file id {})".format(map_file.name, map_file.file_id))
        try:
            # download_to_path fetches the (short-lived) download URL right before downloading.
            self._client.files.download_to_path(partial, id=map_file.file_id)
            os.replace(partial, path)
        finally:
            if os.path.exists(partial):
                os.remove(partial)
        return path

    def load(self, map_file: MapFile, path: str) -> PointCloud:
        return load_pcd(path) if map_file.kind == "pcd" else load_ply_vertices(path)


def _safe_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]", "_", os.path.basename(name)) or "map"


class MapUpdater:
    """Publishes the reference map of the current plan once per map id, loading in the background.

    want(map id) is called for each published plan; tick(now) starts a load when the wanted map
    is not published yet, no load is running, and the last failure is `retry_s` old.
    `publish(map_id, map_file, cloud)`; `start(work)` runs work (default: a daemon thread).
    """

    def __init__(
        self,
        source: Any,
        label: Optional[str],
        publish: Callable[[str, MapFile, PointCloud], None],
        log: Any,
        retry_s: float = 30.0,
        start: Optional[Callable[[Callable[[], None]], None]] = None,
    ) -> None:
        self._source = source
        self._label = label
        self._publish = publish
        self._log = log
        self._retry_s = retry_s
        self._start = start or _start_thread
        self._lock = threading.Lock()
        self._wanted: Optional[str] = None
        self._published: Optional[str] = None
        self._running = False
        self._failed_at: Optional[float] = None
        self._reported_no_map = False

    def want(self, map_external_id: Optional[str]) -> None:
        with self._lock:
            if map_external_id is None:
                if not self._reported_no_map:
                    self._log.info("The plan has no reference map (mapExternalId); no map published")
                    self._reported_no_map = True
                return
            self._reported_no_map = False
            if map_external_id != self._wanted:
                self._wanted = map_external_id
                self._failed_at = None

    def tick(self, now: float) -> None:
        with self._lock:
            wanted = self._wanted
            if wanted is None or wanted == self._published or self._running:
                return
            if self._failed_at is not None and now - self._failed_at < self._retry_s:
                return
            self._running = True
        self._start(lambda: self._load(wanted, now))

    def _load(self, map_external_id: str, now: float) -> None:
        try:
            self._log.info("Loading reference map {}".format(map_external_id))
            map_file = self._source.resolve(map_external_id, self._label)
            path = self._source.fetch(map_file)
            cloud = self._source.load(map_file, path)
            self._publish(map_external_id, map_file, cloud)
            self._log.info(
                "Published map {}: {} ({}, file id {}), {} points".format(
                    map_external_id, map_file.name, map_file.label or map_file.kind,
                    map_file.file_id, cloud.num_points,
                )
            )
            with self._lock:
                self._published = map_external_id
                self._failed_at = None
        except Exception as exc:  # noqa: BLE001 - report and retry later
            self._log.error(
                "Loading map {} failed (will retry): {}: {}".format(map_external_id, type(exc).__name__, exc)
            )
            with self._lock:
                self._failed_at = now
        finally:
            with self._lock:
                self._running = False


def _start_thread(work: Callable[[], None]) -> None:
    threading.Thread(target=work, daemon=True).start()
