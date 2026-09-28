# SPDX-License-Identifier: BSD-3-Clause
"""Findings from the robot's detection stack → region tasks for a Draft plan. ROS-free.

The node subscribes to /autoassess/findings (std_msgs/String). Each message is one JSON
object or an array of objects with the same fields as the AutoAssess findings CSV
(`dss plan import-findings`): required ``x, y, z`` (metres, in the reference map's frame),
optional ``id`` (default: a hash of the values), ``nx, ny, nz`` (surface normal, all three or
none), ``radius`` (metres, default 0.3), ``inspection_type`` (``visual`` | ``ndt_thickness``),
``class``, ``confidence`` (0..1) and ``description``. Bad entries are reported, not raised.

Findings are buffered (deduped by id) during the mission. At mission end they are merged
within a radius and become region tasks with ``suggestionId = "finding:<id>[+<id>…]"``, the
same convention as the AutoAssess SDK, which makes retries idempotent. Normals: the finding's
own, else facing the findings' centroid (nearest-mesh-normal estimation is a follow-up).
Writing the plan to CDF is upload.py's job; this module is pure.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

DEFAULT_RADIUS_M = 0.3
DEFAULT_MERGE_RADIUS_M = 0.5
DEFAULT_BUFFER_CAP = 5000
SUGGESTION_PREFIX = "finding:"
MAX_SUGGESTION_ID_LENGTH = 255
MAX_ID_LENGTH = 200
_UP = (0.0, 0.0, 1.0)

_FIELDS = (
    "id", "x", "y", "z", "nx", "ny", "nz",
    "radius", "inspection_type", "class", "confidence", "description",
)
_INSPECTION_TYPES = ("visual", "ndt_thickness")


@dataclass(frozen=True)
class Finding:
    """One reported finding. ``None`` means "not given" (defaults apply when merging)."""

    id: str
    position: Vec3
    normal: Optional[Vec3] = None
    radius_m: Optional[float] = None
    inspection_type: Optional[str] = None  # visual | ndt_thickness
    finding_class: Optional[str] = None
    confidence: Optional[float] = None
    description: Optional[str] = None


@dataclass(frozen=True)
class FindingCluster:
    """One or more nearby findings that become a single region task."""

    member_ids: Tuple[str, ...]
    position: Vec3  # mean of the members
    normal: Optional[Vec3]  # mean of the members' known normals, if any
    radius_m: float  # covers every member's own radius
    inspection_type: str  # ndt_thickness if any member needs it
    confidence: Optional[float] = None  # highest member confidence


@dataclass(frozen=True)
class PlannedTask:
    """A region task to create, with the same fields as the AutoAssess SDK's NewRegionTask."""

    position3d: Vec3
    normal_vector: Vec3
    radius_m: float
    inspection_type: str
    suggestion_id: str
    normal_source: str  # "given" | "centroid"


@dataclass(frozen=True)
class TaskPlan:
    tasks: List[PlannedTask]
    already_in_plan: int = 0  # findings skipped: a task already covers them
    interior_point: Optional[Vec3] = None  # what fallback normals were oriented towards


class _InvalidEntry(ValueError):
    pass


# --- parsing --------------------------------------------------------------------------------


def parse_findings_json(text: str) -> Tuple[List[Finding], List[str]]:
    """Parse one /autoassess/findings message. Bad entries become error strings, not raises."""
    try:
        payload = json.loads(text)
    except ValueError as exc:
        return [], ["not JSON: {}".format(exc)]
    if isinstance(payload, dict):
        entries: List[Any] = [payload]
    elif isinstance(payload, list):
        entries = payload
    else:
        return [], ["expected a JSON object or array, got {}".format(type(payload).__name__)]

    findings: List[Finding] = []
    errors: List[str] = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            errors.append("entry {}: not an object".format(index))
            continue
        try:
            findings.append(_parse_entry(entry))
        except _InvalidEntry as exc:
            errors.append("entry {}: {}".format(index, exc))
    return findings, errors


def _parse_entry(entry: Dict[str, Any]) -> Finding:
    row = {}
    for key in _FIELDS:
        value = entry.get(key)
        if value is not None and value != "":
            row[key] = value
    position = (_number(row, "x"), _number(row, "y"), _number(row, "z"))
    return Finding(
        id=_entry_id(row),
        position=position,
        normal=_normal(row),
        radius_m=_radius(row),
        inspection_type=_inspection_type(row),
        finding_class=_text(row, "class"),
        confidence=_confidence(row),
        description=_text(row, "description"),
    )


def _entry_id(row: Dict[str, Any]) -> str:
    finding_id = str(row.get("id", "") or "")
    if not finding_id:
        canonical = "\x1f".join(
            "{}={}".format(k, row[k]) for k in sorted(row) if k != "id"
        )
        return "row-" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:12]
    if "+" in finding_id:
        raise _InvalidEntry("id {!r} must not contain '+'".format(finding_id))
    if len(finding_id) > MAX_ID_LENGTH:
        raise _InvalidEntry("id is longer than {} characters".format(MAX_ID_LENGTH))
    return finding_id


def _number(row: Dict[str, Any], key: str) -> float:
    if key not in row:
        raise _InvalidEntry("{} is missing".format(key))
    raw = row[key]
    if isinstance(raw, bool) or not isinstance(raw, (int, float, str)):
        raise _InvalidEntry("{} {!r} is not a number".format(key, raw))
    try:
        value = float(raw)
    except ValueError:
        raise _InvalidEntry("{} {!r} is not a number".format(key, raw))
    if not math.isfinite(value):
        raise _InvalidEntry("{} {!r} is not a finite number".format(key, raw))
    return value


def _normal(row: Dict[str, Any]) -> Optional[Vec3]:
    given = [key in row for key in ("nx", "ny", "nz")]
    if not any(given):
        return None
    if not all(given):
        raise _InvalidEntry("nx, ny and nz must all be set or all be missing")
    normal = _unit((_number(row, "nx"), _number(row, "ny"), _number(row, "nz")))
    if normal is None:
        raise _InvalidEntry("normal (nx, ny, nz) is zero-length")
    return normal


def _radius(row: Dict[str, Any]) -> Optional[float]:
    if "radius" not in row:
        return None
    radius = _number(row, "radius")
    if radius <= 0:
        raise _InvalidEntry("radius must be a positive number of metres, got {}".format(radius))
    return radius


def _inspection_type(row: Dict[str, Any]) -> Optional[str]:
    raw = str(row.get("inspection_type", "") or "").lower()
    if not raw:
        return None
    if raw not in _INSPECTION_TYPES:
        raise _InvalidEntry("inspection_type must be visual or ndt_thickness, got {!r}".format(raw))
    return raw


def _confidence(row: Dict[str, Any]) -> Optional[float]:
    if "confidence" not in row:
        return None
    confidence = _number(row, "confidence")
    if not 0 <= confidence <= 1:
        raise _InvalidEntry("confidence must be between 0 and 1, got {}".format(confidence))
    return confidence


def _text(row: Dict[str, Any], key: str) -> Optional[str]:
    value = row.get(key)
    return str(value) if value else None


# --- buffer ---------------------------------------------------------------------------------


class FindingsBuffer:
    """Thread-safe mission buffer, deduped by finding id (the first report wins), capped."""

    def __init__(self, cap: int = DEFAULT_BUFFER_CAP) -> None:
        self._cap = cap
        self._lock = threading.Lock()
        self._by_id: Dict[str, Finding] = {}
        self.dropped = 0  # findings dropped because the buffer was full

    def __len__(self) -> int:
        with self._lock:
            return len(self._by_id)

    def add(self, finding: Finding) -> bool:
        """True when the finding was new and stored."""
        with self._lock:
            if finding.id in self._by_id:
                return False
            if len(self._by_id) >= self._cap:
                self.dropped += 1
                return False
            self._by_id[finding.id] = finding
            return True

    def snapshot(self) -> List[Finding]:
        with self._lock:
            return list(self._by_id.values())

    def discard(self, ids: Iterable[str]) -> None:
        with self._lock:
            for finding_id in ids:
                self._by_id.pop(finding_id, None)


# --- merging --------------------------------------------------------------------------------


class _ClusterBuilder:
    def __init__(self) -> None:
        self.members: List[Finding] = []
        self.position_sum = [0.0, 0.0, 0.0]
        self.normal_sum = [0.0, 0.0, 0.0]

    def add(self, finding: Finding) -> None:
        self.members.append(finding)
        for i in range(3):
            self.position_sum[i] += finding.position[i]
            if finding.normal is not None:
                self.normal_sum[i] += finding.normal[i]

    @property
    def centre(self) -> Vec3:
        n = len(self.members)
        return (self.position_sum[0] / n, self.position_sum[1] / n, self.position_sum[2] / n)

    def accepts_normal(self, normal: Optional[Vec3]) -> bool:
        # Don't merge findings that face away from each other (two sides of a thin plate).
        if normal is None:
            return True
        s = self.normal_sum
        return s[0] * normal[0] + s[1] * normal[1] + s[2] * normal[2] >= 0


def merge_findings(
    findings: Sequence[Finding],
    merge_radius_m: float = DEFAULT_MERGE_RADIUS_M,
    default_radius_m: float = DEFAULT_RADIUS_M,
) -> List[FindingCluster]:
    """Greedy clustering: each finding joins the nearest cluster centre within
    `merge_radius_m` (unless their normals face away), else starts a new one.
    `merge_radius_m <= 0` disables merging.
    """
    builders: List[_ClusterBuilder] = []
    for finding in findings:
        best: Optional[_ClusterBuilder] = None
        best_distance = math.inf
        if merge_radius_m > 0:
            for builder in builders:
                distance = math.dist(builder.centre, finding.position)
                if distance <= merge_radius_m and distance < best_distance:
                    if builder.accepts_normal(finding.normal):
                        best, best_distance = builder, distance
        if best is None:
            best = _ClusterBuilder()
            builders.append(best)
        best.add(finding)
    return [_finish(builder, default_radius_m) for builder in builders]


def _finish(builder: _ClusterBuilder, default_radius_m: float) -> FindingCluster:
    centre = builder.centre
    radius = max(
        math.dist(f.position, centre) + (f.radius_m if f.radius_m is not None else default_radius_m)
        for f in builder.members
    )
    needs_ndt = any((f.inspection_type or "visual") == "ndt_thickness" for f in builder.members)
    confidences = [f.confidence for f in builder.members if f.confidence is not None]
    return FindingCluster(
        member_ids=tuple(f.id for f in builder.members),
        position=centre,
        normal=_unit((builder.normal_sum[0], builder.normal_sum[1], builder.normal_sum[2])),
        radius_m=radius,
        inspection_type="ndt_thickness" if needs_ndt else "visual",
        confidence=max(confidences) if confidences else None,
    )


# --- suggestion ids -------------------------------------------------------------------------


def finding_suggestion_id(cluster: FindingCluster) -> str:
    """``finding:<id>`` / ``finding:<id1>+<id2>…`` (sorted), capped at 255 characters.

    When capped, as many ids as fit are kept, followed by ``+~<hash>`` of the full id.
    """
    ids = sorted(cluster.member_ids)
    full = SUGGESTION_PREFIX + "+".join(ids)
    if len(full) <= MAX_SUGGESTION_ID_LENGTH:
        return full
    digest = "~" + hashlib.sha256(full.encode("utf-8")).hexdigest()[:12]
    kept: List[str] = []
    length = len(SUGGESTION_PREFIX) + len(digest)
    for finding_id in ids:
        if length + len(finding_id) + 1 > MAX_SUGGESTION_ID_LENGTH:
            break
        kept.append(finding_id)
        length += len(finding_id) + 1
    return SUGGESTION_PREFIX + "+".join(kept + [digest])


def covered_finding_ids(suggestion_ids: Iterable[Optional[str]]) -> set:
    """Finding ids named by existing tasks' ``finding:`` suggestion ids."""
    covered = set()
    for sid in suggestion_ids:
        if not sid or not sid.startswith(SUGGESTION_PREFIX):
            continue
        for token in sid[len(SUGGESTION_PREFIX):].split("+"):
            if token and not token.startswith("~"):
                covered.add(token)
    return covered


# --- planning tasks -------------------------------------------------------------------------


def plan_tasks_from_findings(
    findings: Sequence[Finding],
    merge_radius_m: float = DEFAULT_MERGE_RADIUS_M,
    default_radius_m: float = DEFAULT_RADIUS_M,
    existing_suggestion_ids: Iterable[Optional[str]] = (),
    interior_point: Optional[Vec3] = None,
) -> TaskPlan:
    """Turn buffered findings into region tasks (nothing is written).

    Findings covered by `existing_suggestion_ids` (the plan's current tasks) are skipped, so a
    retry adds nothing twice. Normals: the finding's own, else the direction towards
    `interior_point` (default: the findings' bounding-box centre).
    """
    existing = {sid for sid in existing_suggestion_ids if sid}
    covered = covered_finding_ids(existing)
    fresh = [f for f in findings if f.id not in covered]
    skipped = len(findings) - len(fresh)

    clusters = merge_findings(fresh, merge_radius_m, default_radius_m)
    kept: List[FindingCluster] = []
    for cluster in clusters:
        if finding_suggestion_id(cluster) in existing:
            skipped += len(cluster.member_ids)
        else:
            kept.append(cluster)
    if not kept:
        return TaskPlan(tasks=[], already_in_plan=skipped)

    interior = interior_point or _bbox_centre([f.position for f in findings])
    tasks = [_planned_task(cluster, interior) for cluster in kept]
    return TaskPlan(tasks=tasks, already_in_plan=skipped, interior_point=interior)


def _planned_task(cluster: FindingCluster, interior: Vec3) -> PlannedTask:
    normal = cluster.normal
    source = "given"
    if normal is None:
        normal = _unit(
            (
                interior[0] - cluster.position[0],
                interior[1] - cluster.position[1],
                interior[2] - cluster.position[2],
            )
        ) or _UP
        source = "centroid"
    return PlannedTask(
        position3d=cluster.position,
        normal_vector=normal,
        radius_m=cluster.radius_m,
        inspection_type=cluster.inspection_type,
        suggestion_id=finding_suggestion_id(cluster),
        normal_source=source,
    )


def _bbox_centre(points: Sequence[Vec3]) -> Vec3:
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)


def _unit(v: Tuple[float, float, float]) -> Optional[Vec3]:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length < 1e-9:
        return None
    return (v[0] / length, v[1] / length, v[2] / length)
