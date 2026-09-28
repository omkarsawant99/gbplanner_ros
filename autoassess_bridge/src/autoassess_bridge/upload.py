# SPDX-License-Identifier: BSD-3-Clause
"""Mission writes to CDF: CogniteFiles, the campaign, and the Draft findings plan. ROS-free.

This is the only module that writes to CDF (cdf.py and map.py stay read-only), and the node
only writes when ~upload_enabled is true. The upload follows the AutoAssess ground-station SDK
(`dss campaign upload`):

- every file is a CogniteFile node (cdf_cdm:CogniteFile/v1) in the AutoAssess space, with
  mimeType application/octet-stream and the tags autoassess, ply_mesh or pcd_pointcloud,
  area:<id> (and label:<label> for point clouds); the content is uploaded by instance id.
  The bridge adds plan:<id> and mission:<id> tags. The external id has the SDK's shape,
  {area}-file-{12 hex}-{name}, but the hex part is derived from the mission and the file path
  so that retrying a mission reuses the files already uploaded;
- the campaign is an InspectionResultView node result-<uuid4> with the area, today's date,
  status InProgress and createdBy autoassess_bridge; its file ids are set once the files are
  uploaded, then the status becomes Complete. Nothing existing is changed or deleted.

At mission end the buffered /autoassess/findings are stored as DefectDetection nodes on the
just-created campaign (`DefectService`), sequenced by `MissionFlow`: upload first, then the
defects, then one final status carrying both. So the node's writes are: CogniteFiles, the
campaign, and defects.
"""

from __future__ import annotations

import hashlib
import os
import re
import threading
import time
import uuid
import warnings
from dataclasses import dataclass, field
from datetime import date, datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from cognite.client.data_classes.data_modeling import NodeId, ViewId
from cognite.client.data_classes.data_modeling.instances import NodeApply, NodeOrEdgeData

from autoassess_bridge.cdf import SPACE
from autoassess_bridge.findings import DEFAULT_MERGE_RADIUS_M, Finding, merge_findings

CREATED_BY = "autoassess_bridge"
MIME_TYPE = "application/octet-stream"
INSPECTION_RESULT_VIEW = ("InspectionResultView", "1")
DEFECT_DETECTION_VIEW = ("DefectDetectionView", "1")
DEFAULT_DEFECT_PROBABILITY = 0.5  # DefectDetectionContainer.probability is non-nullable
_DEFECT_CHUNK = 1000
_MAX_EXTERNAL_ID = 255
_UNSAFE = re.compile(r"[^A-Za-z0-9._-]")
_KINDS = {".ply": "ply", ".pcd": "pcd"}


@dataclass(frozen=True)
class MissionContext:
    area_external_id: str
    mission_id: str
    plan_external_id: Optional[str] = None  # the flown plan
    map_external_id: Optional[str] = None  # the flown plan's reference map
    plan_name: Optional[str] = None


@dataclass(frozen=True)
class MissionFile:
    path: str
    kind: str  # "ply" (mesh) or "pcd" (point cloud)
    label: Optional[str] = None  # point clouds only


@dataclass
class UploadResult:
    campaign_external_id: str
    cdf_file_ids: List[int]
    pcd_file_ids: List[int]
    pcd_file_labels: List[str]
    failed: List[Tuple[str, str]] = field(default_factory=list)  # (path, error)

    @property
    def complete(self) -> bool:
        return not self.failed


def mission_id_for(started_at: float) -> str:
    """mission-<UTC start time>, e.g. mission-20260304T050607Z."""
    stamp = datetime.fromtimestamp(started_at, tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return "mission-" + stamp


def file_external_id(area_external_id: str, mission_id: str, path: str) -> str:
    """{area}-file-{12 hex}-{sanitised name}, stable for the same mission and path."""
    digest = hashlib.sha1("{}\n{}".format(mission_id, os.path.abspath(path)).encode("utf-8"))
    head = "{}-file-{}-".format(area_external_id, digest.hexdigest()[:12])
    name = _UNSAFE.sub("_", os.path.basename(path))
    room = _MAX_EXTERNAL_ID - len(head)
    if room < 1:
        return head[:_MAX_EXTERNAL_ID]
    return head + name[-room:]


def file_tags(ctx: MissionContext, mission_file: MissionFile) -> List[str]:
    area = "area:" + ctx.area_external_id
    if mission_file.kind == "ply":
        tags = ["autoassess", "ply_mesh", area]
    else:
        tags = ["autoassess", "pcd_pointcloud", area, "label:{}".format(mission_file.label)]
    if ctx.plan_external_id:
        tags.append("plan:" + ctx.plan_external_id)
    tags.append("mission:" + ctx.mission_id)
    return tags


def pcd_label(path: str) -> str:
    """Default point-cloud label, as in the SDK: the file stem, underscores to spaces, title case."""
    stem = os.path.splitext(os.path.basename(path))[0]
    return stem.replace("_", " ").title()


def collect_files(
    mesh_path: Optional[str], mission_dir: Optional[str]
) -> Tuple[List[MissionFile], List[str]]:
    """The mesh first, then every .ply/.pcd under `mission_dir` (sorted); other files are skipped."""
    files: List[MissionFile] = []
    skipped: List[str] = []
    seen = set()
    if mesh_path:
        files.append(MissionFile(mesh_path, "ply"))
        seen.add(os.path.realpath(mesh_path))
    if mission_dir and os.path.isdir(mission_dir):
        found = []
        for root, _dirs, names in os.walk(mission_dir):
            for name in names:
                found.append(os.path.join(root, name))
        for path in sorted(found):
            if os.path.realpath(path) in seen:
                continue
            kind = _KINDS.get(os.path.splitext(path)[1].lower())
            if kind is None:
                skipped.append(path)
            elif kind == "pcd":
                files.append(MissionFile(path, kind, pcd_label(path)))
            else:
                files.append(MissionFile(path, kind))
    return files, skipped


class MissionUploader:
    """Uploads a mission's files and records them in a campaign (see the module docstring).

    Retrying the same mission id reuses its campaign and skips files already uploaded.
    `log` needs info() and warning().
    """

    def __init__(
        self,
        client: Any,
        log: Any,
        space: str = SPACE,
        today: Callable[[], date] = date.today,
        new_uuid: Callable[[], uuid.UUID] = uuid.uuid4,
    ) -> None:
        self._client = client
        self._log = log
        self._space = space
        self._today = today
        self._new_uuid = new_uuid
        self._campaigns: Dict[str, str] = {}
        # cognite-sdk 7.x flags files-by-instance-id as alpha on every call (FeaturePreviewWarning).
        warnings.filterwarnings("ignore", message=".*Files with Instance ID.*")

    def upload(self, ctx: MissionContext, files: Sequence[MissionFile]) -> UploadResult:
        if not files:
            raise ValueError("Nothing to upload")
        campaign = self._campaigns.get(ctx.mission_id)
        if campaign is None:
            campaign = self._create_campaign(ctx.area_external_id)
            self._campaigns[ctx.mission_id] = campaign
        result = UploadResult(campaign, [], [], [])
        for mission_file in files:
            try:
                file_id = self._upload_file(ctx, mission_file)
            except Exception as exc:  # noqa: BLE001 - report per file, keep going
                error = "{}: {}".format(type(exc).__name__, exc)
                self._log.warning("Upload of {} failed: {}".format(mission_file.path, error))
                result.failed.append((mission_file.path, error))
                continue
            if mission_file.kind == "ply":
                result.cdf_file_ids.append(file_id)
            else:
                result.pcd_file_ids.append(file_id)
                result.pcd_file_labels.append(str(mission_file.label))
        self._apply_campaign(
            campaign,
            {
                "cdfFileIds": result.cdf_file_ids,
                "pcdFileIds": result.pcd_file_ids,
                "pcdFileLabels": result.pcd_file_labels,
            },
        )
        if result.complete:
            self._apply_campaign(campaign, {"status": "Complete"})
            self._log.info("Campaign {} complete".format(campaign))
        return result

    def _create_campaign(self, area_external_id: str) -> str:
        external_id = "result-{}".format(self._new_uuid())
        self._apply_campaign(
            external_id,
            {
                "area": {"space": self._space, "externalId": area_external_id},
                "campaignDate": self._today().isoformat(),
                "status": "InProgress",
                "cdfFileIds": [],
                "pcdFileIds": [],
                "pcdFileLabels": [],
                "createdBy": CREATED_BY,
            },
        )
        self._log.info("Created campaign {} for area {}".format(external_id, area_external_id))
        return external_id

    def _apply_campaign(self, external_id: str, properties: Dict[str, Any]) -> None:
        self._client.data_modeling.instances.apply(
            nodes=[
                NodeApply(
                    space=self._space,
                    external_id=external_id,
                    sources=[
                        NodeOrEdgeData(
                            source=ViewId(self._space, *INSPECTION_RESULT_VIEW),
                            properties=properties,
                        )
                    ],
                )
            ]
        )

    def _upload_file(self, ctx: MissionContext, mission_file: MissionFile) -> int:
        external_id = file_external_id(ctx.area_external_id, ctx.mission_id, mission_file.path)
        node_id = NodeId(self._space, external_id)
        existing = self._client.files.retrieve(instance_id=node_id)
        if existing is not None and getattr(existing, "uploaded", False):
            self._log.info("Already uploaded {} (file id {})".format(mission_file.path, existing.id))
            return int(existing.id)
        if existing is None:
            self._client.data_modeling.instances.apply(
                nodes=[_cognite_file_apply(self._space, external_id, ctx, mission_file)]
            )
        metadata = self._client.files.upload_content(mission_file.path, instance_id=node_id)
        file_id = getattr(metadata, "id", None)
        if file_id is None:
            raise RuntimeError("CogniteFile upload of {} returned no file id".format(mission_file.path))
        self._log.info("Uploaded {} (file id {})".format(mission_file.path, file_id))
        return int(file_id)


def _cognite_file_apply(space: str, external_id: str, ctx: MissionContext, mission_file: MissionFile) -> Any:
    with warnings.catch_warnings():
        # cognite-sdk 7.x flags the Core Data Model classes as alpha (FeaturePreviewWarning).
        warnings.simplefilter("ignore")
        from cognite.client.data_classes.data_modeling.cdm.v1 import CogniteFileApply

    return CogniteFileApply(
        space=space,
        external_id=external_id,
        name=os.path.basename(mission_file.path),
        mime_type=MIME_TYPE,
        tags=file_tags(ctx, mission_file),
    )


def export_mesh(
    generate: Callable[[], None],
    mesh_path: str,
    timeout_s: float,
    clock: Callable[[], float] = time.time,
    sleep: Callable[[float], None] = time.sleep,
    poll_s: float = 0.2,
) -> str:
    """Ask the mapper to write its mesh (`generate`) and wait until `mesh_path` was rewritten."""
    before = _stamp(mesh_path)
    generate()
    deadline = clock() + timeout_s
    while True:
        now_stamp = _stamp(mesh_path)
        if now_stamp is not None and now_stamp != before and now_stamp[1] > 0:
            return mesh_path
        if clock() >= deadline:
            raise TimeoutError(
                "Mesh file {} was not written within {:.0f} s; is gbplanner_node's "
                "mesh_filename parameter set to this path?".format(mesh_path, timeout_s)
            )
        sleep(poll_s)


def _stamp(path: str) -> Optional[Tuple[int, int]]:
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return (stat.st_mtime_ns, stat.st_size)


def status_message(
    state: str,
    ctx: Optional[MissionContext],
    result: Optional[UploadResult] = None,
    message: str = "",
    skipped: Sequence[str] = (),
    findings: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Content of /autoassess/upload_status (published as JSON).

    `findings` is filled on the final status of a mission: {"count", "defectExternalIds"} for
    the DefectDetection nodes stored on the campaign, {"error": …} when storing them failed,
    null without findings.
    """
    return {
        "findings": findings,
        "state": state,  # idle | exporting_mesh | uploading | complete | failed
        "missionId": ctx.mission_id if ctx else None,
        "areaExternalId": ctx.area_external_id if ctx else None,
        "planExternalId": ctx.plan_external_id if ctx else None,
        "campaignExternalId": result.campaign_external_id if result else None,
        "cdfFileIds": list(result.cdf_file_ids) if result else [],
        "pcdFileIds": list(result.pcd_file_ids) if result else [],
        "pcdFileLabels": list(result.pcd_file_labels) if result else [],
        "failedFiles": [{"path": p, "error": e} for p, e in result.failed] if result else [],
        "skippedFiles": list(skipped),
        "message": message,
        "updatedAt": datetime.now(timezone.utc).isoformat(),
    }


@dataclass(frozen=True)
class FollowedPlan:
    plan_external_id: str
    area_external_id: str
    map_external_id: Optional[str] = None
    name: Optional[str] = None


class FollowedPlans:
    """Which plan (and so which area and map) the bridge followed when; thread-safe.

    at(t) is the plan followed at time t: the last one recorded at or before t, or the first one
    if the bridge only picked up a plan after t (it was started mid-mission).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._records: List[Tuple[float, FollowedPlan]] = []

    def __len__(self) -> int:
        with self._lock:
            return len(self._records)

    def record(
        self,
        now: float,
        plan_external_id: str,
        area_external_id: str,
        map_external_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
        followed = FollowedPlan(plan_external_id, area_external_id, map_external_id, name)
        with self._lock:
            if self._records and self._records[-1][1] == followed:
                return
            self._records.append((now, followed))

    def at(self, when: float) -> Optional[FollowedPlan]:
        with self._lock:
            if not self._records:
                return None
            chosen = self._records[0][1]
            for at_time, followed in self._records:
                if at_time <= when:
                    chosen = followed
            return chosen

    def latest(self) -> Optional[FollowedPlan]:
        with self._lock:
            return self._records[-1][1] if self._records else None


NO_PLAN_MESSAGE = (
    "No plan has been followed yet, so the mission has no area to upload to; "
    "mark a plan Ready in AutoAssess and let the bridge pick it up first"
)


class MissionUploadRunner:
    """One upload at a time: export the mesh (optional), collect files, upload, report status.

    `export_mesh() -> path` (or None to skip the mesh), `publish_status(dict)`; `log` needs
    info(), warning() and error(). run() returns (success, message) for the Trigger service.
    """

    def __init__(
        self,
        uploader: Any,
        export_mesh: Optional[Callable[[], str]],
        mission_dir: Optional[str],
        publish_status: Callable[[Dict[str, Any]], None],
        log: Any,
    ) -> None:
        self._uploader = uploader
        self._export_mesh = export_mesh
        self._mission_dir = mission_dir
        self._publish = publish_status
        self._log = log
        self._lock = threading.Lock()

    def run(self, ctx: Optional[MissionContext]) -> Tuple[bool, str]:
        """Upload; `ctx` None means no plan (so no area) is known, which fails without writes."""
        if not self._lock.acquire(False):
            return False, "An upload is already running"
        try:
            if ctx is None:
                return self._fail(None, NO_PLAN_MESSAGE)
            return self._run(ctx)
        finally:
            self._lock.release()

    def _run(self, ctx: MissionContext) -> Tuple[bool, str]:
        self._log.info("Uploading {} for area {}".format(ctx.mission_id, ctx.area_external_id))
        mesh: Optional[str] = None
        if self._export_mesh is not None:
            self._publish(status_message("exporting_mesh", ctx))
            try:
                mesh = self._export_mesh()
            except Exception as exc:  # noqa: BLE001 - report and stop
                return self._fail(ctx, "Mesh export failed: {}: {}".format(type(exc).__name__, exc))
        files, skipped = collect_files(mesh, self._mission_dir)
        if not files:
            return self._fail(ctx, "Nothing to upload (no mesh and no .ply/.pcd in the mission dir)", skipped)
        self._publish(status_message("uploading", ctx, skipped=skipped))
        try:
            result = self._uploader.upload(ctx, files)
        except Exception as exc:  # noqa: BLE001 - report and stop
            return self._fail(ctx, "Upload failed: {}: {}".format(type(exc).__name__, exc), skipped)
        if result.complete:
            message = "Uploaded {} file(s) to campaign {}".format(len(files), result.campaign_external_id)
            self._log.info(message)
            self._publish(status_message("complete", ctx, result, message, skipped))
            return True, message
        message = "{} of {} file(s) failed; campaign {} stays InProgress (retry to resume)".format(
            len(result.failed), len(files), result.campaign_external_id
        )
        self._log.error(message)
        self._publish(status_message("failed", ctx, result, message, skipped))
        return False, message

    def _fail(self, ctx: Optional[MissionContext], message: str, skipped: Sequence[str] = ()) -> Tuple[bool, str]:
        self._log.error(message)
        self._publish(status_message("failed", ctx, message=message, skipped=skipped))
        return False, message


# --- findings plan --------------------------------------------------------------------------


@dataclass(frozen=True)
class DefectResult:
    defect_external_ids: List[str]  # every defect of this mission's findings (new and existing)
    created: int
    already_existing: int


class DefectService:
    """Stores findings as DefectDetection nodes (autoassess/DefectDetectionView/1) in CDF.

    One node per merged cluster of findings, attached to the mission's campaign, with
    `status: New` and `source: ml` so they appear in the viewer's Defects tab for review
    (Confirm -> Suggestions -> task). The external id `defect-<missionId>-<sha1(ids)[:12]>` is
    deterministic, so a retry retrieves the ids and only creates what is missing — no
    registry. `log` needs info() and warning().
    """

    def __init__(
        self,
        client: Any,
        log: Any,
        space: str = SPACE,
        merge_radius_m: float = DEFAULT_MERGE_RADIUS_M,
    ) -> None:
        self._client = client
        self._log = log
        self._space = space
        self._merge_radius_m = merge_radius_m

    def create(
        self,
        ctx: Optional[MissionContext],
        campaign_external_id: Optional[str],
        findings: Sequence[Finding],
    ) -> DefectResult:
        if ctx is None:
            raise ValueError("No mission context")
        if not findings:
            raise ValueError("No findings to store")
        clusters = merge_findings(findings, self._merge_radius_m)
        external_ids = [self._external_id(ctx.mission_id, c.member_ids) for c in clusters]
        existing = self._existing(external_ids)
        if campaign_external_id is None:
            self._log.warning(
                "Mission {} has no campaign (upload failed?); defects are attached to area {} "
                "instead".format(ctx.mission_id, ctx.area_external_id)
            )
        nodes = [
            self._node(ctx, campaign_external_id, cluster, external_id)
            for cluster, external_id in zip(clusters, external_ids)
            if external_id not in existing
        ]
        for i in range(0, len(nodes), _DEFECT_CHUNK):
            self._client.data_modeling.instances.apply(nodes=nodes[i : i + _DEFECT_CHUNK])
        self._log.info(
            "Stored {} defect(s) ({} already existed) for mission {}".format(
                len(nodes), len(existing), ctx.mission_id
            )
        )
        return DefectResult(
            defect_external_ids=external_ids,
            created=len(nodes),
            already_existing=len(existing),
        )

    def _external_id(self, mission_id, member_ids):
        digest = hashlib.sha1("+".join(sorted(member_ids)).encode("utf-8")).hexdigest()[:12]
        return "defect-{}-{}".format(mission_id, digest)

    def _existing(self, external_ids):
        result = self._client.data_modeling.instances.retrieve(
            nodes=[(self._space, external_id) for external_id in external_ids]
        )
        return {node.external_id for node in result.nodes}

    def _node(self, ctx, campaign_external_id, cluster, external_id):
        centre = cluster.position
        properties = {
            "probability": (
                cluster.confidence if cluster.confidence is not None else DEFAULT_DEFECT_PROBABILITY
            ),
            "defectClass": "+".join(cluster.classes) if cluster.classes else "finding",
            # Oriented box [cx, cy, cz, hx, hy, hz, rx, ry, rz]: the centre with zero
            # extents/rotation, like the viewer's manually placed defects.
            "boundingBox3d": [float(centre[0]), float(centre[1]), float(centre[2]),
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "status": "New",
            "source": "ml",
        }
        if cluster.normal is not None:
            properties["normal3d"] = [float(v) for v in cluster.normal]
        if campaign_external_id:
            properties["campaign"] = {"space": self._space, "externalId": campaign_external_id}
        else:
            properties["area"] = {"space": self._space, "externalId": ctx.area_external_id}
        return NodeApply(
            space=self._space,
            external_id=external_id,
            sources=[
                NodeOrEdgeData(
                    source=ViewId(self._space, *DEFECT_DETECTION_VIEW), properties=properties
                )
            ],
        )


class MissionFlow:
    """One mission's writes in order: upload, then the findings' defects, one final status.

    The runner is constructed with `publish_status=flow.runner_publish_status`: intermediate
    statuses pass through, the terminal one (complete/failed) is held back until the defects
    are stored on the new campaign, then published once with the `findings` field filled in.
    A partial upload failure still stores the defects. When storing them fails, the buffer is
    kept for a `~submit_findings` retry. `log` needs info(), warning() and error().
    """

    def __init__(self, runner: Optional[Any], findings_service: Any, buffer: Any,
                 publish_status: Callable[[Dict[str, Any]], None], log: Any) -> None:
        self._runner = runner  # may be bound later (the runner needs runner_publish_status)
        self._service = findings_service
        self._buffer = buffer
        self._publish = publish_status
        self._log = log
        self._held: Optional[Dict[str, Any]] = None
        self._mission_campaigns: Dict[str, Optional[str]] = {}
        self._lock = threading.Lock()

    def bind(self, runner: Any) -> None:
        self._runner = runner

    def runner_publish_status(self, status: Dict[str, Any]) -> None:
        if status.get("state") in ("complete", "failed"):
            self._held = status
        else:
            self._publish(status)

    def run(self, ctx: Optional[MissionContext]) -> Tuple[bool, str]:
        if not self._lock.acquire(False):
            return False, "An upload is already running"
        try:
            return self._run_locked(ctx)
        finally:
            self._lock.release()

    def _run_locked(self, ctx: Optional[MissionContext]) -> Tuple[bool, str]:
        if ctx is None:
            self._publish(status_message("failed", None, message=NO_PLAN_MESSAGE))
            self._log.error(NO_PLAN_MESSAGE)
            return False, NO_PLAN_MESSAGE
        self._held = None
        ok, message = self._runner.run(ctx)
        campaign = (self._held or {}).get("campaignExternalId")
        if campaign or ctx.mission_id not in self._mission_campaigns:
            self._mission_campaigns[ctx.mission_id] = campaign
        findings_info, note = self._create_defects(ctx)
        final = self._held or status_message("complete" if ok else "failed", ctx, message=message)
        final["findings"] = findings_info
        final["updatedAt"] = datetime.now(timezone.utc).isoformat()
        self._publish(final)
        return ok, message + note

    def submit_findings(self, ctx: Optional[MissionContext]) -> Tuple[bool, str]:
        """Manual ~submit_findings: store the buffered findings as defects, outside an upload."""
        if ctx is None:
            return False, NO_PLAN_MESSAGE
        snapshot = self._buffer.snapshot()
        if not snapshot:
            return False, "No findings buffered"
        campaign = self._mission_campaigns.get(ctx.mission_id)
        try:
            result = self._service.create(ctx, campaign, snapshot)
        except Exception as exc:  # noqa: BLE001 - report; the buffer is kept for a retry
            message = "Storing the defects failed: {}: {}".format(type(exc).__name__, exc)
            self._log.error(message)
            return False, message
        self._buffer.discard([f.id for f in snapshot])
        return True, "Stored {} defect(s): {}".format(
            len(result.defect_external_ids), ", ".join(result.defect_external_ids)
        )

    def _create_defects(self, ctx):
        snapshot = self._buffer.snapshot()
        if not snapshot:
            return None, ""
        campaign = self._mission_campaigns.get(ctx.mission_id)
        try:
            result = self._service.create(ctx, campaign, snapshot)
        except Exception as exc:  # noqa: BLE001 - keep the buffer for ~submit_findings
            message = ("Storing the defects failed (findings kept for ~submit_findings): "
                       "{}: {}".format(type(exc).__name__, exc))
            self._log.error(message)
            return {"error": "{}: {}".format(type(exc).__name__, exc)}, ""
        self._buffer.discard([f.id for f in snapshot])
        return (
            {
                "count": len(result.defect_external_ids),
                "defectExternalIds": list(result.defect_external_ids),
            },
            "; {} defect(s) stored on the campaign".format(len(result.defect_external_ids)),
        )
