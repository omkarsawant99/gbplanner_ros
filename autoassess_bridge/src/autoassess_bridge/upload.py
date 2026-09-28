# SPDX-License-Identifier: BSD-3-Clause
"""Upload a finished mission to CDF: CogniteFiles plus an AutoAssess campaign. ROS-free.

This is the only module that writes to CDF, and the node only uses it when ~upload_enabled is
true. It follows the AutoAssess ground-station SDK (`dss campaign upload`):

- every file is a CogniteFile node (cdf_cdm:CogniteFile/v1) in the AutoAssess space, with
  mimeType application/octet-stream and the tags autoassess, ply_mesh or pcd_pointcloud,
  area:<id> (and label:<label> for point clouds); the content is uploaded by instance id.
  The bridge adds plan:<id> and mission:<id> tags. The external id has the SDK's shape,
  {area}-file-{12 hex}-{name}, but the hex part is derived from the mission and the file path
  so that retrying a mission reuses the files already uploaded;
- the campaign is an InspectionResultView node result-<uuid4> with the area, today's date,
  status InProgress and createdBy autoassess_bridge; its file ids are set once the files are
  uploaded, then the status becomes Complete. Nothing existing is changed or deleted.
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

CREATED_BY = "autoassess_bridge"
MIME_TYPE = "application/octet-stream"
INSPECTION_RESULT_VIEW = ("InspectionResultView", "1")
_MAX_EXTERNAL_ID = 255
_UNSAFE = re.compile(r"[^A-Za-z0-9._-]")
_KINDS = {".ply": "ply", ".pcd": "pcd"}


@dataclass(frozen=True)
class MissionContext:
    area_external_id: str
    mission_id: str
    plan_external_id: Optional[str] = None


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
) -> Dict[str, Any]:
    """Content of /autoassess/upload_status (published as JSON)."""
    return {
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


class FollowedPlans:
    """Which plan (and so which area) the bridge followed when; thread-safe.

    at(t) is the plan followed at time t: the last one recorded at or before t, or the first one
    if the bridge only picked up a plan after t (it was started mid-mission).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._records: List[Tuple[float, str, str]] = []

    def __len__(self) -> int:
        with self._lock:
            return len(self._records)

    def record(self, now: float, plan_external_id: str, area_external_id: str) -> None:
        with self._lock:
            if self._records and self._records[-1][1:] == (plan_external_id, area_external_id):
                return
            self._records.append((now, plan_external_id, area_external_id))

    def at(self, when: float) -> Optional[Tuple[str, str]]:
        with self._lock:
            if not self._records:
                return None
            chosen = self._records[0]
            for record in self._records:
                if record[0] <= when:
                    chosen = record
            return chosen[1], chosen[2]

    def latest(self) -> Optional[Tuple[str, str]]:
        with self._lock:
            return self._records[-1][1:] if self._records else None


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
