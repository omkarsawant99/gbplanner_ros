# SPDX-License-Identifier: BSD-3-Clause
"""Conversions between the bridge's internal dicts and the autoassess_bridge ROS messages.

ROS-free: the generated classes in `autoassess_bridge.msg` only exist after a catkin build,
so the node injects them through `MessageClasses` (tests inject lightweight doubles). The
classes must construct from keyword arguments, like genpy messages do.

- `plan_to_msg`: the plan.json dict (cdf.PlanSource.plan_json) -> InspectionPlan message.
- `finding_msg_to_entry`: a Finding message -> the JSON-findings field dict, so the message
  goes through exactly the validation of /autoassess/findings_json
  (`findings.parse_finding_entry`).
- `upload_status_to_msg`: the status dict (upload.status_message) -> UploadStatus message.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict

from autoassess_bridge.plan import task_target


@dataclass(frozen=True)
class MessageClasses:
    """The ROS message classes the conversions construct, injected by the node."""

    inspection_plan: Callable[..., Any]  # autoassess_bridge/InspectionPlan
    inspection_task: Callable[..., Any]  # autoassess_bridge/InspectionTask
    upload_status: Callable[..., Any]  # autoassess_bridge/UploadStatus
    point: Callable[..., Any]  # geometry_msgs/Point
    vector3: Callable[..., Any]  # geometry_msgs/Vector3


def plan_to_msg(plan: Dict[str, Any], classes: MessageClasses, frame_id: str) -> Any:
    """The plan.json dict as an InspectionPlan message (header.stamp is left to the caller).

    Every task is kept, in plan order; a task without a target point gets position (0,0,0)
    (the same tasks are left out of /autoassess/inspection_targets and logged).
    """
    msg = classes.inspection_plan(
        external_id=str(plan["planExternalId"]),
        name=str(plan.get("name") or ""),
        area_external_id=str(plan.get("areaExternalId") or ""),
        map_external_id=str(plan.get("mapExternalId") or ""),
        tasks=[_task_to_msg(task, classes) for task in plan.get("tasks") or []],
    )
    msg.header.frame_id = frame_id
    return msg


def _task_to_msg(task: Dict[str, Any], classes: MessageClasses) -> Any:
    target = task_target(task)
    normal = task.get("normalVector") if task.get("kind") == "region" else None
    has_normal = normal is not None and len(normal) == 3
    return classes.inspection_task(
        id=str(task.get("id") or ""),
        task_type=str(task.get("kind") or ""),
        inspection_type=str(task.get("inspectionType") or ""),
        position=classes.point(*(target if target is not None else (0.0, 0.0, 0.0))),
        normal=classes.vector3(*normal) if has_normal else classes.vector3(),
        has_normal=bool(has_normal),
        radius_m=float(task.get("radiusM") or 0.0),
        target_element_id=str((task.get("targetElement") or {}).get("externalId") or ""),
    )


def finding_msg_to_entry(msg: Any) -> Dict[str, Any]:
    """A Finding message as the field dict of one /autoassess/findings_json entry.

    Message defaults ("" and 0) mean "not given" and are left out, so
    `findings.parse_finding_entry` applies the same defaults and validation as the JSON path.
    """
    entry: Dict[str, Any] = {"x": msg.position.x, "y": msg.position.y, "z": msg.position.z}
    if msg.id:
        entry["id"] = msg.id
    if msg.has_normal:
        entry["nx"], entry["ny"], entry["nz"] = msg.normal.x, msg.normal.y, msg.normal.z
    if msg.radius_m:
        entry["radius"] = msg.radius_m
    if msg.inspection_type:
        entry["inspection_type"] = msg.inspection_type
    if msg.defect_class:
        entry["class"] = msg.defect_class
    if msg.confidence:
        entry["confidence"] = msg.confidence
    if msg.description:
        entry["description"] = msg.description
    return entry


def upload_status_to_msg(status: Dict[str, Any], classes: MessageClasses) -> Any:
    """The upload.status_message dict as an UploadStatus message (header.stamp left to the
    caller). The JSON mirror on /autoassess/upload_status_json keeps the full detail (file
    ids and labels, failed/skipped file lists, the first-mapping note, updatedAt)."""
    findings = status.get("findings") or {}
    files_done = len(status.get("cdfFileIds") or []) + len(status.get("pcdFileIds") or [])
    return classes.upload_status(
        state=str(status.get("state") or ""),
        mission_id=str(status.get("missionId") or ""),
        campaign_external_id=str(status.get("campaignExternalId") or ""),
        files_done=files_done,
        files_total=files_done + len(status.get("failedFiles") or []),
        findings_count=int(findings.get("count") or 0),
        defect_external_ids=[str(i) for i in findings.get("defectExternalIds") or []],
        message=str(status.get("message") or ""),
    )
