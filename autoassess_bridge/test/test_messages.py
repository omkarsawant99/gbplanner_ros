# SPDX-License-Identifier: BSD-3-Clause
"""messages.py: plan.json -> InspectionPlan msg, Finding msg -> Finding, status -> UploadStatus msg.

The generated ROS message classes only exist after a catkin build, so messages.py takes them
injected (`MessageClasses`); these tests pass the lightweight doubles below, which mimic the
keyword construction and default field values of genpy messages.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Optional

from autoassess_bridge.findings import parse_finding_entry
from autoassess_bridge.messages import (
    MessageClasses,
    finding_msg_to_entry,
    plan_to_msg,
    upload_status_to_msg,
)
from autoassess_bridge.upload import MissionContext, UploadResult, status_message


# --- plan -----------------------------------------------------------------------------------


def test_plan_to_msg_region_task() -> None:
    plan = plan_dict(
        tasks=[
            {
                "id": "task-1",
                "kind": "region",
                "inspectionType": "ndt_thickness",
                "position3d": [1.0, 2.0, 3.0],
                "normalVector": [0.0, 0.0, 1.0],
                "radiusM": 0.4,
            }
        ]
    )

    msg = plan_to_msg(plan, CLASSES, "world")

    assert msg.header.frame_id == "world"
    assert msg.external_id == "plan-1"
    assert msg.name == "Plan One"
    assert msg.area_external_id == "area-1"
    assert msg.map_external_id == "map-1"
    task = only(msg.tasks)
    assert task.id == "task-1"
    assert task.task_type == "region"
    assert task.inspection_type == "ndt_thickness"
    assert (task.position.x, task.position.y, task.position.z) == (1.0, 2.0, 3.0)
    assert task.has_normal is True
    assert (task.normal.x, task.normal.y, task.normal.z) == (0.0, 0.0, 1.0)
    assert task.radius_m == 0.4
    assert task.target_element_id == ""


def test_plan_to_msg_element_task_targets_the_element_centre() -> None:
    plan = plan_dict(
        tasks=[
            {
                "id": "task-2",
                "kind": "element",
                "inspectionType": "visual",
                "targetElement": {"externalId": "el-9", "type": "girder", "center": [4.0, 5.0, 6.0]},
            }
        ]
    )

    task = only(plan_to_msg(plan, CLASSES, "world").tasks)

    assert task.task_type == "element"
    assert (task.position.x, task.position.y, task.position.z) == (4.0, 5.0, 6.0)
    assert task.has_normal is False
    assert task.radius_m == 0.0
    assert task.target_element_id == "el-9"


def test_plan_to_msg_keeps_tasks_without_a_target() -> None:
    plan = plan_dict(tasks=[{"id": "task-3", "kind": "region", "inspectionType": "visual"}])

    task = only(plan_to_msg(plan, CLASSES, "world").tasks)

    assert task.id == "task-3"
    assert (task.position.x, task.position.y, task.position.z) == (0.0, 0.0, 0.0)
    assert task.has_normal is False


def test_plan_to_msg_maps_missing_optionals_to_empty_strings() -> None:
    plan = plan_dict(name=None, map_external_id=None, tasks=[])

    msg = plan_to_msg(plan, CLASSES, "world")

    assert msg.name == ""
    assert msg.map_external_id == ""
    assert msg.tasks == []


# --- findings -------------------------------------------------------------------------------


def test_finding_msg_round_trips_through_the_json_validation() -> None:
    msg = finding_msg(
        id="f-1",
        position=DoublePoint(1.0, 2.0, 3.0),
        normal=DoubleVector3(0.0, 0.0, 2.0),
        has_normal=True,
        radius_m=0.5,
        inspection_type="ndt_thickness",
        defect_class="corrosion",
        confidence=0.9,
        description="pitting",
    )

    finding, error = parse_finding_entry(finding_msg_to_entry(msg))

    assert error is None and finding is not None
    assert finding.id == "f-1"
    assert finding.position == (1.0, 2.0, 3.0)
    assert finding.normal == (0.0, 0.0, 1.0)  # normalised, like the JSON path
    assert finding.radius_m == 0.5
    assert finding.inspection_type == "ndt_thickness"
    assert finding.finding_class == "corrosion"
    assert finding.confidence == 0.9
    assert finding.description == "pitting"


def test_finding_msg_defaults_mean_not_given() -> None:
    msg = finding_msg(position=DoublePoint(1.0, 2.0, 3.0))

    finding, error = parse_finding_entry(finding_msg_to_entry(msg))

    assert error is None and finding is not None
    assert finding.id.startswith("row-")  # derived, like the JSON path
    assert finding.normal is None
    assert finding.radius_m is None
    assert finding.inspection_type is None
    assert finding.finding_class is None
    assert finding.confidence is None
    assert finding.description is None


def test_finding_msg_normal_ignored_without_has_normal() -> None:
    msg = finding_msg(position=DoublePoint(1.0, 2.0, 3.0), normal=DoubleVector3(0.0, 0.0, 1.0))

    finding, error = parse_finding_entry(finding_msg_to_entry(msg))

    assert error is None and finding is not None and finding.normal is None


def test_finding_msg_zero_normal_with_has_normal_is_an_error() -> None:
    msg = finding_msg(position=DoublePoint(1.0, 2.0, 3.0), has_normal=True)

    finding, error = parse_finding_entry(finding_msg_to_entry(msg))

    assert finding is None
    assert error is not None and "zero-length" in error


def test_finding_msg_bad_confidence_is_an_error() -> None:
    msg = finding_msg(position=DoublePoint(1.0, 2.0, 3.0), confidence=1.5)

    finding, error = parse_finding_entry(finding_msg_to_entry(msg))

    assert finding is None
    assert error is not None and "confidence" in error


# --- upload status --------------------------------------------------------------------------


def test_upload_status_to_msg_idle() -> None:
    msg = upload_status_to_msg(status_message("idle", None), CLASSES)

    assert msg.state == "idle"
    assert msg.mission_id == ""
    assert msg.campaign_external_id == ""
    assert (msg.files_done, msg.files_total, msg.findings_count) == (0, 0, 0)
    assert msg.defect_external_ids == []
    assert msg.message == ""


def test_upload_status_to_msg_complete_with_findings() -> None:
    ctx = MissionContext("area-1", "mission-1", "plan-1")
    result = UploadResult("result-abc", [11], [12, 13], ["a", "b"], failed=[("/x.ply", "boom")])
    status = status_message(
        "complete", ctx, result, "done",
        findings={"count": 2, "defectExternalIds": ["defect-1", "defect-2"]},
    )

    msg = upload_status_to_msg(status, CLASSES)

    assert msg.state == "complete"
    assert msg.mission_id == "mission-1"
    assert msg.campaign_external_id == "result-abc"
    assert msg.files_done == 3  # mesh + the two point clouds
    assert msg.files_total == 4  # plus the failed file
    assert msg.findings_count == 2
    assert msg.defect_external_ids == ["defect-1", "defect-2"]
    assert msg.message == "done"


def test_upload_status_to_msg_findings_error_counts_zero() -> None:
    ctx = MissionContext("area-1", "mission-1", "plan-1")
    status = status_message("failed", ctx, message="boom", findings={"error": "nope"})

    msg = upload_status_to_msg(status, CLASSES)

    assert msg.state == "failed"
    assert msg.findings_count == 0
    assert msg.defect_external_ids == []
    assert msg.message == "boom"


# --- helpers --------------------------------------------------------------------------------


class DoublePoint:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x, self.y, self.z = float(x), float(y), float(z)


class DoubleVector3(DoublePoint):
    pass


class _KwargsMsg:
    """Base for message doubles: keyword construction over declared defaults, like genpy."""

    def __init__(self, **kwargs: Any) -> None:
        for key, value in self.defaults().items():
            setattr(self, key, value)
        for key, value in kwargs.items():
            if not hasattr(self, key):
                raise TypeError("unexpected field {}".format(key))
            setattr(self, key, value)

    @staticmethod
    def defaults() -> dict:
        raise NotImplementedError


class DoubleInspectionTask(_KwargsMsg):
    @staticmethod
    def defaults() -> dict:
        return {
            "id": "", "task_type": "", "inspection_type": "",
            "position": DoublePoint(), "normal": DoubleVector3(), "has_normal": False,
            "radius_m": 0.0, "target_element_id": "",
        }


class DoubleInspectionPlan(_KwargsMsg):
    @staticmethod
    def defaults() -> dict:
        return {
            "header": SimpleNamespace(frame_id="", stamp=None),
            "external_id": "", "name": "", "area_external_id": "", "map_external_id": "",
            "tasks": [],
        }


class DoubleFinding(_KwargsMsg):
    @staticmethod
    def defaults() -> dict:
        return {
            "id": "", "position": DoublePoint(), "normal": DoubleVector3(), "has_normal": False,
            "radius_m": 0.0, "inspection_type": "", "defect_class": "", "confidence": 0.0,
            "description": "",
        }


class DoubleUploadStatus(_KwargsMsg):
    @staticmethod
    def defaults() -> dict:
        return {
            "header": SimpleNamespace(frame_id="", stamp=None),
            "state": "", "mission_id": "", "campaign_external_id": "",
            "files_done": 0, "files_total": 0, "findings_count": 0,
            "defect_external_ids": [], "message": "",
        }


CLASSES = MessageClasses(
    inspection_plan=DoubleInspectionPlan,
    inspection_task=DoubleInspectionTask,
    upload_status=DoubleUploadStatus,
    point=DoublePoint,
    vector3=DoubleVector3,
)


def finding_msg(**kwargs: Any) -> DoubleFinding:
    return DoubleFinding(**kwargs)


def plan_dict(
    tasks: list,
    name: Optional[str] = "Plan One",
    map_external_id: Optional[str] = "map-1",
) -> dict:
    return {
        "planExternalId": "plan-1",
        "name": name,
        "description": None,
        "areaExternalId": "area-1",
        "areaName": "Tank",
        "mapExternalId": map_external_id,
        "downloadedAt": "2026-01-01T00:00:00+00:00",
        "tasks": tasks,
    }


def only(items: list) -> Any:
    assert len(items) == 1, items
    return items[0]
