# SPDX-License-Identifier: BSD-3-Clause
"""Tests for creating the Draft findings plan in CDF and the mission-end flow (upload.py)."""

from __future__ import annotations

import uuid
from typing import Any, Dict, List, Optional, Tuple

import pytest
from conftest import SPACE, FakeNode, FakeWriteClient, RecordingLog

from autoassess_bridge import upload
from autoassess_bridge.findings import Finding, FindingsBuffer
from autoassess_bridge.upload import (
    FindingsPlanService,
    MissionContext,
    MissionFlow,
    UploadResult,
)

PLAN_UUID = uuid.UUID("99999999-8888-4777-8666-555555555555")
PLAN_XID = "plan-99999999-8888-4777-8666-555555555555"
CTX = MissionContext(
    area_external_id="area-1",
    mission_id="mission-20260101T000000Z",
    plan_external_id="plan-flown",
    map_external_id="result-map",
    plan_name="Weekly inspection",
)
PLAN_VIEW_REF = {"space": SPACE, "externalId": "InspectionPlanView", "version": "4", "type": "view"}
TASK_VIEW_REF = {"space": SPACE, "externalId": "InspectionTaskView", "version": "1", "type": "view"}
TASK_VIEW = ("InspectionTaskView", "1")


# --- FindingsPlanService --------------------------------------------------------------------


def test_create_writes_a_draft_plan_like_the_sdk_plus_provenance() -> None:
    client = FakeWriteClient()

    result = _service(client).create(CTX, "result-map", [Finding("a", (1, 2, 3))])

    plan_node = client.instances.applied[0]
    assert plan_node["space"] == SPACE
    assert plan_node["externalId"] == PLAN_XID
    assert plan_node["sources"][0]["source"] == PLAN_VIEW_REF
    assert plan_node["sources"][0]["properties"] == {
        "area": {"space": SPACE, "externalId": "area-1"},
        "map": {"space": SPACE, "externalId": "result-map"},
        "status": "Draft",
        "name": "Findings mission-20260101T000000Z",
        "description": (
            "Created by autoassess_bridge from mission mission-20260101T000000Z / plan plan-flown"
        ),
    }
    assert result.plan_external_id == PLAN_XID
    assert result.task_count == 1


def test_create_writes_region_tasks_like_the_sdk() -> None:
    client = FakeWriteClient()

    _service(client).create(CTX, "result-map", [Finding("a", (1, 2, 3), normal=(0, 0, 1), radius_m=0.4)])

    (task_node,) = [n for n in client.instances.applied if n["externalId"].startswith("task-")]
    assert task_node["space"] == SPACE
    assert task_node["sources"][0]["source"] == TASK_VIEW_REF
    assert task_node["sources"][0]["properties"] == {
        "plan": {"space": SPACE, "externalId": PLAN_XID},
        "taskType": "region",
        "inspectionType": "visual",
        "position3d": [1.0, 2.0, 3.0],
        "normalVector": [0.0, 0.0, 1.0],
        "radiusM": 0.4,
        "suggestionId": "finding:a",
    }


def test_create_without_a_map_omits_the_relation_and_warns() -> None:
    client = FakeWriteClient()
    log = RecordingLog()

    _service(client, log).create(CTX, None, [Finding("a", (1, 2, 3))])

    assert "map" not in client.instances.applied[0]["sources"][0]["properties"]
    assert any("no reference map" in m for m in log.messages("warning"))


def test_create_without_a_flown_plan_says_so_in_the_description() -> None:
    client = FakeWriteClient()
    ctx = MissionContext("area-1", "mission-x", None)

    _service(client).create(ctx, None, [Finding("a", (1, 2, 3))])

    description = client.instances.applied[0]["sources"][0]["properties"]["description"]
    assert description == "Created by autoassess_bridge from mission mission-x"


def test_create_merges_findings_within_the_radius() -> None:
    client = FakeWriteClient()

    result = _service(client, merge_radius_m=0.5).create(
        CTX, None, [Finding("a", (0, 0, 0)), Finding("b", (0.2, 0, 0)), Finding("c", (5, 0, 0))]
    )

    tasks = [n for n in client.instances.applied if n["externalId"].startswith("task-")]
    assert result.task_count == 2
    assert len(tasks) == 2


def test_retry_reuses_the_plan_and_skips_existing_suggestion_ids() -> None:
    client = FakeWriteClient()
    service = _service(client)
    service.create(CTX, "result-map", [Finding("a", (0, 0, 0))])
    client.instances.add(
        TASK_VIEW,
        FakeNode("task-existing", TASK_VIEW, {"plan": {"externalId": PLAN_XID}, "suggestionId": "finding:a"}),
    )

    result = service.create(CTX, "result-map", [Finding("a", (0, 0, 0)), Finding("b", (9, 9, 9))])

    assert result.plan_external_id == PLAN_XID
    assert result.task_count == 1
    assert result.already_in_plan == 1
    # one plan node ever; the retry listed the plan's tasks with a filter on the plan
    assert [n["externalId"] for n in client.instances.applied].count(PLAN_XID) == 1
    (list_call,) = client.instances.calls_to("list")
    assert list_call["filter"] == {
        "equals": {
            "property": [SPACE, "InspectionTaskContainer", "plan"],
            "value": {"space": SPACE, "externalId": PLAN_XID},
        }
    }


def test_create_with_no_new_tasks_writes_nothing() -> None:
    client = FakeWriteClient()
    service = _service(client)
    service.create(CTX, None, [Finding("a", (0, 0, 0))])
    client.instances.add(
        TASK_VIEW,
        FakeNode("task-1", TASK_VIEW, {"suggestionId": "finding:a"}),
    )
    before = len(client.instances.applied)

    result = service.create(CTX, None, [Finding("a", (0.0, 0, 0))])

    assert result.task_count == 0
    assert result.already_in_plan == 1
    assert len(client.instances.applied) == before


def test_create_without_findings_raises() -> None:
    with pytest.raises(ValueError):
        _service(FakeWriteClient()).create(CTX, None, [])


# --- MissionFlow ----------------------------------------------------------------------------


def test_flow_publishes_one_final_status_with_the_findings_plan() -> None:
    statuses: List[Dict[str, Any]] = []
    buffer = _buffer([Finding("a", (1, 2, 3))])
    flow = _flow(_FakeRunner(ok=True), _FakeService(), buffer, statuses)

    ok, message = flow.run(CTX)

    assert ok
    assert PLAN_XID in message
    terminal = [s for s in statuses if s["state"] in ("complete", "failed")]
    assert len(terminal) == 1  # exactly one final status, published after the findings plan
    assert terminal[0]["state"] == "complete"
    assert terminal[0]["findings"] == {"count": 1, "planExternalId": PLAN_XID}
    assert [s["state"] for s in statuses] == ["uploading", "complete"]
    assert len(buffer) == 0  # the snapshot was cleared after success


def test_flow_without_findings_reports_none() -> None:
    statuses: List[Dict[str, Any]] = []
    service = _FakeService()

    ok, _ = _flow(_FakeRunner(ok=True), service, _buffer([]), statuses).run(CTX)

    assert ok
    assert service.calls == []
    assert statuses[-1]["findings"] is None


def test_flow_creates_the_plan_even_when_the_upload_partially_failed() -> None:
    statuses: List[Dict[str, Any]] = []
    service = _FakeService()

    ok, message = _flow(_FakeRunner(ok=False), service, _buffer([Finding("a", (1, 2, 3))]), statuses).run(CTX)

    assert not ok
    assert len(service.calls) == 1
    assert statuses[-1]["state"] == "failed"
    assert statuses[-1]["findings"] == {"count": 1, "planExternalId": PLAN_XID}
    assert PLAN_XID in message


def test_flow_keeps_the_buffer_when_plan_creation_fails() -> None:
    statuses: List[Dict[str, Any]] = []
    buffer = _buffer([Finding("a", (1, 2, 3))])
    log = RecordingLog()
    flow = _flow(_FakeRunner(ok=True), _FakeService(raises=RuntimeError("CDF down")), buffer, statuses, log)

    ok, _ = flow.run(CTX)

    assert ok  # the upload itself succeeded
    assert len(buffer) == 1
    assert statuses[-1]["findings"] == {"error": "RuntimeError: CDF down"}
    assert any("CDF down" in m for m in log.messages("error"))


def test_flow_maps_fall_back_to_the_uploaded_campaign() -> None:
    service = _FakeService()
    ctx = MissionContext("area-1", "mission-x", "plan-flown", map_external_id=None)

    _flow(_FakeRunner(ok=True, campaign="result-new"), service, _buffer([Finding("a", (1, 2, 3))]), []).run(ctx)

    ((_, map_id, _),) = service.calls
    assert map_id == "result-new"


def test_flow_prefers_the_flown_plans_map() -> None:
    service = _FakeService()

    _flow(_FakeRunner(ok=True, campaign="result-new"), service, _buffer([Finding("a", (1, 2, 3))]), []).run(CTX)

    ((_, map_id, _),) = service.calls
    assert map_id == "result-map"


def test_flow_with_no_mission_context_reports_the_no_plan_failure() -> None:
    statuses: List[Dict[str, Any]] = []
    service = _FakeService()

    ok, message = _flow(_FakeRunner(ok=False), service, _buffer([Finding("a", (1, 2, 3))]), statuses).run(None)

    assert not ok
    assert "No plan has been followed" in message
    assert service.calls == []


def test_submit_findings_creates_a_plan_outside_the_upload() -> None:
    buffer = _buffer([Finding("a", (1, 2, 3))])
    service = _FakeService()
    flow = _flow(_FakeRunner(ok=True), service, buffer, [])

    ok, message = flow.submit_findings(CTX)

    assert ok
    assert PLAN_XID in message
    assert len(service.calls) == 1
    assert len(buffer) == 0


def test_submit_findings_without_findings_fails() -> None:
    ok, message = _flow(_FakeRunner(ok=True), _FakeService(), _buffer([]), []).submit_findings(CTX)

    assert not ok
    assert "No findings" in message


def test_submit_findings_without_a_context_fails_with_the_no_plan_message() -> None:
    ok, message = _flow(_FakeRunner(ok=True), _FakeService(), _buffer([Finding("a", (1, 2, 3))]), []).submit_findings(None)

    assert not ok
    assert "No plan has been followed" in message


# --- helpers --------------------------------------------------------------------------------


def _service(client: FakeWriteClient, log: Any = None, merge_radius_m: float = 0.5) -> FindingsPlanService:
    return FindingsPlanService(
        client,
        log or RecordingLog(),
        merge_radius_m=merge_radius_m,
        new_uuid=lambda: PLAN_UUID,
    )


def _buffer(findings: List[Finding]) -> FindingsBuffer:
    buffer = FindingsBuffer()
    for finding in findings:
        buffer.add(finding)
    return buffer


class _FakeRunner:
    """Publishes intermediate + terminal statuses through the flow's gate, like the real one."""

    def __init__(self, ok: bool, campaign: str = "result-up") -> None:
        self._ok = ok
        self._campaign = campaign
        self.publish_status = None  # type: Optional[Any]

    def run(self, ctx: MissionContext) -> Tuple[bool, str]:
        assert self.publish_status is not None
        self.publish_status(upload.status_message("uploading", ctx))
        result = UploadResult(self._campaign, [11], [], [], failed=[] if self._ok else [("/f", "e")])
        state = "complete" if self._ok else "failed"
        self.publish_status(upload.status_message(state, ctx, result, "upload message"))
        return self._ok, "upload message"


class _FakeService:
    def __init__(self, raises: Optional[Exception] = None) -> None:
        self.calls: List[Tuple[MissionContext, Optional[str], List[Finding]]] = []
        self._raises = raises

    def create(self, ctx: MissionContext, map_external_id: Optional[str], findings: List[Finding]) -> Any:
        if self._raises is not None:
            raise self._raises
        self.calls.append((ctx, map_external_id, list(findings)))
        return upload.FindingsPlanResult(PLAN_XID, task_count=len(findings), already_in_plan=0)


def _flow(
    runner: _FakeRunner,
    service: Any,
    buffer: FindingsBuffer,
    statuses: List[Dict[str, Any]],
    log: Any = None,
) -> MissionFlow:
    flow = MissionFlow(runner, service, buffer, statuses.append, log or RecordingLog())
    runner.publish_status = flow.runner_publish_status
    return flow
