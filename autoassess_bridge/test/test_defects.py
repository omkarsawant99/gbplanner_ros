# SPDX-License-Identifier: BSD-3-Clause
"""Tests for storing findings as DefectDetection nodes and the mission-end flow (upload.py)."""

from __future__ import annotations

import hashlib
from typing import Any, Dict, List, Optional, Tuple

import pytest
from conftest import SPACE, FakeWriteClient, RecordingLog

from autoassess_bridge import upload
from autoassess_bridge.findings import Finding, FindingsBuffer
from autoassess_bridge.upload import DefectResult, DefectService, MissionContext, MissionFlow, UploadResult

MISSION = "mission-20260101T000000Z"
CTX = MissionContext(
    area_external_id="area-1",
    mission_id=MISSION,
    plan_external_id="plan-flown",
    map_external_id="result-map",
    plan_name="Weekly inspection",
)
CAMPAIGN = "result-up"
DEFECT_VIEW_REF = {"space": SPACE, "externalId": "DefectDetectionView", "version": "1", "type": "view"}


def defect_xid(*member_ids: str) -> str:
    digest = hashlib.sha1("+".join(sorted(member_ids)).encode("utf-8")).hexdigest()[:12]
    return "defect-{}-{}".format(MISSION, digest)


# --- DefectService --------------------------------------------------------------------------


def test_create_writes_one_defect_node_per_finding() -> None:
    client = FakeWriteClient()

    result = _service(client).create(
        CTX, CAMPAIGN, [Finding("a", (1, 2, 3), normal=(0, 0, 1), finding_class="corrosion", confidence=0.9)]
    )

    (node,) = client.instances.applied
    assert node["space"] == SPACE
    assert node["externalId"] == defect_xid("a")
    assert node["sources"][0]["source"] == DEFECT_VIEW_REF
    assert node["sources"][0]["properties"] == {
        "campaign": {"space": SPACE, "externalId": CAMPAIGN},
        "probability": 0.9,
        "defectClass": "corrosion",
        "boundingBox3d": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "status": "New",
        "normal3d": [0.0, 0.0, 1.0],
        "source": "ml",
    }
    assert result == DefectResult(defect_external_ids=[defect_xid("a")], created=1, already_existing=0)


def test_defaults_apply_without_class_confidence_or_normal() -> None:
    client = FakeWriteClient()

    _service(client).create(CTX, CAMPAIGN, [Finding("a", (1, 2, 3))])

    props = client.instances.applied[0]["sources"][0]["properties"]
    assert props["defectClass"] == "finding"
    assert props["probability"] == 0.5  # probability is non-nullable in the container
    assert "normal3d" not in props  # no centroid fallback: defects allow a missing normal


def test_nearby_findings_merge_into_one_defect() -> None:
    client = FakeWriteClient()

    result = _service(client).create(
        CTX,
        CAMPAIGN,
        [
            Finding("a", (0, 0, 0), finding_class="crack", confidence=0.4),
            Finding("b", (0.2, 0, 0), finding_class="corrosion", confidence=0.8),
            Finding("c", (5, 0, 0)),
        ],
    )

    assert result.created == 2
    merged = [n for n in client.instances.applied if n["externalId"] == defect_xid("a", "b")]
    assert len(merged) == 1
    props = merged[0]["sources"][0]["properties"]
    assert props["boundingBox3d"][:3] == [0.1, 0.0, 0.0]
    assert props["defectClass"] == "corrosion+crack"
    assert props["probability"] == 0.8


def test_retry_skips_defects_that_already_exist() -> None:
    client = FakeWriteClient()
    service = _service(client)
    service.create(CTX, CAMPAIGN, [Finding("a", (0, 0, 0))])

    result = service.create(CTX, CAMPAIGN, [Finding("a", (0, 0, 0)), Finding("b", (9, 9, 9))])

    assert result.created == 1
    assert result.already_existing == 1
    assert sorted(result.defect_external_ids) == sorted([defect_xid("a"), defect_xid("b")])
    assert [n["externalId"] for n in client.instances.applied].count(defect_xid("a")) == 1
    # idempotency comes from a retrieve of the deterministic ids, not a registry
    retrieves = client.instances.calls_to("retrieve")
    assert (SPACE, defect_xid("a")) in [tuple(n) for n in retrieves[-1]["nodes"]]


def test_defect_ids_are_deterministic_per_mission_and_finding() -> None:
    client = FakeWriteClient()
    other = MissionContext("area-1", "mission-other", "plan-flown")

    _service(client).create(CTX, CAMPAIGN, [Finding("a", (0, 0, 0))])
    _service(FakeWriteClient()).create(other, CAMPAIGN, [Finding("a", (0, 0, 0))])

    assert client.instances.applied[0]["externalId"] == defect_xid("a")
    assert defect_xid("a").startswith("defect-" + MISSION + "-")


def test_create_without_a_campaign_attaches_the_area_and_warns() -> None:
    client = FakeWriteClient()
    log = RecordingLog()

    _service(client, log).create(CTX, None, [Finding("a", (1, 2, 3))])

    props = client.instances.applied[0]["sources"][0]["properties"]
    assert "campaign" not in props
    assert props["area"] == {"space": SPACE, "externalId": "area-1"}
    assert any("no campaign" in m for m in log.messages("warning"))


def test_create_without_findings_raises() -> None:
    with pytest.raises(ValueError):
        _service(FakeWriteClient()).create(CTX, CAMPAIGN, [])

    with pytest.raises(ValueError):
        _service(FakeWriteClient()).create(None, CAMPAIGN, [Finding("a", (0, 0, 0))])


# --- MissionFlow ----------------------------------------------------------------------------


def test_flow_publishes_one_final_status_with_the_defects() -> None:
    statuses: List[Dict[str, Any]] = []
    buffer = _buffer([Finding("a", (1, 2, 3))])
    flow = _flow(_FakeRunner(ok=True), _FakeService(), buffer, statuses)

    ok, message = flow.run(CTX)

    assert ok
    assert "defect" in message
    terminal = [s for s in statuses if s["state"] in ("complete", "failed")]
    assert len(terminal) == 1  # exactly one final status, published after the defects
    assert terminal[0]["state"] == "complete"
    assert terminal[0]["findings"] == {"count": 1, "defectExternalIds": [defect_xid("a")]}
    assert [s["state"] for s in statuses] == ["uploading", "complete"]
    assert len(buffer) == 0  # the snapshot was cleared after success


def test_flow_hands_the_new_campaign_to_the_defects() -> None:
    service = _FakeService()

    _flow(_FakeRunner(ok=True, campaign="result-new"), service, _buffer([Finding("a", (1, 2, 3))]), []).run(CTX)

    ((_, campaign, _),) = service.calls
    assert campaign == "result-new"


def test_flow_without_findings_reports_none() -> None:
    statuses: List[Dict[str, Any]] = []
    service = _FakeService()

    ok, _ = _flow(_FakeRunner(ok=True), service, _buffer([]), statuses).run(CTX)

    assert ok
    assert service.calls == []
    assert statuses[-1]["findings"] is None


def test_flow_creates_defects_even_when_the_upload_partially_failed() -> None:
    statuses: List[Dict[str, Any]] = []
    service = _FakeService()

    ok, message = _flow(_FakeRunner(ok=False), service, _buffer([Finding("a", (1, 2, 3))]), statuses).run(CTX)

    assert not ok
    assert len(service.calls) == 1
    assert statuses[-1]["state"] == "failed"
    assert statuses[-1]["findings"]["count"] == 1
    assert "defect" in message


def test_flow_keeps_the_buffer_when_defect_creation_fails() -> None:
    statuses: List[Dict[str, Any]] = []
    buffer = _buffer([Finding("a", (1, 2, 3))])
    log = RecordingLog()
    flow = _flow(_FakeRunner(ok=True), _FakeService(raises=RuntimeError("CDF down")), buffer, statuses, log)

    ok, _ = flow.run(CTX)

    assert ok  # the upload itself succeeded
    assert len(buffer) == 1
    assert statuses[-1]["findings"] == {"error": "RuntimeError: CDF down"}
    assert any("CDF down" in m for m in log.messages("error"))


def test_flow_with_no_mission_context_reports_the_no_plan_failure() -> None:
    statuses: List[Dict[str, Any]] = []
    service = _FakeService()

    ok, message = _flow(_FakeRunner(ok=False), service, _buffer([Finding("a", (1, 2, 3))]), statuses).run(None)

    assert not ok
    assert "No plan has been followed" in message
    assert service.calls == []


def test_submit_findings_creates_defects_with_the_missions_campaign() -> None:
    buffer = _buffer([Finding("a", (1, 2, 3))])
    service = _FakeService(raises=RuntimeError("down"))
    statuses: List[Dict[str, Any]] = []
    flow = _flow(_FakeRunner(ok=True, campaign="result-mission"), service, buffer, statuses)
    flow.run(CTX)  # defect creation failed; the buffer is kept
    service.raises = None

    ok, message = flow.submit_findings(CTX)

    assert ok
    assert defect_xid("a") in message
    ((_, campaign, _),) = service.calls
    assert campaign == "result-mission"  # remembered from the mission's upload
    assert len(buffer) == 0


def test_flow_first_mapping_passes_no_campaign_to_the_defects() -> None:
    ctx = MissionContext("area-1", MISSION, None, first_mapping=True)
    service = _FakeService()
    statuses: List[Dict[str, Any]] = []

    ok, _ = _flow(_FakeRunner(ok=True, campaign=None), service, _buffer([Finding("a", (1, 2, 3))]), statuses).run(ctx)

    assert ok
    ((_, campaign, _),) = service.calls
    assert campaign is None
    assert statuses[-1]["campaignExternalId"] is None
    assert statuses[-1]["note"] == "no plan followed: uploaded without a campaign (first mapping)"
    assert statuses[-1]["findings"]["count"] == 1


def test_submit_findings_without_findings_fails() -> None:
    ok, message = _flow(_FakeRunner(ok=True), _FakeService(), _buffer([]), []).submit_findings(CTX)

    assert not ok
    assert "No findings" in message


def test_submit_findings_without_a_context_fails_with_the_no_plan_message() -> None:
    ok, message = _flow(_FakeRunner(ok=True), _FakeService(), _buffer([Finding("a", (1, 2, 3))]), []).submit_findings(None)

    assert not ok
    assert "No plan has been followed" in message


# --- helpers --------------------------------------------------------------------------------


def _service(client: FakeWriteClient, log: Any = None) -> DefectService:
    return DefectService(client, log or RecordingLog(), merge_radius_m=0.5)


def _buffer(findings: List[Finding]) -> FindingsBuffer:
    buffer = FindingsBuffer()
    for finding in findings:
        buffer.add(finding)
    return buffer


class _FakeRunner:
    """Publishes intermediate + terminal statuses through the flow's gate, like the real one."""

    def __init__(self, ok: bool, campaign: Optional[str] = CAMPAIGN) -> None:
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
        self.raises = raises

    def create(self, ctx: MissionContext, campaign_external_id: Optional[str], findings: List[Finding]) -> Any:
        if self.raises is not None:
            raise self.raises
        self.calls.append((ctx, campaign_external_id, list(findings)))
        ids = [defect_xid(f.id) for f in findings]
        return DefectResult(defect_external_ids=ids, created=len(ids), already_existing=0)


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
