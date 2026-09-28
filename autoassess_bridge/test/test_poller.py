# SPDX-License-Identifier: BSD-3-Clause
"""Tests for change detection (autoassess_bridge.poller)."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from conftest import RecordingLog

from autoassess_bridge.cdf import AreaInfo
from autoassess_bridge.poller import PlanPoller, PlanUpdate

AREA_1 = AreaInfo("area-1", "Tank 1", "v-1", "Carrier")


def test_first_poll_returns_the_plan_and_area_centres() -> None:
    source = StubSource()
    poller = PlanPoller(source, RecordingLog())

    update = poller.poll()

    assert update == PlanUpdate(plan=source.payload(), element_centres=[(0.0, 0.0, 0.0)], area=AREA_1)
    assert source.plan_json_args == [("p-1", "Tank 1")]
    assert source.element_centres_args == ["area-1"]


def test_the_plan_area_comes_from_the_plan() -> None:
    source = StubSource()
    source.plan_area = "area-2"
    poller = PlanPoller(source, RecordingLog())

    update = poller.poll()

    assert update is not None
    assert update.area.external_id == "area-2"
    assert source.element_centres_args == ["area-2"]


def test_filters_are_passed_to_the_plan_lookup() -> None:
    source = StubSource()

    PlanPoller(source, RecordingLog(), area_external_id="area-9", vessel_external_id="v-9").poll()

    assert source.lookup_args == [("area-9", "v-9")]


def test_without_filters_the_whole_project_is_searched() -> None:
    source = StubSource()

    PlanPoller(source, RecordingLog()).poll()

    assert source.lookup_args == [(None, None)]


def test_following_a_plan_is_logged_with_vessel_and_area() -> None:
    source = StubSource()
    log = RecordingLog()
    poller = PlanPoller(source, log)

    poller.poll()
    source.tasks = [{"id": "t-2"}]
    poller.poll()
    source.plan_id = "p-2"
    poller.poll()

    following = [m for m in log.messages("info") if m.startswith("Following plan")]
    assert following == [
        "Following plan p-1 'Plan p-1' in Carrier/Tank 1",
        "Following plan p-2 'Plan p-2' in Carrier/Tank 1",
    ]


def test_following_an_unnamed_plan_says_so() -> None:
    source = StubSource()
    source.plan_name = None
    log = RecordingLog()

    PlanPoller(source, log).poll()

    assert "Following plan p-1 (no name) in Carrier/Tank 1" in log.messages("info")


def test_no_ready_plan_message_names_the_filter() -> None:
    source = StubSource()
    source.plan_id = None
    log = RecordingLog()

    PlanPoller(source, log).poll()
    PlanPoller(source, log, area_external_id="area-9", vessel_external_id="v-9").poll()

    assert [m for m in log.messages("info") if "No Ready plan" in m] == [
        "No Ready plan (filter: whole project) yet",
        "No Ready plan (filter: vessel v-9, area area-9) yet",
    ]


def test_unchanged_plan_is_not_returned_again_even_with_new_download_time() -> None:
    source = StubSource()
    poller = PlanPoller(source, RecordingLog())
    poller.poll()
    source.downloaded_at = "later"

    assert poller.poll() is None
    assert source.element_centres_calls == 1


def test_changed_task_content_is_returned() -> None:
    source = StubSource()
    poller = PlanPoller(source, RecordingLog())
    poller.poll()
    source.tasks = [{"id": "t-1", "kind": "region", "position3d": [1.0, 0.0, 0.0]}]

    update = poller.poll()

    assert update is not None
    assert update.plan["tasks"] == source.tasks


def test_switch_to_another_plan_is_returned() -> None:
    source = StubSource()
    poller = PlanPoller(source, RecordingLog())
    poller.poll()
    source.plan_id = "p-2"

    update = poller.poll()

    assert update is not None
    assert update.plan["planExternalId"] == "p-2"


def test_missing_ready_plan_is_logged_once_until_one_appears() -> None:
    source = StubSource()
    source.plan_id = None
    log = RecordingLog()
    poller = PlanPoller(source, log)

    assert poller.poll() is None
    assert poller.poll() is None
    source.plan_id = "p-1"
    poller.poll()
    source.plan_id = None
    poller.poll()

    assert len([m for m in log.messages("info") if "No Ready plan" in m]) == 2


def test_errors_are_logged_and_the_next_poll_retries() -> None:
    source = StubSource()
    source.fail = RuntimeError("boom")
    log = RecordingLog()
    poller = PlanPoller(source, log)

    assert poller.poll() is None
    source.fail = None
    update = poller.poll()

    assert update is not None
    assert any("boom" in m for m in log.messages("error"))


def test_repeated_identical_errors_are_logged_once() -> None:
    source = StubSource()
    source.fail = RuntimeError("down")
    log = RecordingLog()
    poller = PlanPoller(source, log)

    poller.poll()
    poller.poll()
    source.fail = RuntimeError("other")
    poller.poll()

    assert len(log.messages("error")) == 2


def test_recovery_after_errors_is_logged() -> None:
    source = StubSource()
    source.fail = RuntimeError("down")
    log = RecordingLog()
    poller = PlanPoller(source, log)
    poller.poll()
    source.fail = None

    poller.poll()

    assert any("recovered" in m for m in log.messages("info"))


def test_error_after_success_does_not_forget_the_published_plan() -> None:
    source = StubSource()
    poller = PlanPoller(source, RecordingLog())
    poller.poll()
    source.fail = RuntimeError("blip")
    poller.poll()
    source.fail = None

    assert poller.poll() is None


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


class StubSource:
    def __init__(self) -> None:
        self.plan_id: Optional[str] = "p-1"
        self.downloaded_at = "now"
        self.tasks: List[Dict[str, Any]] = [{"id": "t-1", "kind": "region"}]
        self.fail: Optional[Exception] = None
        self.plan_json_args: List[tuple] = []
        self.element_centres_calls = 0
        self.element_centres_args: List[str] = []
        self.lookup_args: List[tuple] = []
        self.plan_area = "area-1"
        self.plan_name: Optional[str] = "named"

    def latest_ready_plan(self, area_external_id: Any = None, vessel_external_id: Any = None) -> Any:
        self.lookup_args.append((area_external_id, vessel_external_id))
        if self.fail is not None:
            raise self.fail
        if self.plan_id is None:
            return None
        plan = _Plan(self.plan_id, self.plan_area)
        if self.plan_name is None:
            plan.name = None
        return plan

    def area_info(self, area_external_id: str) -> AreaInfo:
        return AreaInfo(area_external_id, "Tank 1", "v-1", "Carrier")

    def plan_json(self, plan: Any, area_name: str) -> Dict[str, Any]:
        self.plan_json_args.append((plan.external_id, area_name))
        return self.payload()

    def element_centres(self, area_external_id: str) -> List[tuple]:
        self.element_centres_calls += 1
        self.element_centres_args.append(area_external_id)
        return [(0.0, 0.0, 0.0)]

    def payload(self) -> Dict[str, Any]:
        return {"planExternalId": self.plan_id, "areaName": "Tank 1",
                "downloadedAt": self.downloaded_at, "tasks": list(self.tasks)}


class _Plan:
    def __init__(self, external_id: str, area_external_id: str) -> None:
        self.external_id = external_id
        self.name: Optional[str] = "Plan " + external_id
        self.area_external_id = area_external_id
