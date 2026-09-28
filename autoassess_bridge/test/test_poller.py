# SPDX-License-Identifier: BSD-3-Clause
"""Tests for change detection (autoassess_bridge.poller)."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from conftest import RecordingLog

from autoassess_bridge.poller import PlanPoller, PlanUpdate


def test_first_poll_returns_the_plan_and_area_centres() -> None:
    source = StubSource()
    poller = PlanPoller(source, "area-1", RecordingLog())

    update = poller.poll()

    assert update == PlanUpdate(plan=source.payload(), element_centres=[(0.0, 0.0, 0.0)])
    assert source.plan_json_args == [("p-1", "Tank 1")]


def test_unchanged_plan_is_not_returned_again_even_with_new_download_time() -> None:
    source = StubSource()
    poller = PlanPoller(source, "area-1", RecordingLog())
    poller.poll()
    source.downloaded_at = "later"

    assert poller.poll() is None
    assert source.element_centres_calls == 1


def test_changed_task_content_is_returned() -> None:
    source = StubSource()
    poller = PlanPoller(source, "area-1", RecordingLog())
    poller.poll()
    source.tasks = [{"id": "t-1", "kind": "region", "position3d": [1.0, 0.0, 0.0]}]

    update = poller.poll()

    assert update is not None
    assert update.plan["tasks"] == source.tasks


def test_switch_to_another_plan_is_returned() -> None:
    source = StubSource()
    poller = PlanPoller(source, "area-1", RecordingLog())
    poller.poll()
    source.plan_id = "p-2"

    update = poller.poll()

    assert update is not None
    assert update.plan["planExternalId"] == "p-2"


def test_missing_ready_plan_is_logged_once_until_one_appears() -> None:
    source = StubSource()
    source.plan_id = None
    log = RecordingLog()
    poller = PlanPoller(source, "area-1", log)

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
    poller = PlanPoller(source, "area-1", log)

    assert poller.poll() is None
    source.fail = None
    update = poller.poll()

    assert update is not None
    assert any("boom" in m for m in log.messages("error"))


def test_repeated_identical_errors_are_logged_once() -> None:
    source = StubSource()
    source.fail = RuntimeError("down")
    log = RecordingLog()
    poller = PlanPoller(source, "area-1", log)

    poller.poll()
    poller.poll()
    source.fail = RuntimeError("other")
    poller.poll()

    assert len(log.messages("error")) == 2


def test_recovery_after_errors_is_logged() -> None:
    source = StubSource()
    source.fail = RuntimeError("down")
    log = RecordingLog()
    poller = PlanPoller(source, "area-1", log)
    poller.poll()
    source.fail = None

    poller.poll()

    assert any("recovered" in m for m in log.messages("info"))


def test_error_after_success_does_not_forget_the_published_plan() -> None:
    source = StubSource()
    poller = PlanPoller(source, "area-1", RecordingLog())
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

    def latest_ready_plan(self, area_external_id: str) -> Any:
        if self.fail is not None:
            raise self.fail
        assert area_external_id == "area-1"
        return None if self.plan_id is None else _Plan(self.plan_id)

    def area_name(self, area_external_id: str) -> str:
        assert area_external_id == "area-1"
        return "Tank 1"

    def plan_json(self, plan: Any, area_name: str) -> Dict[str, Any]:
        self.plan_json_args.append((plan.external_id, area_name))
        return self.payload()

    def element_centres(self, area_external_id: str) -> List[tuple]:
        self.element_centres_calls += 1
        return [(0.0, 0.0, 0.0)]

    def payload(self) -> Dict[str, Any]:
        return {"planExternalId": self.plan_id, "areaName": "Tank 1",
                "downloadedAt": self.downloaded_at, "tasks": list(self.tasks)}


class _Plan:
    def __init__(self, external_id: str) -> None:
        self.external_id = external_id
