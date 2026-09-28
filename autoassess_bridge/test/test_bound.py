# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the global-bound retry logic (autoassess_bridge.bound)."""

from __future__ import annotations

from typing import List, Tuple

from conftest import RecordingLog

from autoassess_bridge.bound import GlobalBoundSetter, ServiceUnavailable

BOUNDS = ((0.0, 0.0, 0.0), (1.0, 2.0, 3.0))


def test_nothing_is_called_without_pending_bounds() -> None:
    service = StubService()
    setter = GlobalBoundSetter(service, RecordingLog())

    setter.tick()

    assert service.calls == []


def test_pending_bounds_are_sent_once_on_success() -> None:
    service = StubService()
    log = RecordingLog()
    setter = GlobalBoundSetter(service, log)
    setter.set_pending(BOUNDS)

    setter.tick()
    setter.tick()

    assert service.calls == [BOUNDS]
    assert not setter.pending
    assert any("Global bound set" in m for m in log.messages("info"))


def test_unavailable_service_is_retried_on_the_next_tick() -> None:
    service = StubService()
    service.unavailable = 2
    setter = GlobalBoundSetter(service, RecordingLog())
    setter.set_pending(BOUNDS)

    setter.tick()
    setter.tick()
    assert setter.pending
    setter.tick()

    assert service.calls == [BOUNDS]
    assert not setter.pending


def test_unavailable_service_warning_is_logged_once_per_pending_bound() -> None:
    service = StubService()
    service.unavailable = 3
    log = RecordingLog()
    setter = GlobalBoundSetter(service, log)
    setter.set_pending(BOUNDS)

    for _ in range(3):
        setter.tick()

    assert len(log.messages("warning")) == 1


def test_rejected_bound_is_logged_and_not_retried() -> None:
    service = StubService()
    service.accept = False
    log = RecordingLog()
    setter = GlobalBoundSetter(service, log)
    setter.set_pending(BOUNDS)

    setter.tick()
    setter.tick()

    assert service.calls == [BOUNDS]
    assert any("rejected" in m for m in log.messages("warning"))
    assert not setter.pending


def test_failed_call_is_logged_and_not_retried() -> None:
    service = StubService()
    service.error = RuntimeError("service exploded")
    log = RecordingLog()
    setter = GlobalBoundSetter(service, log)
    setter.set_pending(BOUNDS)

    setter.tick()
    setter.tick()

    assert len(service.calls) == 1
    assert any("service exploded" in m for m in log.messages("warning"))


def test_newer_bounds_replace_pending_ones() -> None:
    service = StubService()
    service.unavailable = 1
    setter = GlobalBoundSetter(service, RecordingLog())
    setter.set_pending(BOUNDS)
    setter.tick()
    newer = ((1.0, 1.0, 1.0), (2.0, 2.0, 2.0))

    setter.set_pending(newer)
    setter.tick()

    assert service.calls == [newer]


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


class StubService:
    def __init__(self) -> None:
        self.unavailable = 0
        self.accept = True
        self.error = None
        self.calls: List[Tuple[tuple, tuple]] = []

    def __call__(self, bounds: Tuple[tuple, tuple]) -> Tuple[bool, str]:
        if self.unavailable:
            self.unavailable -= 1
            raise ServiceUnavailable("not up yet")
        self.calls.append(bounds)
        if self.error is not None:
            raise self.error
        return self.accept, "min [0, 0, 0] max [1, 2, 3]"
