# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the mission-end heuristic (autoassess_bridge.mission_end)."""

from __future__ import annotations

from autoassess_bridge.mission_end import MissionEndDetector

QUIET_S = 10.0
MAX_SPEED = 0.1


def test_no_mission_end_before_any_path() -> None:
    detector = _detector()
    detector.on_homing(True, 1.0)
    detector.on_speed(0.0, 100.0)

    assert detector.check(100.0) is None


def test_reports_mission_end_after_path_homing_quiet_and_still() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_speed(0.02, 60.0)

    assert detector.check(60.0) == 5.0


def test_reports_each_mission_end_only_once() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_speed(0.0, 60.0)
    detector.check(60.0)
    detector.on_speed(0.0, 70.0)

    assert detector.check(70.0) is None


def test_no_mission_end_without_homing() -> None:
    detector = _detector()
    detector.on_path(5.0)
    detector.on_path(50.0)
    detector.on_speed(0.0, 60.0)

    assert detector.check(60.0) is None


def test_no_mission_end_while_paths_are_recent() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_speed(0.0, 59.0)

    assert detector.check(59.0) is None


def test_no_mission_end_while_moving() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_speed(0.5, 60.0)

    assert detector.check(60.0) is None


def test_no_mission_end_without_odometry() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)

    assert detector.check(60.0) is None


def test_no_mission_end_when_odometry_is_stale() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_speed(0.0, 55.0)

    assert detector.check(60.0) is None


def test_leaving_homing_cancels_the_mission_end() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_homing(False, 51.0)
    detector.on_speed(0.0, 70.0)

    assert detector.check(70.0) is None


def test_next_mission_starts_with_the_next_path() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.on_speed(0.0, 60.0)
    detector.check(60.0)

    _fly_home(detector, start=100.0, last_path=150.0)
    detector.on_speed(0.0, 161.0)

    assert detector.check(161.0) == 100.0


def test_mission_started_at_tracks_the_current_mission() -> None:
    detector = _detector()
    assert detector.mission_started_at is None

    detector.on_path(5.0)
    detector.on_path(8.0)

    assert detector.mission_started_at == 5.0


def test_reset_forgets_the_current_mission() -> None:
    detector = _detector()
    _fly_home(detector, start=5.0, last_path=50.0)
    detector.reset()
    detector.on_speed(0.0, 60.0)

    assert detector.mission_started_at is None
    assert detector.check(60.0) is None


def _detector() -> MissionEndDetector:
    return MissionEndDetector(quiet_s=QUIET_S, max_speed=MAX_SPEED, max_odometry_age_s=2.0)


def _fly_home(detector: MissionEndDetector, start: float, last_path: float) -> None:
    detector.on_path(start)
    detector.on_homing(False, start)
    detector.on_homing(True, last_path)
    detector.on_path(last_path)
