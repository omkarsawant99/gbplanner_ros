# SPDX-License-Identifier: BSD-3-Clause
"""Guess when a gbplanner mission has ended. gbplanner has no "mission complete" signal of its own.

A mission starts with the first planned path (/gbplanner_path). It has ended when the robot
was sent home (/gbplanner_is_homing true), no new path has come for `quiet_s`, and the robot
stands still (odometry speed at most `max_speed`, sample newer than `max_odometry_age_s`).
Each mission end is reported once; the next path starts the next mission. ROS-free and
thread-safe (the node feeds it from subscriber callbacks and checks it from its main loop).
"""

from __future__ import annotations

import threading
from typing import Optional

DEFAULT_QUIET_S = 10.0
DEFAULT_MAX_SPEED = 0.1
DEFAULT_MAX_ODOMETRY_AGE_S = 2.0


class MissionEndDetector:
    def __init__(
        self,
        quiet_s: float = DEFAULT_QUIET_S,
        max_speed: float = DEFAULT_MAX_SPEED,
        max_odometry_age_s: float = DEFAULT_MAX_ODOMETRY_AGE_S,
    ) -> None:
        self._quiet_s = quiet_s
        self._max_speed = max_speed
        self._max_odometry_age_s = max_odometry_age_s
        self._lock = threading.Lock()
        self._started_at: Optional[float] = None
        self._last_path_at: Optional[float] = None
        self._homing = False
        self._speed: Optional[float] = None
        self._speed_at: Optional[float] = None

    @property
    def mission_started_at(self) -> Optional[float]:
        with self._lock:
            return self._started_at

    def on_path(self, now: float) -> None:
        with self._lock:
            if self._started_at is None:
                self._started_at = now
            self._last_path_at = now

    def on_homing(self, homing: bool, now: float) -> None:
        with self._lock:
            self._homing = bool(homing)

    def on_speed(self, speed: float, now: float) -> None:
        with self._lock:
            self._speed = speed
            self._speed_at = now

    def check(self, now: float) -> Optional[float]:
        """The start time of the mission that has just ended, or None."""
        with self._lock:
            if self._started_at is None or not self._homing or self._last_path_at is None:
                return None
            if now - self._last_path_at < self._quiet_s:
                return None
            if self._speed is None or self._speed_at is None:
                return None
            if now - self._speed_at > self._max_odometry_age_s or self._speed > self._max_speed:
                return None
            started_at = self._started_at
            self._reset_locked()
            return started_at

    def reset(self) -> None:
        with self._lock:
            self._reset_locked()

    def _reset_locked(self) -> None:
        self._started_at = None
        self._last_path_at = None
        self._homing = False
