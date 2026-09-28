# SPDX-License-Identifier: BSD-3-Clause
"""Poll CDF for the area's Ready plan and report it only when its content changed."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from autoassess_bridge.plan import Vec3


@dataclass(frozen=True)
class PlanUpdate:
    plan: Dict[str, Any]  # plan.json dict (see cdf.PlanSource.plan_json)
    element_centres: List[Vec3]


class PlanPoller:
    """Change detection over `PlanSource`.

    A plan is new when its plan.json (minus `downloadedAt`) differs from the last one returned,
    so task edits are caught even though they do not touch the plan node. Errors are logged
    (once per distinct message) and swallowed; the next poll simply retries.
    `log` needs info(), warning() and error().
    """

    def __init__(self, source: Any, area_external_id: str, log: Any) -> None:
        self._source = source
        self._area = area_external_id
        self._log = log
        self._last_key: Optional[str] = None
        self._reported_no_plan = False
        self._last_error: Optional[str] = None

    def poll(self) -> Optional[PlanUpdate]:
        try:
            update = self._poll()
        except Exception as exc:  # noqa: BLE001 - keep polling whatever CDF or the network does
            message = "{}: {}".format(type(exc).__name__, exc)
            if message != self._last_error:
                self._log.error("Polling AutoAssess plans failed (will retry): " + message)
                self._last_error = message
            return None
        if self._last_error is not None:
            self._log.info("Polling AutoAssess plans recovered")
            self._last_error = None
        return update

    def _poll(self) -> Optional[PlanUpdate]:
        plan = self._source.latest_ready_plan(self._area)
        if plan is None:
            if not self._reported_no_plan:
                self._log.info("No Ready plan for area {} yet".format(self._area))
                self._reported_no_plan = True
            return None
        self._reported_no_plan = False
        payload = self._source.plan_json(plan, self._source.area_name(self._area))
        key = json.dumps(
            {k: v for k, v in payload.items() if k != "downloadedAt"}, sort_keys=True
        )
        if key == self._last_key:
            return None
        centres = self._source.element_centres(self._area)
        self._last_key = key
        return PlanUpdate(plan=payload, element_centres=centres)
