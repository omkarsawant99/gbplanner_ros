# SPDX-License-Identifier: BSD-3-Clause
"""Poll CDF for the area's Ready plan and report it only when its content changed."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from autoassess_bridge.cdf import AreaInfo
from autoassess_bridge.plan import Vec3


@dataclass(frozen=True)
class PlanUpdate:
    plan: Dict[str, Any]  # plan.json dict (see cdf.PlanSource.plan_json)
    element_centres: List[Vec3]
    area: AreaInfo  # the plan's area (the plan decides the area, not a parameter)


class PlanPoller:
    """Change detection over `PlanSource`.

    A plan is new when its plan.json (minus `downloadedAt`) differs from the last one returned,
    so task edits are caught even though they do not touch the plan node. Errors are logged
    (once per distinct message) and swallowed; the next poll simply retries.
    Without filters it follows the newest Ready plan of the whole project; `area_external_id`
    and/or `vessel_external_id` restrict that. `log` needs info(), warning() and error().
    """

    def __init__(
        self,
        source: Any,
        log: Any,
        area_external_id: Optional[str] = None,
        vessel_external_id: Optional[str] = None,
    ) -> None:
        self._source = source
        self._area = area_external_id or None
        self._vessel = vessel_external_id or None
        self._log = log
        self._last_key: Optional[str] = None
        self._last_plan_id: Optional[str] = None
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
        plan = self._source.latest_ready_plan(
            area_external_id=self._area, vessel_external_id=self._vessel
        )
        if plan is None:
            if not self._reported_no_plan:
                self._log.info("No Ready plan (filter: {}) yet".format(self.filter_description))
                self._reported_no_plan = True
            return None
        self._reported_no_plan = False
        area = self._source.area_info(plan.area_external_id)
        payload = self._source.plan_json(plan, area.name)
        key = json.dumps(
            {k: v for k, v in payload.items() if k != "downloadedAt"}, sort_keys=True
        )
        if key == self._last_key:
            return None
        centres = self._source.element_centres(plan.area_external_id)
        self._last_key = key
        if plan.external_id != self._last_plan_id:
            self._log.info(
                "Following plan {} {} in {}/{}".format(
                    plan.external_id,
                    "'{}'".format(plan.name) if plan.name else "(no name)",
                    area.vessel_name or area.vessel_external_id or "?",
                    area.name or area.external_id,
                )
            )
            self._last_plan_id = plan.external_id
        return PlanUpdate(plan=payload, element_centres=centres, area=area)

    @property
    def filter_description(self) -> str:
        parts = []
        if self._vessel:
            parts.append("vessel " + self._vessel)
        if self._area:
            parts.append("area " + self._area)
        return ", ".join(parts) if parts else "whole project"
