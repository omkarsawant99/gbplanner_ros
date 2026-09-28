# SPDX-License-Identifier: BSD-3-Clause
"""Read-only access to AutoAssess inspection plans in Cognite Data Fusion (CDF).

Only `data_modeling.instances.list` and `data_modeling.instances.retrieve` are called. The plan
JSON built here has the same shape as the file written by the AutoAssess ground-station CLI
(`dss plan download`), so consumers can treat both the same.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

from cognite.client import CogniteClient
from cognite.client.config import ClientConfig, global_config
from cognite.client.credentials import OAuthClientCredentials
from cognite.client.data_classes.data_modeling import ViewId

from autoassess_bridge.plan import Vec3

SPACE = "autoassess"

# Views as (externalId, version) and containers (for filter paths) of the AutoAssess data model.
VESSEL_VIEW = ("VesselView", "2")
AREA_VIEW = ("AreaView", "4")
INSPECTION_PLAN_VIEW = ("InspectionPlanView", "4")
INSPECTION_TASK_VIEW = ("InspectionTaskView", "1")
STRUCTURAL_ELEMENT_VIEW = ("StructuralElementView", "1")
AREA_CONTAINER = "AreaContainer"
INSPECTION_PLAN_CONTAINER = "InspectionPlanContainer"
INSPECTION_TASK_CONTAINER = "InspectionTaskContainer"
STRUCTURAL_ELEMENT_CONTAINER = "StructuralElementContainer"

ENV_VARS = (
    "COGNITE_PROJECT",
    "COGNITE_CLUSTER",
    "COGNITE_TENANT_ID",
    "COGNITE_CLIENT_ID",
    "COGNITE_CLIENT_SECRET",
)

_UUID_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$", re.IGNORECASE
)
_VALID_TASK_KINDS = frozenset({"element", "region"})
_VALID_INSPECTION_TYPES = frozenset({"visual", "ndt_thickness"})
_VALID_ELEMENT_TYPES = frozenset({"manhole", "longitudinal", "wall", "compartment"})
_VALID_STATUSES = frozenset({"Draft", "Ready", "Complete"})


def token_url(tenant_id: str) -> str:
    """Azure AD token endpoint for a UUID tenant, otherwise the Cognite IdP."""
    if _UUID_RE.match(tenant_id):
        return "https://login.microsoftonline.com/{}/oauth2/v2.0/token".format(tenant_id)
    return "https://auth.cognite.com/oauth2/token"


def make_client(env: Mapping[str, str]) -> CogniteClient:
    """Client-credentials CogniteClient from the COGNITE_* variables in `env`."""
    missing = [name for name in ENV_VARS if not env.get(name)]
    if missing:
        raise ValueError("Missing CDF environment variables: {}".format(", ".join(missing)))
    global_config.disable_pypi_version_check = True
    base_url = "https://{}.cognitedata.com".format(env["COGNITE_CLUSTER"])
    tenant = env["COGNITE_TENANT_ID"]
    credentials = OAuthClientCredentials(
        token_url=token_url(tenant),
        client_id=env["COGNITE_CLIENT_ID"],
        client_secret=env["COGNITE_CLIENT_SECRET"],
        # Cognite IdP: request no scope. None rather than [] because cognite-sdk 7.x
        # (requests-oauthlib) otherwise raises 'Scope has changed from "" to ""'.
        scopes=["{}/.default".format(base_url)] if _UUID_RE.match(tenant) else None,
    )
    return CogniteClient(
        ClientConfig(
            client_name="autoassess-bridge",
            project=env["COGNITE_PROJECT"],
            base_url=base_url,
            credentials=credentials,
        )
    )


def dumps_plan(plan: Dict[str, Any]) -> str:
    """Serialise a plan exactly like `dss plan download` writes plan.json."""
    return json.dumps(plan, indent=2)


@dataclass(frozen=True)
class Plan:
    space: str
    external_id: str
    name: Optional[str]
    description: Optional[str]
    area_external_id: str
    map_external_id: Optional[str]
    status: str
    created_time: int
    last_updated_time: int


@dataclass(frozen=True)
class AreaInfo:
    external_id: str
    name: str
    vessel_external_id: Optional[str]
    vessel_name: Optional[str]


class PlanSource:
    """Reads plans, tasks, and area data for one AutoAssess space."""

    def __init__(
        self,
        client: CogniteClient,
        space: str = SPACE,
        now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
    ) -> None:
        self._instances = client.data_modeling.instances
        self._space = space
        self._now = now

    def latest_ready_plan(
        self,
        area_external_id: Optional[str] = None,
        vessel_external_id: Optional[str] = None,
    ) -> Optional[Plan]:
        """The most recently updated non-deleted Ready plan in the project, if any.

        Optionally only plans of one area and/or of the areas of one vessel.
        """
        plan_prop = lambda name: self._prop(INSPECTION_PLAN_CONTAINER, name)  # noqa: E731
        filters: List[Dict[str, Any]] = [
            {"equals": {"property": plan_prop("status"), "value": "Ready"}},
            {"not": {"exists": {"property": plan_prop("deletedAt")}}},
        ]
        if area_external_id:
            filters.append({"equals": {"property": plan_prop("area"), "value": self._ref(area_external_id)}})
        if vessel_external_id:
            areas = self._vessel_areas(vessel_external_id)
            if not areas:
                return None
            filters.append(
                {"in": {"property": plan_prop("area"), "values": [self._ref(a) for a in areas]}}
            )
        items = self._instances.list(
            instance_type="node",
            sources=[self._view(INSPECTION_PLAN_VIEW)],
            filter={"and": filters},
            limit=None,
        )
        plans = [self._map_plan(item) for item in items if item.instance_type == "node"]
        ready = [p for p in plans if p.status == "Ready"]
        if not ready:
            return None
        return max(ready, key=lambda p: (p.last_updated_time, p.created_time))

    def area_info(self, area_external_id: str) -> AreaInfo:
        """The area's name and its vessel (externalId and name, None if not set)."""
        result = self._instances.retrieve(
            nodes=[(self._space, area_external_id)], sources=[self._view(AREA_VIEW)]
        )
        if not result.nodes:
            raise LookupError("Area '{}' not found in space '{}'".format(area_external_id, self._space))
        props = self._props(result.nodes[0], AREA_VIEW)
        vessel_ref = props.get("vessel") or {}
        vessel_id = vessel_ref.get("externalId") if isinstance(vessel_ref, dict) else None
        vessel_name: Optional[str] = None
        if vessel_id:
            vessels = self._instances.retrieve(
                nodes=[(self._space, str(vessel_id))], sources=[self._view(VESSEL_VIEW)]
            )
            if vessels.nodes:
                vessel_name = str(self._props(vessels.nodes[0], VESSEL_VIEW).get("name", "")) or None
        return AreaInfo(
            area_external_id,
            str(props.get("name", "")),
            str(vessel_id) if vessel_id else None,
            vessel_name,
        )

    def plan_json(self, plan: Plan, area_name: str) -> Dict[str, Any]:
        """The plan and its tasks as the plan.json dict written by `dss plan download`."""
        items = self._instances.list(
            instance_type="node",
            sources=[self._view(INSPECTION_TASK_VIEW)],
            filter={
                "equals": {
                    "property": self._prop(INSPECTION_TASK_CONTAINER, "plan"),
                    "value": self._ref(plan.external_id),
                }
            },
            limit=10000,
        )
        task_props = [
            (item.external_id, self._props(item, INSPECTION_TASK_VIEW))
            for item in items
            if item.instance_type == "node"
        ]
        elements = self._element_targets(task_props)
        return {
            "planExternalId": plan.external_id,
            "name": plan.name,
            "description": plan.description,
            "areaExternalId": plan.area_external_id,
            "areaName": area_name,
            "mapExternalId": plan.map_external_id,
            "downloadedAt": self._now().isoformat(),
            "tasks": [_task_to_dict(eid, props, elements) for eid, props in task_props],
        }

    def area_name(self, area_external_id: str) -> str:
        result = self._instances.retrieve(
            nodes=[(self._space, area_external_id)], sources=[self._view(AREA_VIEW)]
        )
        if not result.nodes:
            raise LookupError("Area '{}' not found in space '{}'".format(area_external_id, self._space))
        return str(self._props(result.nodes[0], AREA_VIEW).get("name", ""))

    def element_centres(self, area_external_id: str) -> List[Vec3]:
        """Centres of the area's structural elements (elements without a full centre skipped)."""
        items = self._instances.list(
            instance_type="node",
            sources=[self._view(STRUCTURAL_ELEMENT_VIEW)],
            filter={
                "equals": {
                    "property": self._prop(STRUCTURAL_ELEMENT_CONTAINER, "area"),
                    "value": self._ref(area_external_id),
                }
            },
            limit=None,
        )
        centres: List[Vec3] = []
        for item in items:
            if item.instance_type != "node":
                continue
            props = self._props(item, STRUCTURAL_ELEMENT_VIEW)
            xyz = [props.get(k) for k in ("centerX", "centerY", "centerZ")]
            if all(v is not None for v in xyz):
                centres.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))
        return centres

    # -- helpers -----------------------------------------------------------------------------

    def _vessel_areas(self, vessel_external_id: str) -> List[str]:
        items = self._instances.list(
            instance_type="node",
            sources=[self._view(AREA_VIEW)],
            filter={
                "and": [
                    {
                        "equals": {
                            "property": self._prop(AREA_CONTAINER, "vessel"),
                            "value": self._ref(vessel_external_id),
                        }
                    },
                    {"not": {"exists": {"property": self._prop(AREA_CONTAINER, "deletedAt")}}},
                ]
            },
            limit=None,
        )
        return [item.external_id for item in items if item.instance_type == "node"]

    def _element_targets(
        self, task_props: List[Tuple[str, Dict[str, Any]]]
    ) -> Dict[str, Dict[str, Any]]:
        """All element targets of the tasks in one retrieve, keyed by element externalId."""
        ids: List[str] = []
        for _, props in task_props:
            eid = _element_ref(props)
            if eid and eid not in ids:
                ids.append(eid)
        if not ids:
            return {}
        result = self._instances.retrieve(
            nodes=[(self._space, eid) for eid in ids],
            sources=[self._view(STRUCTURAL_ELEMENT_VIEW)],
        )
        targets: Dict[str, Dict[str, Any]] = {}
        for node in result.nodes:
            ep = self._props(node, STRUCTURAL_ELEMENT_VIEW)
            etype = str(ep.get("elementType", "longitudinal"))
            targets[node.external_id] = {
                "externalId": node.external_id,
                "type": etype if etype in _VALID_ELEMENT_TYPES else "longitudinal",
                "center": [_float(ep.get(k)) for k in ("centerX", "centerY", "centerZ")],
            }
        return targets

    def _map_plan(self, item: Any) -> Plan:
        props = self._props(item, INSPECTION_PLAN_VIEW)
        area_ref = props.get("area") or {}
        map_ref = props.get("map") or {}
        status = str(props.get("status", "Draft"))
        name = props.get("name")
        description = props.get("description")
        return Plan(
            space=getattr(item, "space", self._space),
            external_id=item.external_id,
            name=str(name) if name else None,
            description=str(description) if description else None,
            area_external_id=(
                str(area_ref.get("externalId", "")) if isinstance(area_ref, dict) else ""
            ),
            map_external_id=(
                str(map_ref["externalId"])
                if isinstance(map_ref, dict) and map_ref.get("externalId")
                else None
            ),
            status=status if status in _VALID_STATUSES else "Draft",
            created_time=int(getattr(item, "created_time", 0) or 0),
            last_updated_time=int(getattr(item, "last_updated_time", 0) or 0),
        )

    def _view(self, view: Tuple[str, str]) -> ViewId:
        return ViewId(self._space, view[0], view[1])

    def _prop(self, container: str, prop: str) -> List[str]:
        return [self._space, container, prop]

    def _ref(self, external_id: str) -> Dict[str, str]:
        return {"space": self._space, "externalId": external_id}

    def _props(self, item: Any, view: Tuple[str, str]) -> Dict[str, Any]:
        props = getattr(item, "properties", None) or {}
        return dict(props.get(self._view(view)) or {})


def _element_ref(task_props: Dict[str, Any]) -> str:
    if _kind(task_props) != "element":
        return ""
    ref = task_props.get("targetElement") or {}
    return str(ref.get("externalId", "")) if isinstance(ref, dict) else ""


def _kind(task_props: Dict[str, Any]) -> str:
    kind = str(task_props.get("taskType", "region"))
    return kind if kind in _VALID_TASK_KINDS else "region"


def _vec3(value: Any) -> Optional[List[float]]:
    if value and len(value) == 3:
        return [float(value[0]), float(value[1]), float(value[2])]
    return None


def _float(value: Any) -> float:
    return 0.0 if value is None else float(value)


def _task_to_dict(
    external_id: str, props: Dict[str, Any], elements: Dict[str, Dict[str, Any]]
) -> Dict[str, Any]:
    kind = _kind(props)
    itype = str(props.get("inspectionType", "visual"))
    task: Dict[str, Any] = {
        "id": external_id,
        "kind": kind,
        "inspectionType": itype if itype in _VALID_INSPECTION_TYPES else "visual",
    }
    if kind == "element":
        target = elements.get(_element_ref(props))
        if target is not None:
            task["targetElement"] = dict(target, center=list(target["center"]))
    else:
        position = _vec3(props.get("position3d"))
        normal = _vec3(props.get("normalVector"))
        if position:
            task["position3d"] = position
        if normal:
            task["normalVector"] = normal
        if props.get("radiusM") is not None:
            task["radiusM"] = float(props["radiusM"])
    return task
