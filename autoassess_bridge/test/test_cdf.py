# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the read-only CDF access (autoassess_bridge.cdf)."""

from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest
from conftest import SPACE, FakeClient, FakeNode

from autoassess_bridge import cdf

PLAN_VIEW = ("InspectionPlanView", "4")
TASK_VIEW = ("InspectionTaskView", "1")
ELEMENT_VIEW = ("StructuralElementView", "1")
AREA_VIEW = ("AreaView", "4")
FIXED_NOW = datetime(2026, 1, 2, 3, 4, 5, tzinfo=timezone.utc)


def test_token_url_uses_azure_ad_for_uuid_tenant() -> None:
    tenant = "12345678-abcd-4ef0-9abc-1234567890ab"

    assert cdf.token_url(tenant) == (
        "https://login.microsoftonline.com/{}/oauth2/v2.0/token".format(tenant)
    )


def test_token_url_uses_cognite_idp_for_named_tenant() -> None:
    assert cdf.token_url("some-org") == "https://auth.cognite.com/oauth2/token"


def test_make_client_configures_azure_ad_credentials() -> None:
    env = _env(tenant="12345678-abcd-4ef0-9abc-1234567890ab")

    client = cdf.make_client(env)

    assert client.config.project == "proj"
    assert client.config.base_url == "https://clu.cognitedata.com"
    assert client.config.credentials.token_url.startswith("https://login.microsoftonline.com/")
    assert client.config.credentials.scopes == ["https://clu.cognitedata.com/.default"]


def test_make_client_requests_no_scope_for_cognite_idp() -> None:
    client = cdf.make_client(_env(tenant="some-org"))

    assert client.config.credentials.token_url == "https://auth.cognite.com/oauth2/token"
    # None, not []: with cognite-sdk 7.x (requests-oauthlib) an empty list makes oauthlib raise
    # 'Scope has changed from "" to ""' on the Cognite IdP token response.
    assert client.config.credentials.scopes is None


def test_make_client_names_every_missing_variable() -> None:
    env = _env()
    del env["COGNITE_PROJECT"]
    env["COGNITE_CLIENT_SECRET"] = ""

    with pytest.raises(ValueError) as excinfo:
        cdf.make_client(env)

    assert "COGNITE_PROJECT" in str(excinfo.value)
    assert "COGNITE_CLIENT_SECRET" in str(excinfo.value)
    assert "COGNITE_CLUSTER" not in str(excinfo.value)


def test_latest_ready_plan_filters_on_area_and_not_deleted() -> None:
    client = FakeClient()

    cdf.PlanSource(client).latest_ready_plan("area-1")

    (call,) = client.instances.calls_to("list")
    assert call["instance_type"] == "node"
    assert call["sources"][0].external_id == "InspectionPlanView"
    assert call["sources"][0].version == "4"
    assert call["limit"] == 1000
    assert call["filter"] == {
        "and": [
            {
                "equals": {
                    "property": [SPACE, "InspectionPlanContainer", "area"],
                    "value": {"space": SPACE, "externalId": "area-1"},
                }
            },
            {"not": {"exists": {"property": [SPACE, "InspectionPlanContainer", "deletedAt"]}}},
        ]
    }


def test_latest_ready_plan_returns_none_without_ready_plans() -> None:
    client = FakeClient()
    client.instances.add(PLAN_VIEW, _plan_node("p-draft", status="Draft"))
    client.instances.add(PLAN_VIEW, _plan_node("p-bad", status="Bogus"))

    assert cdf.PlanSource(client).latest_ready_plan("area-1") is None


def test_latest_ready_plan_picks_most_recently_updated_ready_plan() -> None:
    client = FakeClient()
    client.instances.add(PLAN_VIEW, _plan_node("p-old", created=1, updated=10))
    client.instances.add(PLAN_VIEW, _plan_node("p-new", created=2, updated=30))
    client.instances.add(PLAN_VIEW, _plan_node("p-mid", created=3, updated=20))
    client.instances.add(PLAN_VIEW, _plan_node("p-draft", status="Draft", created=4, updated=99))

    plan = cdf.PlanSource(client).latest_ready_plan("area-1")

    assert plan is not None
    assert plan.external_id == "p-new"


def test_latest_ready_plan_breaks_update_ties_by_creation_time() -> None:
    client = FakeClient()
    client.instances.add(PLAN_VIEW, _plan_node("p-a", created=5, updated=10))
    client.instances.add(PLAN_VIEW, _plan_node("p-b", created=7, updated=10))

    plan = cdf.PlanSource(client).latest_ready_plan("area-1")

    assert plan is not None
    assert plan.external_id == "p-b"


def test_latest_ready_plan_maps_plan_fields() -> None:
    client = FakeClient()
    client.instances.add(PLAN_VIEW, _plan_node("p-1", name="", description="desc", map_id=None))

    plan = cdf.PlanSource(client).latest_ready_plan("area-1")

    assert plan == cdf.Plan(
        space=SPACE,
        external_id="p-1",
        name=None,
        description="desc",
        area_external_id="area-1",
        map_external_id=None,
        status="Ready",
        created_time=0,
        last_updated_time=0,
    )


def test_plan_json_lists_tasks_of_the_plan() -> None:
    client = FakeClient()

    _source(client).plan_json(_plan(), "Tank 1")

    (call,) = client.instances.calls_to("list")
    assert call["sources"][0].external_id == "InspectionTaskView"
    assert call["limit"] == 10000
    assert call["filter"] == {
        "equals": {
            "property": [SPACE, "InspectionTaskContainer", "plan"],
            "value": {"space": SPACE, "externalId": "p-1"},
        }
    }


def test_plan_json_matches_dss_plan_download_shape() -> None:
    client = FakeClient()
    client.instances.add(TASK_VIEW, _task_node("t-el", taskType="element",
                                               inspectionType="ndt_thickness",
                                               targetElement={"space": SPACE, "externalId": "e-1"}))
    client.instances.add(TASK_VIEW, _task_node("t-reg", taskType="region",
                                               position3d=[1, 2, 3], normalVector=[0, 1, 0],
                                               radiusM=0.5))
    client.instances.add(ELEMENT_VIEW, _element_node("e-1", "manhole", (4, 5, 6)), listable=False)

    payload = _source(client).plan_json(_plan(map_id="map-1"), "Tank 1")

    assert list(payload) == [
        "planExternalId", "name", "description", "areaExternalId", "areaName",
        "mapExternalId", "downloadedAt", "tasks",
    ]
    assert payload == {
        "planExternalId": "p-1",
        "name": "Plan 1",
        "description": None,
        "areaExternalId": "area-1",
        "areaName": "Tank 1",
        "mapExternalId": "map-1",
        "downloadedAt": "2026-01-02T03:04:05+00:00",
        "tasks": [
            {
                "id": "t-el",
                "kind": "element",
                "inspectionType": "ndt_thickness",
                "targetElement": {"externalId": "e-1", "type": "manhole",
                                  "center": [4.0, 5.0, 6.0]},
            },
            {
                "id": "t-reg",
                "kind": "region",
                "inspectionType": "visual",
                "position3d": [1.0, 2.0, 3.0],
                "normalVector": [0.0, 1.0, 0.0],
                "radiusM": 0.5,
            },
        ],
    }
    assert list(payload["tasks"][1]) == [
        "id", "kind", "inspectionType", "position3d", "normalVector", "radiusM",
    ]


def test_plan_json_retrieves_element_targets_in_one_deduplicated_call() -> None:
    client = FakeClient()
    for tid, eid in [("t-1", "e-1"), ("t-2", "e-2"), ("t-3", "e-1")]:
        client.instances.add(TASK_VIEW, _task_node(tid, taskType="element",
                                                   targetElement={"space": SPACE, "externalId": eid}))
    client.instances.add(ELEMENT_VIEW, _element_node("e-1", "wall", (0, 0, 0)), listable=False)
    client.instances.add(ELEMENT_VIEW, _element_node("e-2", "wall", (1, 1, 1)), listable=False)

    payload = _source(client).plan_json(_plan(), "Tank 1")

    (call,) = client.instances.calls_to("retrieve")
    assert call["nodes"] == [(SPACE, "e-1"), (SPACE, "e-2")]
    assert call["sources"][0].external_id == "StructuralElementView"
    assert [t["targetElement"]["externalId"] for t in payload["tasks"]] == ["e-1", "e-2", "e-1"]


def test_plan_json_skips_element_retrieve_without_element_tasks() -> None:
    client = FakeClient()
    client.instances.add(TASK_VIEW, _task_node("t-reg", taskType="region", position3d=[0, 0, 0]))

    _source(client).plan_json(_plan(), "Tank 1")

    assert client.instances.calls_to("retrieve") == []


def test_plan_json_applies_dss_fallbacks() -> None:
    client = FakeClient()
    client.instances.add(TASK_VIEW, _task_node("t-none"))
    client.instances.add(TASK_VIEW, _task_node("t-odd", taskType="weird", inspectionType="xray",
                                               position3d=[1, 2], normalVector=[1, 2, 3, 4],
                                               radiusM=None))
    client.instances.add(TASK_VIEW, _task_node("t-missing", taskType="element",
                                               targetElement={"space": SPACE, "externalId": "gone"}))
    client.instances.add(TASK_VIEW, _task_node("t-noref", taskType="element"))
    client.instances.add(TASK_VIEW, _task_node("t-elem", taskType="element",
                                               targetElement={"space": SPACE, "externalId": "e-x"}))
    client.instances.add(ELEMENT_VIEW, _element_node("e-x", "beam", None), listable=False)

    tasks = _source(client).plan_json(_plan(), "Tank 1")["tasks"]

    assert tasks == [
        {"id": "t-none", "kind": "region", "inspectionType": "visual"},
        {"id": "t-odd", "kind": "region", "inspectionType": "visual"},
        {"id": "t-missing", "kind": "element", "inspectionType": "visual"},
        {"id": "t-noref", "kind": "element", "inspectionType": "visual"},
        {"id": "t-elem", "kind": "element", "inspectionType": "visual",
         "targetElement": {"externalId": "e-x", "type": "longitudinal",
                           "center": [0.0, 0.0, 0.0]}},
    ]


def test_plan_json_serialises_like_dss() -> None:
    client = FakeClient()
    client.instances.add(TASK_VIEW, _task_node("t-reg", taskType="region", position3d=[1, 2, 3]))

    payload = _source(client).plan_json(_plan(), "Tank 1")

    assert json.loads(cdf.dumps_plan(payload)) == payload
    assert cdf.dumps_plan(payload).startswith('{\n  "planExternalId": "p-1",')


def test_area_name_reads_area_view() -> None:
    client = FakeClient()
    client.instances.add(AREA_VIEW, FakeNode("area-1", AREA_VIEW, {"name": "Tank 1"}))

    name = cdf.PlanSource(client).area_name("area-1")

    assert name == "Tank 1"
    (call,) = client.instances.calls_to("retrieve")
    assert call["nodes"] == [(SPACE, "area-1")]
    assert call["sources"][0].external_id == "AreaView"
    assert call["sources"][0].version == "4"


def test_area_name_raises_for_unknown_area() -> None:
    with pytest.raises(LookupError):
        cdf.PlanSource(FakeClient()).area_name("nope")


def test_element_centres_lists_elements_of_the_area() -> None:
    client = FakeClient()
    client.instances.add(ELEMENT_VIEW, _element_node("e-1", "wall", (1, 2, 3)))
    client.instances.add(ELEMENT_VIEW, _element_node("e-2", "wall", (-1, 0, 5)))

    centres = cdf.PlanSource(client).element_centres("area-1")

    assert centres == [(1.0, 2.0, 3.0), (-1.0, 0.0, 5.0)]
    (call,) = client.instances.calls_to("list")
    assert call["sources"][0].external_id == "StructuralElementView"
    assert call["limit"] is None
    assert call["filter"] == {
        "equals": {
            "property": [SPACE, "StructuralElementContainer", "area"],
            "value": {"space": SPACE, "externalId": "area-1"},
        }
    }


def test_element_centres_skips_elements_without_a_full_centre() -> None:
    client = FakeClient()
    client.instances.add(ELEMENT_VIEW, FakeNode("e-1", ELEMENT_VIEW, {"centerX": 1.0}))

    assert cdf.PlanSource(client).element_centres("area-1") == []


def test_plan_source_uses_configured_space() -> None:
    client = FakeClient()

    cdf.PlanSource(client, space="other").latest_ready_plan("area-1")

    (call,) = client.instances.calls_to("list")
    assert call["sources"][0].space == "other"
    assert call["filter"]["and"][0]["equals"]["property"][0] == "other"
    assert call["filter"]["and"][0]["equals"]["value"]["space"] == "other"


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _env(tenant: str = "some-org") -> dict:
    return {
        "COGNITE_PROJECT": "proj",
        "COGNITE_CLUSTER": "clu",
        "COGNITE_TENANT_ID": tenant,
        "COGNITE_CLIENT_ID": "id",
        "COGNITE_CLIENT_SECRET": "secret",
    }


def _source(client: FakeClient) -> cdf.PlanSource:
    return cdf.PlanSource(client, now=lambda: FIXED_NOW)


def _plan(map_id: object = None) -> cdf.Plan:
    return cdf.Plan(
        space=SPACE, external_id="p-1", name="Plan 1", description=None,
        area_external_id="area-1", map_external_id=map_id, status="Ready",
        created_time=0, last_updated_time=0,
    )


def _plan_node(external_id: str, status: str = "Ready", created: int = 0, updated: int = 0,
               name: str = "Plan", description: object = None, map_id: object = None) -> FakeNode:
    props = {"area": {"space": SPACE, "externalId": "area-1"}, "status": status, "name": name,
             "description": description}
    if map_id is not None:
        props["map"] = {"space": SPACE, "externalId": map_id}
    return FakeNode(external_id, PLAN_VIEW, props, created_time=created, last_updated_time=updated)


def _task_node(external_id: str, **props: object) -> FakeNode:
    props.setdefault("plan", {"space": SPACE, "externalId": "p-1"})
    return FakeNode(external_id, TASK_VIEW, dict(props))


def _element_node(external_id: str, element_type: str, centre: object) -> FakeNode:
    props = {"elementType": element_type, "area": {"space": SPACE, "externalId": "area-1"}}
    if centre is not None:
        props.update(centerX=centre[0], centerY=centre[1], centerZ=centre[2])
    return FakeNode(external_id, ELEMENT_VIEW, props)
