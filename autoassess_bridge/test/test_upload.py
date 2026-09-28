# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the mission upload to CDF (autoassess_bridge.upload)."""

from __future__ import annotations

import os
import re
import uuid
from datetime import date, datetime, timezone
from typing import Any, Dict, List

import pytest
from conftest import SPACE, FakeWriteClient, RecordingLog

from autoassess_bridge import upload
from autoassess_bridge.upload import MissionContext, MissionFile, MissionUploader

AREA = "area-1"
CTX = MissionContext(area_external_id=AREA, mission_id="mission-20260101T000000Z", plan_external_id="plan-1")
CAMPAIGN_UUID = uuid.UUID("11111111-2222-4333-8444-555555555555")
CAMPAIGN_XID = "result-11111111-2222-4333-8444-555555555555"
RESULT_VIEW = {"space": SPACE, "externalId": "InspectionResultView", "version": "1", "type": "view"}
FILE_VIEW = {"space": "cdf_cdm", "externalId": "CogniteFile", "version": "v1", "type": "view"}


# --- naming and tagging ---------------------------------------------------------------------


def test_mission_id_is_the_utc_start_time() -> None:
    start = datetime(2026, 3, 4, 5, 6, 7, tzinfo=timezone.utc).timestamp()

    assert upload.mission_id_for(start) == "mission-20260304T050607Z"


def test_file_external_id_follows_the_uidss_shape() -> None:
    xid = upload.file_external_id(AREA, CTX.mission_id, "/data/run 1/map.ply")

    assert re.match(r"^area-1-file-[0-9a-f]{12}-map\.ply$", xid)


def test_file_external_id_is_stable_per_mission_and_path() -> None:
    first = upload.file_external_id(AREA, "mission-a", "/data/map.ply")

    assert upload.file_external_id(AREA, "mission-a", "/data/map.ply") == first
    assert upload.file_external_id(AREA, "mission-b", "/data/map.ply") != first
    assert upload.file_external_id(AREA, "mission-a", "/other/map.ply") != first


def test_file_external_id_sanitises_and_truncates_the_name() -> None:
    xid = upload.file_external_id(AREA, CTX.mission_id, "/d/" + "a b" * 200 + ".ply")

    assert len(xid) == 255
    assert " " not in xid
    assert xid.endswith("a_b.ply")


def test_ply_tags_match_uidss_plus_plan_and_mission() -> None:
    tags = upload.file_tags(CTX, MissionFile("/d/map.ply", "ply"))

    assert tags == ["autoassess", "ply_mesh", "area:area-1", "plan:plan-1", "mission:" + CTX.mission_id]


def test_pcd_tags_include_the_label() -> None:
    tags = upload.file_tags(CTX, MissionFile("/d/hull_scan.pcd", "pcd", "Hull Scan"))

    assert tags == [
        "autoassess",
        "pcd_pointcloud",
        "area:area-1",
        "label:Hull Scan",
        "plan:plan-1",
        "mission:" + CTX.mission_id,
    ]


def test_tags_omit_the_plan_when_unknown() -> None:
    ctx = MissionContext(AREA, "mission-x", None)

    assert upload.file_tags(ctx, MissionFile("/d/m.ply", "ply")) == [
        "autoassess",
        "ply_mesh",
        "area:area-1",
        "mission:mission-x",
    ]


def test_pcd_label_is_the_title_cased_stem() -> None:
    assert upload.pcd_label("/d/hull_scan_left.pcd") == "Hull Scan Left"


# --- collecting files -----------------------------------------------------------------------


def test_collect_files_takes_the_mesh_then_the_mission_dir_by_path(tmp_path: Any) -> None:
    mesh = _touch(tmp_path / "mesh" / "gbplanner.ply")
    mission = tmp_path / "mission"
    b = _touch(mission / "b_scan.pcd")
    a = _touch(mission / "sub" / "a.ply")
    other = _touch(mission / "notes.csv")

    files, skipped = upload.collect_files(mesh, str(mission))

    assert files == [
        MissionFile(mesh, "ply"),
        MissionFile(b, "pcd", "B Scan"),
        MissionFile(a, "ply"),
    ]
    assert skipped == [other]


def test_collect_files_does_not_repeat_a_mesh_inside_the_mission_dir(tmp_path: Any) -> None:
    mesh = _touch(tmp_path / "map.ply")

    files, _ = upload.collect_files(mesh, str(tmp_path))

    assert files == [MissionFile(mesh, "ply")]


def test_collect_files_without_inputs_is_empty() -> None:
    assert upload.collect_files(None, None) == ([], [])


# --- uploading ------------------------------------------------------------------------------


def test_upload_creates_the_campaign_like_uidss_plus_created_by(tmp_path: Any) -> None:
    client = FakeWriteClient()

    _uploader(client).upload(CTX, [MissionFile(_touch(tmp_path / "m.ply"), "ply")])

    assert client.instances.applied[0] == {
        "space": SPACE,
        "externalId": CAMPAIGN_XID,
        "instanceType": "node",
        "sources": [
            {
                "properties": {
                    "area": {"space": SPACE, "externalId": AREA},
                    "campaignDate": "2026-05-06",
                    "status": "InProgress",
                    "cdfFileIds": [],
                    "pcdFileIds": [],
                    "pcdFileLabels": [],
                    "createdBy": "autoassess_bridge",
                },
                "source": RESULT_VIEW,
            }
        ],
    }


def test_upload_creates_cognite_file_nodes_and_uploads_by_instance_id(tmp_path: Any) -> None:
    client = FakeWriteClient()
    ply = _touch(tmp_path / "m.ply")

    _uploader(client).upload(CTX, [MissionFile(ply, "ply")])

    file_node = client.instances.applied[1]
    xid = upload.file_external_id(AREA, CTX.mission_id, ply)
    assert file_node["space"] == SPACE
    assert file_node["externalId"] == xid
    assert file_node["sources"][0]["source"] == FILE_VIEW
    props = file_node["sources"][0]["properties"]
    assert props["name"] == "m.ply"
    assert props["mimeType"] == "application/octet-stream"
    assert props["tags"] == upload.file_tags(CTX, MissionFile(ply, "ply"))
    assert client.files.uploads == [(ply, (SPACE, xid))]


def test_upload_sets_file_ids_then_completes(tmp_path: Any) -> None:
    client = FakeWriteClient()
    ply = _touch(tmp_path / "m.ply")
    pcd = _touch(tmp_path / "hull.pcd")

    result = _uploader(client).upload(CTX, [MissionFile(ply, "ply"), MissionFile(pcd, "pcd", "Hull")])

    ply_id, pcd_id = _file_id(client, ply), _file_id(client, pcd)
    assert _campaign_writes(client) == [
        _created_props(),
        {"cdfFileIds": [ply_id], "pcdFileIds": [pcd_id], "pcdFileLabels": ["Hull"]},
        {"status": "Complete"},
    ]
    assert result.campaign_external_id == CAMPAIGN_XID
    assert result.cdf_file_ids == [ply_id]
    assert result.pcd_file_ids == [pcd_id]
    assert result.complete
    assert result.failed == []


def test_upload_keeps_the_campaign_in_progress_when_a_file_fails(tmp_path: Any) -> None:
    client = FakeWriteClient()
    ply = _touch(tmp_path / "m.ply")
    pcd = _touch(tmp_path / "hull.pcd")
    client.files.fail_paths.add(pcd)

    result = _uploader(client).upload(CTX, [MissionFile(ply, "ply"), MissionFile(pcd, "pcd", "Hull")])

    assert _campaign_writes(client)[-1] == {
        "cdfFileIds": [_file_id(client, ply)],
        "pcdFileIds": [],
        "pcdFileLabels": [],
    }
    assert not result.complete
    assert [path for path, _ in result.failed] == [pcd]


def test_retry_reuses_the_campaign_and_skips_uploaded_files(tmp_path: Any) -> None:
    client = FakeWriteClient()
    uploader = _uploader(client)
    ply = _touch(tmp_path / "m.ply")
    pcd = _touch(tmp_path / "hull.pcd")
    files = [MissionFile(ply, "ply"), MissionFile(pcd, "pcd", "Hull")]
    client.files.fail_paths.add(pcd)
    uploader.upload(CTX, files)
    client.files.fail_paths.clear()

    result = uploader.upload(CTX, files)

    assert [path for path, _ in client.files.uploads] == [ply, pcd]
    assert sum(1 for w in _campaign_writes(client) if w.get("status") == "InProgress") == 1
    assert result.campaign_external_id == CAMPAIGN_XID
    assert result.complete


def test_upload_content_goes_to_an_existing_node_without_reapplying_it(tmp_path: Any) -> None:
    client = FakeWriteClient()
    ply = _touch(tmp_path / "m.ply")
    xid = upload.file_external_id(AREA, CTX.mission_id, ply)
    client.files.register(SPACE, xid, uploaded=False)

    _uploader(client).upload(CTX, [MissionFile(ply, "ply")])

    assert [n["externalId"] for n in client.instances.applied].count(xid) == 0
    assert client.files.uploads == [(ply, (SPACE, xid))]


def test_upload_without_files_writes_nothing() -> None:
    client = FakeWriteClient()

    with pytest.raises(ValueError):
        _uploader(client).upload(CTX, [])

    assert client.instances.calls == []


def test_upload_logs_each_uploaded_file(tmp_path: Any) -> None:
    client = FakeWriteClient()
    log = RecordingLog()

    _uploader(client, log).upload(CTX, [MissionFile(_touch(tmp_path / "m.ply"), "ply")])

    assert any("m.ply" in m for m in log.messages("info"))


# --- helpers --------------------------------------------------------------------------------


def _uploader(client: FakeWriteClient, log: Any = None) -> MissionUploader:
    return MissionUploader(
        client,
        log=log or RecordingLog(),
        today=lambda: date(2026, 5, 6),
        new_uuid=lambda: CAMPAIGN_UUID,
    )


def _touch(path: Any) -> str:
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    with open(str(path), "w") as handle:
        handle.write("x")
    return str(path)


def _file_id(client: FakeWriteClient, path: str) -> int:
    return client.files.store[(SPACE, upload.file_external_id(AREA, CTX.mission_id, path))].id


def _campaign_writes(client: FakeWriteClient) -> List[Dict[str, Any]]:
    return [
        n["sources"][0]["properties"]
        for n in client.instances.applied
        if n["externalId"] == CAMPAIGN_XID
    ]


def _created_props() -> Dict[str, Any]:
    return {
        "area": {"space": SPACE, "externalId": AREA},
        "campaignDate": "2026-05-06",
        "status": "InProgress",
        "cdfFileIds": [],
        "pcdFileIds": [],
        "pcdFileLabels": [],
        "createdBy": "autoassess_bridge",
    }
