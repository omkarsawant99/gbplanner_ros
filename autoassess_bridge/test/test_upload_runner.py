# SPDX-License-Identifier: BSD-3-Clause
"""Tests for mesh export and the upload run with its status messages (autoassess_bridge.upload)."""

from __future__ import annotations

import os
import threading
from typing import Any, Dict, List, Optional

import pytest
from conftest import RecordingLog

from autoassess_bridge import upload
from autoassess_bridge.upload import MissionContext, MissionFile, MissionUploadRunner, UploadResult

CTX = MissionContext("area-1", "mission-20260101T000000Z", "plan-1")


# --- export_mesh ----------------------------------------------------------------------------


def test_export_mesh_returns_the_file_once_it_was_rewritten(tmp_path: Any) -> None:
    mesh = str(tmp_path / "mesh.ply")
    _write(mesh, "old")
    os.utime(mesh, (1.0, 1.0))
    calls: List[str] = []

    def generate() -> None:
        calls.append("generate")
        _write(mesh, "new")

    assert upload.export_mesh(generate, mesh, timeout_s=1.0) == mesh
    assert calls == ["generate"]


def test_export_mesh_waits_for_a_file_written_after_the_call(tmp_path: Any) -> None:
    mesh = str(tmp_path / "mesh.ply")
    clock = _Clock()

    def sleep(seconds: float) -> None:
        clock.now += seconds
        if clock.now >= 0.6:
            _write(mesh, "ply")

    path = upload.export_mesh(lambda: None, mesh, timeout_s=5.0, clock=clock, sleep=sleep)

    assert path == mesh


def test_export_mesh_times_out_when_the_file_is_not_updated(tmp_path: Any) -> None:
    mesh = str(tmp_path / "mesh.ply")
    _write(mesh, "old")
    clock = _Clock()

    def sleep(seconds: float) -> None:
        clock.now += seconds

    with pytest.raises(TimeoutError) as excinfo:
        upload.export_mesh(lambda: None, mesh, timeout_s=3.0, clock=clock, sleep=sleep)

    assert "mesh_filename" in str(excinfo.value)


# --- MissionUploadRunner --------------------------------------------------------------------


def test_run_exports_uploads_and_publishes_progress(tmp_path: Any) -> None:
    mesh = _write(str(tmp_path / "mesh.ply"), "ply")
    uploader = _FakeUploader()
    statuses: List[Dict[str, Any]] = []
    runner = _runner(uploader, statuses, export=lambda: mesh)

    ok, message = runner.run(CTX)

    assert ok
    assert "result-1" in message
    assert uploader.calls == [[MissionFile(mesh, "ply")]]
    assert [s["state"] for s in statuses] == ["exporting_mesh", "uploading", "complete"]
    final = statuses[-1]
    assert final["missionId"] == CTX.mission_id
    assert final["areaExternalId"] == "area-1"
    assert final["planExternalId"] == "plan-1"
    assert final["campaignExternalId"] == "result-1"
    assert final["cdfFileIds"] == [11]
    assert final["pcdFileIds"] == []
    assert final["failedFiles"] == []
    assert "updatedAt" in final


def test_run_without_mesh_export_uploads_the_mission_dir(tmp_path: Any) -> None:
    pcd = _write(str(tmp_path / "scan.pcd"), "pcd")
    uploader = _FakeUploader()
    statuses: List[Dict[str, Any]] = []
    runner = _runner(uploader, statuses, export=None, mission_dir=str(tmp_path))

    ok, _ = runner.run(CTX)

    assert ok
    assert uploader.calls == [[MissionFile(pcd, "pcd", "Scan")]]
    assert [s["state"] for s in statuses] == ["uploading", "complete"]


def test_run_reports_skipped_files(tmp_path: Any) -> None:
    _write(str(tmp_path / "scan.pcd"), "pcd")
    notes = _write(str(tmp_path / "notes.txt"), "x")
    statuses: List[Dict[str, Any]] = []

    _runner(_FakeUploader(), statuses, export=None, mission_dir=str(tmp_path)).run(CTX)

    assert statuses[-1]["skippedFiles"] == [notes]


def test_run_fails_without_uploading_when_the_mesh_export_fails() -> None:
    uploader = _FakeUploader()
    statuses: List[Dict[str, Any]] = []

    def export() -> str:
        raise TimeoutError("mesh not written")

    ok, message = _runner(uploader, statuses, export=export).run(CTX)

    assert not ok
    assert "mesh not written" in message
    assert uploader.calls == []
    assert statuses[-1]["state"] == "failed"
    assert "mesh not written" in statuses[-1]["message"]


def test_run_fails_when_there_is_nothing_to_upload(tmp_path: Any) -> None:
    uploader = _FakeUploader()
    statuses: List[Dict[str, Any]] = []

    ok, message = _runner(uploader, statuses, export=None, mission_dir=str(tmp_path)).run(CTX)

    assert not ok
    assert "Nothing to upload" in message
    assert uploader.calls == []
    assert statuses[-1]["state"] == "failed"


def test_run_reports_partial_uploads_as_failed(tmp_path: Any) -> None:
    mesh = _write(str(tmp_path / "mesh.ply"), "ply")
    uploader = _FakeUploader(failed=[(mesh, "boom")])
    statuses: List[Dict[str, Any]] = []

    ok, message = _runner(uploader, statuses, export=lambda: mesh).run(CTX)

    assert not ok
    assert "result-1" in message
    assert statuses[-1]["state"] == "failed"
    assert statuses[-1]["failedFiles"] == [{"path": mesh, "error": "boom"}]


def test_run_reports_upload_exceptions_as_failed(tmp_path: Any) -> None:
    mesh = _write(str(tmp_path / "mesh.ply"), "ply")
    uploader = _FakeUploader(raises=RuntimeError("CDF down"))
    statuses: List[Dict[str, Any]] = []
    log = RecordingLog()

    ok, message = _runner(uploader, statuses, export=lambda: mesh, log=log).run(CTX)

    assert not ok
    assert "CDF down" in message
    assert statuses[-1]["state"] == "failed"
    assert any("CDF down" in m for m in log.messages("error"))


def test_run_refuses_a_second_upload_while_one_is_running(tmp_path: Any) -> None:
    mesh = _write(str(tmp_path / "mesh.ply"), "ply")
    entered, release = threading.Event(), threading.Event()

    def export() -> str:
        entered.set()
        release.wait(5.0)
        return mesh

    runner = _runner(_FakeUploader(), [], export=export)
    first = threading.Thread(target=runner.run, args=(CTX,))
    first.start()
    entered.wait(5.0)

    ok, message = runner.run(CTX)
    release.set()
    first.join(5.0)

    assert not ok
    assert "already running" in message


def test_idle_status_has_no_mission() -> None:
    status = upload.status_message("idle", None)

    assert status["state"] == "idle"
    assert status["missionId"] is None
    assert status["campaignExternalId"] is None


# --- helpers --------------------------------------------------------------------------------


class _FakeUploader:
    def __init__(self, failed: Optional[List[Any]] = None, raises: Optional[Exception] = None) -> None:
        self.calls: List[List[MissionFile]] = []
        self._failed = failed or []
        self._raises = raises

    def upload(self, ctx: MissionContext, files: List[MissionFile]) -> UploadResult:
        self.calls.append(list(files))
        if self._raises is not None:
            raise self._raises
        return UploadResult(
            campaign_external_id="result-1",
            cdf_file_ids=[11],
            pcd_file_ids=[],
            pcd_file_labels=[],
            failed=list(self._failed),
        )


class _Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _runner(
    uploader: Any,
    statuses: List[Dict[str, Any]],
    export: Any,
    mission_dir: Optional[str] = None,
    log: Any = None,
) -> MissionUploadRunner:
    return MissionUploadRunner(
        uploader,
        export_mesh=export,
        mission_dir=mission_dir,
        publish_status=statuses.append,
        log=log or RecordingLog(),
    )


def _write(path: str, text: str) -> str:
    with open(path, "w") as handle:
        handle.write(text)
    return path
