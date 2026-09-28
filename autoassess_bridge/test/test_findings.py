# SPDX-License-Identifier: BSD-3-Clause
"""Tests for findings parsing, buffering, merging and task planning (autoassess_bridge.findings)."""

from __future__ import annotations

import json
import math
from typing import Any, List, Optional, Tuple

import pytest

from autoassess_bridge import findings as fnd
from autoassess_bridge.findings import Finding, FindingsBuffer


# --- parsing --------------------------------------------------------------------------------


def test_parse_a_single_object() -> None:
    parsed, errors = fnd.parse_findings_json(json.dumps({"x": 1, "y": 2.5, "z": -3, "id": "f-1"}))

    assert errors == []
    assert parsed == [Finding(id="f-1", position=(1.0, 2.5, -3.0))]


def test_parse_an_array_with_all_fields() -> None:
    text = json.dumps(
        [
            {
                "x": 1,
                "y": 2,
                "z": 3,
                "id": "corr-1",
                "nx": 0,
                "ny": 0,
                "nz": 2,
                "radius": 0.4,
                "inspection_type": "ndt_thickness",
                "class": "corrosion",
                "confidence": 0.75,
                "description": "pit",
            }
        ]
    )

    parsed, errors = fnd.parse_findings_json(text)

    assert errors == []
    (f,) = parsed
    assert f.normal == (0.0, 0.0, 1.0)  # normalised
    assert f.radius_m == 0.4
    assert f.inspection_type == "ndt_thickness"
    assert f.finding_class == "corrosion"
    assert f.confidence == 0.75
    assert f.description == "pit"


def test_parse_gives_a_stable_id_when_missing() -> None:
    text = json.dumps({"x": 1, "y": 2, "z": 3, "class": "crack"})

    first, _ = fnd.parse_findings_json(text)
    second, _ = fnd.parse_findings_json(text)
    other, _ = fnd.parse_findings_json(json.dumps({"x": 9, "y": 2, "z": 3, "class": "crack"}))

    assert first[0].id == second[0].id
    assert first[0].id != other[0].id
    assert first[0].id.startswith("row-")


def test_parse_reports_bad_entries_and_keeps_good_ones() -> None:
    text = json.dumps(
        [
            {"x": 1, "y": 2, "z": 3, "id": "good"},
            {"y": 2, "z": 3},
            {"x": "not a number", "y": 2, "z": 3},
            {"x": 1, "y": 2, "z": 3, "nx": 1},
            {"x": 1, "y": 2, "z": 3, "id": "a+b"},
            {"x": 1, "y": 2, "z": 3, "confidence": 7},
            {"x": 1, "y": 2, "z": 3, "radius": -1},
            {"x": 1, "y": 2, "z": 3, "inspection_type": "sonar"},
            "not an object",
        ]
    )

    parsed, errors = fnd.parse_findings_json(text)

    assert [f.id for f in parsed] == ["good"]
    assert len(errors) == 8
    assert any("x is missing" in e for e in errors)
    assert any("nx, ny and nz" in e for e in errors)


def test_parse_rejects_non_json_and_wrong_shapes() -> None:
    for text in ["nonsense{", json.dumps("a string"), json.dumps(42)]:
        parsed, errors = fnd.parse_findings_json(text)
        assert parsed == []
        assert len(errors) == 1


def test_parse_rejects_non_finite_coordinates() -> None:
    parsed, errors = fnd.parse_findings_json('{"x": 1, "y": NaN, "z": 3}')

    assert parsed == []
    assert len(errors) == 1


# --- buffer ---------------------------------------------------------------------------------


def test_buffer_dedupes_by_id_keeping_the_first() -> None:
    buffer = FindingsBuffer()

    assert buffer.add(Finding("f-1", (1, 2, 3), finding_class="a"))
    assert not buffer.add(Finding("f-1", (9, 9, 9), finding_class="b"))

    (kept,) = buffer.snapshot()
    assert kept.finding_class == "a"
    assert len(buffer) == 1


def test_buffer_snapshot_is_a_copy_and_discard_removes_only_named_ids() -> None:
    buffer = FindingsBuffer()
    buffer.add(Finding("f-1", (1, 2, 3)))
    buffer.add(Finding("f-2", (4, 5, 6)))

    snapshot = buffer.snapshot()
    buffer.add(Finding("f-3", (7, 8, 9)))
    buffer.discard([f.id for f in snapshot])

    assert [f.id for f in snapshot] == ["f-1", "f-2"]
    assert [f.id for f in buffer.snapshot()] == ["f-3"]


def test_buffer_drops_new_findings_at_the_cap() -> None:
    buffer = FindingsBuffer(cap=2)
    assert buffer.add(Finding("f-1", (1, 2, 3)))
    assert buffer.add(Finding("f-2", (1, 2, 3)))

    assert not buffer.add(Finding("f-3", (1, 2, 3)))
    assert buffer.dropped == 1
    assert len(buffer) == 2


# --- merging --------------------------------------------------------------------------------


def test_nearby_findings_merge_into_one_cluster() -> None:
    clusters = fnd.merge_findings(
        [Finding("a", (0, 0, 0)), Finding("b", (0.3, 0, 0)), Finding("c", (10, 0, 0))],
        merge_radius_m=0.5,
    )

    assert [c.member_ids for c in clusters] == [("a", "b"), ("c",)]
    assert clusters[0].position == (0.15, 0.0, 0.0)


def test_cluster_radius_covers_every_member() -> None:
    (cluster,) = fnd.merge_findings(
        [Finding("a", (0, 0, 0), radius_m=0.2), Finding("b", (0.4, 0, 0))], merge_radius_m=0.5
    )

    assert cluster.radius_m == pytest.approx(0.2 + 0.3)  # distance to centre + default radius
    assert cluster.inspection_type == "visual"


def test_merging_prefers_ndt_and_the_highest_confidence() -> None:
    (cluster,) = fnd.merge_findings(
        [
            Finding("a", (0, 0, 0), inspection_type="ndt_thickness", confidence=0.4),
            Finding("b", (0.1, 0, 0), confidence=0.9),
        ]
    )

    assert cluster.inspection_type == "ndt_thickness"
    assert cluster.confidence == 0.9


def test_findings_facing_away_do_not_merge() -> None:
    clusters = fnd.merge_findings(
        [Finding("a", (0, 0, 0), normal=(0, 0, 1)), Finding("b", (0.1, 0, 0), normal=(0, 0, -1))],
        merge_radius_m=0.5,
    )

    assert len(clusters) == 2


def test_zero_merge_radius_disables_merging() -> None:
    clusters = fnd.merge_findings([Finding("a", (0, 0, 0)), Finding("b", (0, 0, 0))], merge_radius_m=0)

    assert len(clusters) == 2


# --- suggestion ids -------------------------------------------------------------------------


def test_suggestion_id_joins_sorted_member_ids() -> None:
    (cluster,) = fnd.merge_findings([Finding("b", (0, 0, 0)), Finding("a", (0.1, 0, 0))])

    assert fnd.finding_suggestion_id(cluster) == "finding:a+b"


def test_long_suggestion_ids_are_capped_with_a_hash() -> None:
    findings = [Finding("f-{:03d}".format(i) * 8, (i * 0.01, 0, 0)) for i in range(30)]
    (cluster,) = fnd.merge_findings(findings, merge_radius_m=5.0)

    sid = fnd.finding_suggestion_id(cluster)

    assert len(sid) <= 255
    assert sid.startswith("finding:")
    assert "+~" in sid


def test_covered_finding_ids_reads_suggestion_ids_back() -> None:
    covered = fnd.covered_finding_ids(["finding:a+b", "finding:c+~deadbeef", "other:x", None])

    assert covered == {"a", "b", "c"}


# --- planning tasks -------------------------------------------------------------------------


def test_plan_tasks_uses_the_given_normal() -> None:
    plan = fnd.plan_tasks_from_findings([Finding("a", (1, 0, 0), normal=(0, 0, 1))])

    (task,) = plan.tasks
    assert task.normal_vector == (0.0, 0.0, 1.0)
    assert task.normal_source == "given"
    assert task.position3d == (1.0, 0.0, 0.0)
    assert task.radius_m == 0.3
    assert task.inspection_type == "visual"
    assert task.suggestion_id == "finding:a"


def test_plan_tasks_falls_back_to_facing_the_centroid() -> None:
    plan = fnd.plan_tasks_from_findings(
        [Finding("a", (2, 0, 0)), Finding("b", (-2, 0, 0), normal=(0, 1, 0))],
        merge_radius_m=0.1,
    )

    by_id = {t.suggestion_id: t for t in plan.tasks}
    assert by_id["finding:a"].normal_vector == (-1.0, 0.0, 0.0)  # towards the centroid (0,0,0)
    assert by_id["finding:a"].normal_source == "centroid"
    assert by_id["finding:b"].normal_source == "given"
    assert plan.interior_point == (0.0, 0.0, 0.0)


def test_plan_tasks_uses_a_provided_interior_point() -> None:
    plan = fnd.plan_tasks_from_findings([Finding("a", (0, 0, 5))], interior_point=(0.0, 0.0, 0.0))

    (task,) = plan.tasks
    assert task.normal_vector == (0.0, 0.0, -1.0)


def test_plan_tasks_for_a_finding_at_the_centroid_uses_up() -> None:
    plan = fnd.plan_tasks_from_findings([Finding("a", (1, 1, 1))], interior_point=(1.0, 1.0, 1.0))

    assert plan.tasks[0].normal_vector == (0.0, 0.0, 1.0)


def test_plan_tasks_skips_findings_already_in_the_plan() -> None:
    plan = fnd.plan_tasks_from_findings(
        [Finding("a", (0, 0, 0)), Finding("b", (9, 9, 9))],
        existing_suggestion_ids=["finding:a"],
    )

    assert [t.suggestion_id for t in plan.tasks] == ["finding:b"]
    assert plan.already_in_plan == 1


def test_plan_tasks_skips_a_cluster_whose_suggestion_id_exists() -> None:
    plan = fnd.plan_tasks_from_findings(
        [Finding("a", (0, 0, 0)), Finding("b", (0.1, 0, 0))],
        existing_suggestion_ids=["finding:a+b"],
    )

    assert plan.tasks == []
    assert plan.already_in_plan == 2


def test_plan_tasks_with_no_findings_is_empty() -> None:
    plan = fnd.plan_tasks_from_findings([])

    assert plan.tasks == []
    assert plan.already_in_plan == 0


def test_task_normals_are_unit_length() -> None:
    plan = fnd.plan_tasks_from_findings([Finding("a", (3, 4, 0))], interior_point=(0.0, 0.0, 0.0))

    normal = plan.tasks[0].normal_vector
    assert math.sqrt(sum(v * v for v in normal)) == pytest.approx(1.0)
    assert normal == pytest.approx((-0.6, -0.8, 0.0))
