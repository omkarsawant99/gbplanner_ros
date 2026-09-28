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


def test_clusters_collect_the_member_classes() -> None:
    (cluster,) = fnd.merge_findings(
        [
            Finding("a", (0, 0, 0), finding_class="crack"),
            Finding("b", (0.1, 0, 0), finding_class="corrosion"),
            Finding("c", (0.2, 0, 0)),
        ]
    )

    assert cluster.classes == ("corrosion", "crack")


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


