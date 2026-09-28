# SPDX-License-Identifier: BSD-3-Clause
"""Tests for the ROS-free plan geometry (autoassess_bridge.plan)."""

from __future__ import annotations

import math

import pytest

from autoassess_bridge import plan


def test_task_target_of_element_task_is_element_centre() -> None:
    task = {"kind": "element", "targetElement": {"center": [1, 2, 3]}, "position3d": [9, 9, 9]}

    assert plan.task_target(task) == (1.0, 2.0, 3.0)


def test_task_target_of_region_task_is_position3d() -> None:
    assert plan.task_target({"kind": "region", "position3d": [4, 5, 6]}) == (4.0, 5.0, 6.0)


def test_task_target_is_none_without_a_point() -> None:
    assert plan.task_target({"kind": "element"}) is None
    assert plan.task_target({"kind": "region"}) is None


def test_region_pose_backs_off_along_normal_and_looks_back_at_target() -> None:
    task = {"kind": "region", "position3d": [1, 2, 3], "normalVector": [0, 2, 0]}

    position, orientation = plan.task_pose(task, standoff_m=0.5)

    assert position == pytest.approx((1.0, 2.5, 3.0))
    assert _body_x(orientation) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_region_pose_handles_oblique_normals() -> None:
    task = {"kind": "region", "position3d": [0, 0, 0], "normalVector": [-1, 0, -1]}

    position, orientation = plan.task_pose(task, standoff_m=math.sqrt(2))

    assert position == pytest.approx((-1.0, 0.0, -1.0))
    s = 1 / math.sqrt(2)
    assert _body_x(orientation) == pytest.approx((s, 0.0, s), abs=1e-9)
    assert _norm(orientation) == pytest.approx(1.0)


def test_region_pose_without_normal_hovers_above_and_looks_down() -> None:
    position, orientation = plan.task_pose({"kind": "region", "position3d": [1, 1, 1],
                                            "normalVector": [0, 0, 0]})

    assert position == pytest.approx((1.0, 1.0, 1.8))
    assert _body_x(orientation) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_element_pose_hovers_standoff_above_centre_looking_down() -> None:
    task = {"kind": "element", "targetElement": {"center": [2, 3, 4]}}

    position, orientation = plan.task_pose(task, standoff_m=1.0)

    assert position == pytest.approx((2.0, 3.0, 5.0))
    assert _body_x(orientation) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_task_pose_raises_without_target() -> None:
    with pytest.raises(ValueError):
        plan.task_pose({"id": "t-1", "kind": "region"})


def test_plan_to_poses_keeps_task_order_and_reports_skipped_tasks() -> None:
    payload = {"tasks": [
        {"id": "a", "kind": "region", "position3d": [0, 0, 0], "normalVector": [1, 0, 0]},
        {"id": "b", "kind": "element"},
        {"id": "c", "kind": "element", "targetElement": {"center": [5, 5, 5]}},
    ]}

    poses, skipped = plan.plan_to_poses(payload, standoff_m=1.0)

    assert [p[0] for p in poses] == [pytest.approx((1.0, 0.0, 0.0)),
                                     pytest.approx((5.0, 5.0, 6.0))]
    assert skipped == ["b"]


def test_plan_to_poses_of_empty_plan_is_empty() -> None:
    assert plan.plan_to_poses({"tasks": []}) == ([], [])


def test_area_bounds_is_the_padded_box_around_the_centres() -> None:
    bounds = plan.area_bounds([(0, 5, 1), (2, -1, 3), (1, 1, 2)], margin_m=0.5)

    assert bounds == ((-0.5, -1.5, 0.5), (2.5, 5.5, 3.5))


def test_area_bounds_defaults_to_a_30_cm_margin() -> None:
    lo, hi = plan.area_bounds([(0, 0, 0)])

    assert lo == pytest.approx((-0.3, -0.3, -0.3))
    assert hi == pytest.approx((0.3, 0.3, 0.3))


def test_area_bounds_of_no_centres_is_none() -> None:
    assert plan.area_bounds([]) is None


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _body_x(q: tuple) -> tuple:
    """Rotate the body +x axis by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    return (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w))


def _norm(q: tuple) -> float:
    return math.sqrt(sum(c * c for c in q))
