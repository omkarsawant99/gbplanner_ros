# SPDX-License-Identifier: BSD-3-Clause
"""Plan geometry: inspection poses for plan.json tasks and the padded area box. ROS-free."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # (x, y, z, w)
Pose = Tuple[Vec3, Quaternion]
Bounds = Tuple[Vec3, Vec3]  # (min corner, max corner)

DEFAULT_STANDOFF_M = 0.8
DEFAULT_BOUND_MARGIN_M = 0.3
_UP: Vec3 = (0.0, 0.0, 1.0)


def task_target(task: Dict[str, Any]) -> Optional[Vec3]:
    """The point a plan.json task inspects: the element centre, or the region's position3d."""
    if task.get("kind") == "element":
        point = (task.get("targetElement") or {}).get("center")
    else:
        point = task.get("position3d")
    if point is None:
        return None
    return (float(point[0]), float(point[1]), float(point[2]))


def task_pose(task: Dict[str, Any], standoff_m: float = DEFAULT_STANDOFF_M) -> Pose:
    """Where to hover to inspect a task, and the attitude that points the body +x axis at it.

    Region tasks back off `standoff_m` along their normalVector; element tasks (no normal) hover
    `standoff_m` above the element centre looking straight down. Raises ValueError for a task
    without a target point.
    """
    target = task_target(task)
    if target is None:
        raise ValueError("Task {} has no target (element centre / position3d)".format(task.get("id")))
    normal = task.get("normalVector") if task.get("kind") == "region" else None
    n = _unit(normal) or _UP
    position = (
        target[0] + n[0] * standoff_m,
        target[1] + n[1] * standoff_m,
        target[2] + n[2] * standoff_m,
    )
    return position, _look_quaternion((-n[0], -n[1], -n[2]))


def plan_to_poses(
    plan: Dict[str, Any], standoff_m: float = DEFAULT_STANDOFF_M
) -> Tuple[List[Pose], List[str]]:
    """Poses for the plan's tasks in task order, plus the ids of tasks that have no target."""
    poses: List[Pose] = []
    skipped: List[str] = []
    for task in plan.get("tasks") or []:
        if task_target(task) is None:
            skipped.append(str(task.get("id")))
        else:
            poses.append(task_pose(task, standoff_m))
    return poses, skipped


def area_bounds(
    centres: Iterable[Sequence[float]], margin_m: float = DEFAULT_BOUND_MARGIN_M
) -> Optional[Bounds]:
    """Axis-aligned box around the given points, padded by `margin_m`; None when there are none."""
    points = [(float(p[0]), float(p[1]), float(p[2])) for p in centres]
    if not points:
        return None
    lo = tuple(min(p[i] for p in points) - margin_m for i in range(3))
    hi = tuple(max(p[i] for p in points) + margin_m for i in range(3))
    return (lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])


def _look_quaternion(look: Vec3) -> Quaternion:
    """Level attitude (no roll) whose body +x axis points along `look` (REP-103: x fwd, z up).

    q = qz(yaw) * qy(-pitch), with pitch positive looking up.
    """
    horizontal = math.hypot(look[0], look[1])
    pitch = math.atan2(look[2], horizontal)
    yaw = math.atan2(look[1], look[0]) if horizontal > 1e-9 else 0.0
    sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
    sy, cy = math.sin(-pitch / 2), math.cos(-pitch / 2)
    return (-sz * sy, cz * sy, sz * cy, cz * cy)


def _unit(v: Optional[Sequence[float]]) -> Optional[Vec3]:
    if v is None or len(v) != 3:
        return None
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    length = math.sqrt(x * x + y * y + z * z)
    if length < 1e-9:
        return None
    return (x / length, y / length, z / length)
