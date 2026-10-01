#!/usr/bin/env python3
"""Classify inspection viewpoints between detected compartment wall planes."""

import math


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _direction(orientation):
    x, y, z, w = orientation
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid wall or target orientation")
    return (1 - 2*(y*y + z*z)/norm,
            2*(x*y + w*z)/norm, 2*(x*z - w*y)/norm)


def _wall_side(position, wall, forward):
    origin, orientation = wall
    if not all(math.isfinite(value) for value in origin):
        raise ValueError("Invalid wall position")
    normal = _direction(orientation)
    alignment = _dot(normal, forward)
    if abs(alignment) < 0.2:
        raise ValueError("Detected wall normal is perpendicular to compartment progress")
    if alignment < 0:
        normal = tuple(-value for value in normal)
    distance = _dot(tuple(p - c for p, c in zip(position, origin)), normal)
    return distance, normal


def target_in_compartment(position, orientation, entry_wall, exit_wall,
                          forward, wall_tolerance=0.05):
    """Test membership after the entry plane and before the exit plane.

    Walls are (position, quaternion) tuples; body +x is the detected normal.
    Its sign is made consistent with mission progress. The first compartment
    has no entry plane and the final one has no exit plane. Near a wall, a
    target facing into the compartment is deferred to the other side. A view
    parallel to the wall uses the signed position for an unambiguous result.
    Missing required walls must be rejected by the caller before this function.
    """
    if not all(math.isfinite(value) for value in (*position, *forward)):
        raise ValueError("Invalid target position or mission direction")
    length = math.sqrt(_dot(forward, forward))
    if length < 1e-9:
        raise ValueError("Compartment progress direction is zero")
    forward = tuple(value / length for value in forward)
    view = _direction(orientation)
    for wall, is_entry in ((entry_wall, True), (exit_wall, False)):
        if wall is None:
            continue
        distance, normal = _wall_side(position, wall, forward)
        # Positive inward distance lies inside this compartment.
        inward = distance if is_entry else -distance
        if inward < -wall_tolerance:
            return False
        if abs(distance) <= wall_tolerance:
            facing_inward = _dot(view, normal) * (1 if is_entry else -1)
            if facing_inward > 0.2:
                return False
            if abs(facing_inward) <= 0.2 and (inward < 0 or (is_entry and inward == 0)):
                return False
    return True
