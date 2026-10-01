import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from compartments import target_in_compartment


def wall(x, yaw=0, y=0):
    return ((x, y, 0.9), (0, 0, math.sin(yaw/2), math.cos(yaw/2)))


FORWARD = (1, 0, 0)
SIDE_VIEW = (0, 0, math.sin(math.pi/4), math.cos(math.pi/4))


def test_live_target_is_before_detected_exit_despite_old_box_cutoff():
    position = (4.13162647325947, 0.10932422783453888, 0.5542993741013826)
    orientation = (-0.15210168114981742, 0.17400067206982822,
                   0.6403224362890199, 0.7325134963231336)
    entry = wall(1.85997, -0.112, -0.17067)
    exit_wall = wall(4.22141, -0.096, -0.40234)
    assert target_in_compartment(position, orientation, entry, exit_wall, FORWARD)
    assert not target_in_compartment(position, orientation, exit_wall,
                                     wall(6.63684, -0.162, -0.65441), FORWARD)


def test_targets_in_other_compartments_are_deferred():
    for x, expected in ((1, False), (3, True), (5, False)):
        assert target_in_compartment((x, 0, 1), SIDE_VIEW,
                                     wall(2), wall(4), FORWARD) == expected


def test_reversed_detector_normal_does_not_change_assignment():
    assert target_in_compartment((3, 0, 1), SIDE_VIEW,
                                 wall(2, math.pi), wall(4, math.pi), FORWARD)


def test_view_direction_resolves_targets_on_shared_wall():
    for q, in_left in (((0, 0, 0, 1), True), ((0, 0, 1, 0), False)):
        assert target_in_compartment((4, 0, 1), q, wall(2), wall(4), FORWARD) == in_left
        assert target_in_compartment((4, 0, 1), q, wall(4), wall(6), FORWARD) != in_left


def test_slanted_wall_uses_plane_not_x_coordinate():
    assert target_in_compartment((4.1, -1, 1), SIDE_VIEW,
                                 wall(2), wall(4, math.pi/4), FORWARD)
    assert not target_in_compartment((3.9, 1, 1), SIDE_VIEW,
                                     wall(2), wall(4, math.pi/4), FORWARD)


def test_first_and_last_compartment_have_one_open_end():
    assert target_in_compartment((0, 0, 1), SIDE_VIEW, None, wall(2), FORWARD)
    assert target_in_compartment((8, 0, 1), SIDE_VIEW, wall(6), None, FORWARD)


def test_invalid_plane_is_reported_instead_of_skipping_targets():
    with pytest.raises(ValueError):
        target_in_compartment((3, 0, 1), SIDE_VIEW, wall(2), wall(4, math.pi/2), FORWARD)
    with pytest.raises(ValueError):
        target_in_compartment((3, 0, 1), SIDE_VIEW, wall(2), wall(4), (0, 0, 0))
