"""Exercise wall readiness and target completion using real generated ROS messages."""
import os
import sys
from unittest.mock import patch

from geometry_msgs.msg import Pose, PoseArray
from planner_msgs.srv import GetInspectionTargetRequest
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from inspection_target_compartments import CompartmentTargets


def pose(x):
    p = Pose()
    p.position.x = x
    p.orientation.w = 1
    return p


def request(compartment=1):
    r = GetInspectionTargetRequest(compartment_index=compartment, completed_target_index=-1)
    r.has_entry_wall = compartment > 0
    r.has_exit_wall = compartment < 2
    r.entry_wall = pose(1.86 if compartment == 1 else 4.22)
    r.exit_wall = pose(4.22 if compartment == 1 else 1.86)
    r.forward.x = 1
    return r


def queue():
    with patch('rospy.get_param', side_effect=lambda name, default=None: default), \
         patch('rospy.Subscriber'), patch('rospy.Service'):
        node = CompartmentTargets()
    targets = PoseArray()
    targets.header.frame_id = 'world'
    targets.poses = [pose(4.13)]
    node.on_targets(targets)
    node.on_plan_id(String(data='test-plan'))
    return node


def call(node, req):
    with patch('rospy.get_param', return_value=[0, 0, 1.5, 2, 0, 1.5, 4, 0, 1.5]), \
         patch('rospy.loginfo'), patch('rospy.logwarn_throttle'):
        return node.get_target(req)


def test_target_before_wall_is_available_in_second_compartment():
    result = call(queue(), request())
    assert result.ready and result.has_target and result.target_index == 0


def test_missing_exit_wall_waits_instead_of_reporting_empty_compartment():
    req = request()
    req.has_exit_wall = False
    result = call(queue(), req)
    assert not result.ready
    assert 'exit wall' in result.message


def test_completed_target_is_not_repeated_and_new_plan_resets_completion():
    node = queue()
    result = call(node, request())
    req = request()
    req.completed_target_index = 0
    req.completed_revision = result.revision
    result = call(node, req)
    assert result.ready and not result.has_target
    node.on_plan_id(String(data='new-plan'))
    assert call(node, request()).has_target


def test_target_in_later_compartment_does_not_block_traversal():
    result = call(queue(), request(0))
    assert result.ready and not result.has_target
