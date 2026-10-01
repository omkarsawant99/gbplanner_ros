#!/usr/bin/env python3
"""Assign AutoAssess inspection targets to planner compartments."""

import hashlib
import os
import sys
import threading

import rospy
# Catkin's devel launcher executes this source with its original __file__.
# Import the adjacent module directly, rather than its executable relay, which
# executes definitions in a separate dictionary and does not export them.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compartments import target_in_compartment
from planner_msgs.srv import GetInspectionTarget, GetInspectionTargetResponse
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String


class CompartmentTargets:
    def __init__(self):
        self.lock = threading.Lock()
        self.targets = None
        self.plan_id = None
        self.signature = None
        self.completed = set()
        self.config_namespace = rospy.get_param(
            "~planning_params_namespace", "/gbplanner_node/PlanningParams"
        ).rstrip("/")
        self.world_frame = rospy.get_param("~world_frame", "world").lstrip("/")
        rospy.Subscriber("/autoassess/inspection_targets", PoseArray,
                         self.on_targets, queue_size=1)
        rospy.Subscriber("/autoassess/plan_id", String, self.on_plan_id, queue_size=1)
        rospy.Service("~get_target", GetInspectionTarget, self.get_target)

    def on_targets(self, msg):
        geometry = (
            msg.header.frame_id,
            tuple((p.position.x, p.position.y, p.position.z,
                   p.orientation.x, p.orientation.y, p.orientation.z,
                   p.orientation.w) for p in msg.poses),
        )
        with self.lock:
            if geometry != self.signature:
                self.completed.clear()
            self.signature = geometry
            self.targets = msg

    def on_plan_id(self, msg):
        with self.lock:
            if self.plan_id != msg.data:
                self.completed.clear()
            self.plan_id = msg.data

    @staticmethod
    def wall(pose):
        return ((pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w))

    def get_target(self, request):
        response = GetInspectionTargetResponse()
        response.target_index = -1
        with self.lock:
            if self.targets is None or self.plan_id is None:
                response.message = "Waiting for AutoAssess targets and plan ID"
                return response
            targets = self.targets
            revision = hashlib.sha256(
                repr((self.plan_id, self.signature)).encode("utf-8")
            ).hexdigest()
            response.revision = revision
            if targets.header.frame_id.lstrip("/") != self.world_frame:
                response.message = "Target frame %r differs from planner frame %r" % (
                    targets.header.frame_id, self.world_frame)
                return response
            try:
                centers = rospy.get_param(self.config_namespace + "/compartment_centers")
                if not centers or len(centers) % 3:
                    raise ValueError("compartment_centers must contain XYZ triples")
                count = len(centers) // 3
                if not 0 <= request.compartment_index < count:
                    raise ValueError("Invalid compartment index")
                if bool(request.has_entry_wall) != (request.compartment_index > 0):
                    raise ValueError("Waiting for the traversed entry wall")
                if bool(request.has_exit_wall) != (request.compartment_index < count - 1):
                    raise ValueError("Waiting for a detected exit wall")
                entry = self.wall(request.entry_wall) if request.has_entry_wall else None
                exit_wall = self.wall(request.exit_wall) if request.has_exit_wall else None
                forward = (request.forward.x, request.forward.y, request.forward.z)
                membership = [
                    target_in_compartment(*self.wall(pose), entry, exit_wall, forward)
                    for pose in targets.poses
                ]
            except (KeyError, ValueError, rospy.ROSException) as error:
                response.message = "Compartment walls unavailable: %s" % error
                rospy.logwarn_throttle(10.0, response.message)
                return response

            response.ready = True
            if request.completed_target_index >= 0:
                if request.completed_revision != revision:
                    response.message = "Target plan changed before completion was recorded"
                elif (request.completed_target_index >= len(membership) or
                      not membership[request.completed_target_index]):
                    response.message = "Completed target does not belong to this compartment"
                    response.ready = False
                    return response
                else:
                    self.completed.add(request.completed_target_index)
                    rospy.loginfo("Completed AutoAssess target %d in compartment %d",
                                  request.completed_target_index, request.compartment_index)

            for index, belongs in enumerate(membership):
                if belongs and index not in self.completed:
                    response.has_target = True
                    response.target_index = index
                    response.target = PoseStamped()
                    response.target.header = targets.header
                    response.target.pose = targets.poses[index]
                    response.message = "Next inspection target %d" % index
                    return response
            response.message = "No pending targets in compartment %d" % request.compartment_index
            return response


if __name__ == "__main__":
    rospy.init_node("inspection_target_compartments")
    CompartmentTargets()
    rospy.spin()
