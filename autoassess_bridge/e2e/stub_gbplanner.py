#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Stub gbplanner for the e2e test: one fake mission, no planner needed.

Offers /gbplanner_node/generate_mesh (copies ~mesh_source to ~mesh_target, like voxblox
writing its mesh_filename) and flies one mission on the bridge's inputs: /gbplanner_path at
t=2, 4 and 6 s (homing announced before the last one), odometry moving until t=9 s, then
standing still — which is exactly what the bridge's mission-end detection waits for.

Params: ~mesh_source (default /data/test_mesh.ply), ~mesh_target (default
/tmp/gbplanner_mesh.ply; give the same path to the bridge's ~mesh_filename).
"""

import shutil

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse


def main():
    rospy.init_node("gbplanner_node")
    mesh_source = rospy.get_param("~mesh_source", "/data/test_mesh.ply")
    mesh_target = rospy.get_param("~mesh_target", "/tmp/gbplanner_mesh.ply")

    def generate(_req):
        shutil.copyfile(mesh_source, mesh_target)
        rospy.loginfo("stub: generate_mesh wrote %s", mesh_target)
        return EmptyResponse()

    rospy.Service("/gbplanner_node/generate_mesh", Empty, generate)
    path_pub = rospy.Publisher("/gbplanner_path", Path, queue_size=1)
    homing_pub = rospy.Publisher("/gbplanner_is_homing", Bool, queue_size=1)
    odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=1)

    rospy.sleep(3.0)  # let the bridge subscribe
    start = rospy.get_time()
    sent = set()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t = rospy.get_time() - start
        for when, homing in ((2, False), (4, False), (6, True)):
            if t >= when and when not in sent:
                sent.add(when)
                homing_pub.publish(Bool(data=homing))
                path = Path()
                path.header.frame_id = "world"
                path.poses = [PoseStamped()]
                path_pub.publish(path)
                rospy.loginfo("stub: t=%.1f path (homing=%s)", t, homing)
        odometry = Odometry()
        odometry.twist.twist.linear.x = 0.5 if t < 9 else 0.0
        odom_pub.publish(odometry)
        rate.sleep()


if __name__ == "__main__":
    main()
