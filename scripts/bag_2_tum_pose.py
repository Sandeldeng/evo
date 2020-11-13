#!/usr/bin/env python

## Publish a tf that put the origin of frame world at first local_position message

import rospy
import tf
import tf2_ros
import numpy 
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

init_mav_pose = None

path_pub = None
path = Path()

f_fusion = open("/home/mhc/fusion_pose.txt", 'w')
def fusion_pose_to_file_callback(msg):
    translation = msg.pose.position
    q = msg.pose.orientation
    t = msg.header.stamp
    f_fusion.write("{} {} {} {} {} {} {} {}\n".format(t, translation.x, translation.y, translation.z,
                q.x, q.y, q.z, q.w))
    f_fusion.flush()

f_vins = open("/home/mhc/vins_pose.txt", 'w')
def vins_pose_to_file_callback(msg):
    translation = msg.pose.pose.position
    q = msg.pose.pose.orientation
    t = msg.header.stamp
    f_vins.write("{} {} {} {} {} {} {} {}\n".format(t, translation.x, translation.y, translation.z,
                 q.x, q.y, q.z, q.w))
    f_vins.flush()

f_vins_loop = open("/home/mhc/vins_pose_loop.txt", 'w')
def loop_odom_pose_to_file_callback(msg):
    translation = msg.pose.pose.position
    q = msg.pose.pose.orientation
    t = msg.header.stamp
    f_vins_loop.write("{} {} {} {} {} {} {} {}\n".format(t, translation.x, translation.y, translation.z,
                 q.x, q.y, q.z, q.w))
    f_vins_loop.flush()

if __name__ == '__main__':
    rospy.init_node('mavros_basalt_tf_publisher')
    sub_fusion_pose_to_file = rospy.Subscriber('/fusion_pose', PoseStamped, fusion_pose_to_file_callback)
    sub_vins_pose_to_file = rospy.Subscriber('/vins_estimator/odometry', Odometry, vins_pose_to_file_callback)
    sub_loop_odom_pose_to_file = rospy.Subscriber('/odom_loop_filtered', Odometry, loop_odom_pose_to_file_callback)

    while not init_mav_pose:
        pass
    sub_fusion_pose_to_file.unregister()