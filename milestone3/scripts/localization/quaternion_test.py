#!/usr/bin/env python

import math
import sys
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3Stamped, QuaternionStamped
from std_msgs.msg import Bool
from crazyflie_driver.msg import Position
from aruco_msgs.msg import Marker, MarkerArray

import matplotlib.pyplot as plt


def main():
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, queue_size=1)
    rospy.sleep(2)
    q = quaternion_from_euler(1.57, 0, 0)
    pose_rotation = PoseStamped()
    pose_rotation.header.frame_id = "cf1/odom"
    pose_rotation.pose.orientation.x = q[0]
    pose_rotation.pose.orientation.y = q[1]
    pose_rotation.pose.orientation.z = q[2]
    pose_rotation.pose.orientation.w = q[3]


    pose_map = tf_buf.transform(pose_rotation, "map")
    print(euler_from_quaternion([pose_map.pose.orientation.x, 
                                 pose_map.pose.orientation.y, 
                                 pose_map.pose.orientation.z, 
                                 pose_map.pose.orientation.w]))

    t = tf_buf.lookup_transform("cf1/odom", "map", rospy.Time(0))
    print(euler_from_quaternion([t.transform.rotation.x, 
                                 t.transform.rotation.y, 
                                 t.transform.rotation.z, 
                                 t.transform.rotation.w]))

if __name__ == '__main__':
    
    rospy.init_node('quaternion_test')

    main()
    
