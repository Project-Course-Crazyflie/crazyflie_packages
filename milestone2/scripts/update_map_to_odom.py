#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray


class MapOdomUpdate:
    def __init__(self):
        # TODO: import MarkerArray
        self.aruco_detect_sub = rospy.Subscriber('/aruco/markers', MarkerArray, self.update_callback)
        self.old_msg = None
        self.last_transform = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf)

        self.broadcaster = tf2_ros.TransformBroadcaster()
       
    
    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.last_transform == None:
                t = TransformStamped()
                t.header.stamp = rospy.Time(0)
                t.header.frame_id = "map"
                t.child_frame_id = "cf1/odom"
                t.transform.rotation.w = 1.0
                self.broadcaster.sendTransform(t)
            else:
                self.broadcaster.sendTransform(self.last_transform)
            rate.sleep()

    def update_callback(self, m_array):
        if m_array == self.old_msg:
            # Message old 
            return
        self.old_msg = m_array
        
        # TODO: Make possible to update multiple markes?
        m = m_array.markers[0]
        frame_detected = "aruco/detected" + str(m.id)
        frame_map = "aruco/marker" + str(m.id)
        
        if not self.tf_buf.can_transform(frame_detected, frame_map, rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % frame_map)
            return
        
        transform = self.tf_buf.lookupTransform(frame_map, frame_detected, rospy.Time(0))
        # TODO: outlier detection
        transform.header.frame_id = "map"
        transform.child_frame_id = "cf1/odom"
        transform.header.stamp = rospy.Time(0)
        self.last_transform = transform
        
        
if __name__ == '__main__':
    rospy.init_node('map_to_odom')
    p = MapOdomUpdate()
    p.spin()