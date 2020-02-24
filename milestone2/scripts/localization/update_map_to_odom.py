#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import tf
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
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, queue_size=100)
        self.broadcaster = tf2_ros.TransformBroadcaster()
       
    
    def spin(self):
        while not rospy.is_shutdown():
            if self.last_transform == None:
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "cf1/odom"
                t.transform.rotation.w = 1
                self.broadcaster.sendTransform(t)
            else:
                self.last_transform.header.stamp = rospy.Time.now()
                self.broadcaster.sendTransform(self.last_transform)

    def update_callback(self, m_array):
        if m_array == self.old_msg:
            # Message old 
            return
        self.old_msg = m_array
        
        # TODO: Make possible to update multiple markes?
        m = m_array.markers[0]
        frame_detected = "aruco/detected" + str(m.id)
        frame_map = "aruco/marker" + str(m.id)
        
        if not self.tf_buf.can_transform(frame_map, frame_detected, rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from {} to {}'.format(frame_map, frame_detected))
            return

        # Transform between marker and map is the same as detected to measured map
        detected_to_map_new = self.tf_buf.lookup_transform(frame_map, "map", rospy.Time(0))
        detected_to_map_new.header.frame_id = frame_detected
        detected_to_map_new.child_frame_id = "map_measured"
        detected_to_map_new.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(detected_to_map_new)

        # Transform between map and measured map filtered using kalman filter
        map_to_map_new = self.tf_buf.lookup_transform("map", "map_measured", rospy.Time(0))
        map_filtered = self.kalman_filter(map_to_map_new)
        map_filtered.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(map_filtered)

        # Filtered map to cf1/odom redefines new map to cf1/odom
        map_filtered_to_odom = self.tf_buf.lookup_transform("map_filtered", "cf1/odom", rospy.Time(0))
        map_filtered_to_odom.header.frame_id = "map"
        map_filtered_to_odom.child_frame_id = "cf1/odom"
        map_to_odom = map_filtered_to_odom
        map_to_odom.header.stamp = rospy.Time.now()


        rospy.loginfo("Created new transform between map and cf1/odom")
        self.last_transform = map_to_odom
        
    def kalman_filter(self, transform):
        # TODO: Kalman filter, for now fixed kalman gain
        K = 0.05

        t = transform
        t.header.frame_id = "map"
        t.child_frame_id = "map_filtered"
        t.transform.translation.x = K*t.transform.translation.x
        t.transform.translation.y = K*t.transform.translation.y
        t.transform.translation.z = K*t.transform.translation.z *0 # Multiply by zero, no risk of drift

        K_rot = 0.1

        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        
        a1, a2, a3 = euler_from_quaternion(q)
        a1 = K_rot*a1 *0 # Multiply by zero, no risk of drift
        a2 = K_rot*a2 *0 # Multiply by zero, no risk of drift
        a3 = K_rot*a3
        q_new = quaternion_from_euler(a1, a2, a3)
        t.transform.rotation.x = q_new[0]
        t.transform.rotation.y = q_new[1]
        t.transform.rotation.z = q_new[2]
        t.transform.rotation.w = q_new[3]

        return t
        
if __name__ == '__main__':
    rospy.init_node('map_to_odom')
    p = MapOdomUpdate()
    p.spin()