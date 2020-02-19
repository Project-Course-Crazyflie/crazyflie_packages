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
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.last_transform == None:
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "cf1/odom"
                t.transform.rotation.w = 1.0
                self.broadcaster.sendTransform(t)
                #rospy.loginfo("Sending 0 between map and cf1/odom")
            else:
                
                self.last_transform.header.stamp = rospy.Time.now()
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
        
        #self.tf_buf.waitForTransform(frame_map, frame_detected, rospy.Time(0), rospy.Duration(3.0))
        if not self.tf_buf.can_transform(frame_map, frame_detected, rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from {} to {}'.format(frame_map, frame_detected))
            return

        # TODO: dont do transform in callback?
        try:
            map_to_detected = self.tf_buf.lookup_transform(frame_map, frame_detected, rospy.Time(0))
            map_to_odom = self.tf_buf.lookup_transform("cf1/odom", "map", rospy.Time(0))
        except:
            rospy.loginfo("Transform no longer exists")
            #self.last_transform.header.stamp = rospy.Time.now()
            #self.broadcaster.sendTransform(self.last_transform)
            return
        # TODO: outlier detection
        map_to_detected = self.kalman_filter(map_to_detected)
        map_to_detected.header.frame_id = "map"
        map_to_detected.child_frame_id = "map_new"
        map_to_detected.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(map_to_detected)


        map_to_odom = self.tf_buf.lookup_transform("map_new", "cf1/odom", rospy.Time(0))

        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "cf1/odom"
        map_to_odom.header.stamp = rospy.Time.now()
        rospy.loginfo("Created new transform between map and cf1/odom")
        self.last_transform = map_to_odom
        
    def kalman_filter(self, transform):
        K = 0.2
        #K = 1
        t = transform
        t.transform.translation.x = K*t.transform.translation.x
        t.transform.translation.y = K*t.transform.translation.y
        t.transform.translation.z = K*t.transform.translation.z

        K_rot = 0.1
        #K_rot = 1
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        a1, a2, a3 = euler_from_quaternion(q)
        a1 = K_rot*a1
        a2 = K_rot*a2
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