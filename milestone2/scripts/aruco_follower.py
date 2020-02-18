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



    

class ArucoFollower:
    def __init__(self):
        
        self.follower_pub = rospy.Publisher('/cf1/move_to', PoseStamped, queue_size=2)
        self.cf1_pose_top = rospy.Subscriber('/cf1/pose', PoseStamped, self.cf1_pose_callback)
        self.cf1_pose = None

        self.tf_buf = tf2_ros.Buffer()
        tf_lstn = tf2_ros.TransformListener(self.tf_buf)


    def cf1_pose_callback(self, pose):
        self.cf1_pose = pose
        
    def yaw_towards_frame(self, cf1_pose, target_frame, T=None):
        """Return the yaw towards frame represented in a quaternion (list)
        """
        # TODO: send in transform T as argument instead of calculating here
        if not self.tf_buf.can_transform(target_frame, 'cf1/odom', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % target_frame)
            return
            
        frame_pose = PoseStamped() # [0, 0, 0]
        frame_pose.header.stamp = rospy.Time(0)
        frame_pose.header.frame_id = target_frame
        
        p = self.tf_buf.transform(frame_pose, 'cf1/odom')
        
        frame_pos = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
        curr = np.array([cf1_pose.pose.position.x, cf1_pose.pose.position.y, cf1_pose.pose.position.z])
                    
        diff = frame_pos-curr
        theta = np.arctan2(diff[1], diff[0])
        
        q_rot = quaternion_from_euler(0, 0, theta,"sxyz")
        
        return q_rot

    def follow_detected(self, aruco_id):
        aruco_frame = "aruco/detected" + str(aruco_id)

        while not self.cf1_pose: rospy.loginfo("Waiting for cf1/pose...")

        if not self.tf_buf.can_transform(aruco_frame, 'cf1/odom', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % aruco_frame)
            return
        
        # TODO: look up transform instead of using .transform multiple times
        #(trans, rot) = self.tf_lstn.lookupTransform(aruco_frame, 'cf1/odom',  rospy.Time(0))
        aruco_pose = PoseStamped()
        
        aruco_pose.header.frame_id = aruco_frame
        aruco_pose.header.stamp = rospy.Time(0)
        
        p0 = self.tf_buf.transform(aruco_pose, 'cf1/odom')
        aruco_pose.pose.position.y = 0.5
        p1 = self.tf_buf.transform(aruco_pose, 'cf1/odom')
        
        
        goal = PoseStamped()
        goal.header.frame_id = aruco_frame
        
        q = self.yaw_towards_frame(self.cf1_pose, target_frame=aruco_frame, T=None)
        
        
        goal_odom = PoseStamped()
        goal_odom.header.frame_id = "cf1/odom"
        goal_odom.pose.position.x = p1.pose.position.x
        goal_odom.pose.position.y = p1.pose.position.y
        goal_odom.pose.position.z = p1.pose.position.z # use p0 here to keep the crazyflie on the same height as the aruco marker
        goal_odom.pose.orientation.x = q[0] 
        goal_odom.pose.orientation.y = q[1]
        goal_odom.pose.orientation.z = q[2]
        goal_odom.pose.orientation.w = q[3]
        
        goal_aruco = self.tf_buf.transform(goal_odom, aruco_frame)
        
        
        """
        aruco_y_odom = np.array([p1.pose.position.x-p0.pose.position.x, 
                                 p1.pose.position.y-p0.pose.position.y, 
                                 p1.pose.position.z-p0.pose.position.z])
                                 
        if y_odom.pose.position.z < 0.1:
            # y axle is parallell with x,y odom and aruco is treated as being on a wall
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = p1.pose.position.x, p1.pose.position.y, p0.pose.position.z
            
        else:
            # treated as being on the floor
            pass
        """
        
        self.follower_pub.publish(goal_aruco)

    def follow_map(self, aruco_id):
        aruco_frame = "aruco/marker" + str(aruco_id)

        while not self.cf1_pose: rospy.loginfo("Waiting for cf1/pose...")

        if not self.tf_buf.can_transform(aruco_frame, 'map', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % aruco_frame)
            return
        
        # TODO: look up transform instead of using .transform multiple times
        #(trans, rot) = self.tf_lstn.lookupTransform(aruco_frame, 'cf1/odom',  rospy.Time(0))
        aruco_pose = PoseStamped()
        
        aruco_pose.header.frame_id = aruco_frame
        aruco_pose.header.stamp = rospy.Time(0)
        
        p0 = self.tf_buf.transform(aruco_pose, 'map')
        aruco_pose.pose.position.y = 0.5
        p1 = self.tf_buf.transform(aruco_pose, 'map')
        
        
        goal = PoseStamped()
        goal.header.frame_id = aruco_frame
        
        q = self.yaw_towards_frame(self.cf1_pose, target_frame=aruco_frame, T=None)
        
        
        goal_odom = PoseStamped()
        goal_odom.header.frame_id = "map"
        goal_odom.pose.position.x = p1.pose.position.x
        goal_odom.pose.position.y = p1.pose.position.y
        goal_odom.pose.position.z = p1.pose.position.z # use p0 here to keep the crazyflie on the same height as the aruco marker
        goal_odom.pose.orientation.x = q[0] 
        goal_odom.pose.orientation.y = q[1]
        goal_odom.pose.orientation.z = q[2]
        goal_odom.pose.orientation.w = q[3]
        
        goal_aruco = self.tf_buf.transform(goal_odom, aruco_frame)
        
        
        """
        aruco_y_odom = np.array([p1.pose.position.x-p0.pose.position.x, 
                                 p1.pose.position.y-p0.pose.position.y, 
                                 p1.pose.position.z-p0.pose.position.z])
                                 
        if y_odom.pose.position.z < 0.1:
            # y axle is parallell with x,y odom and aruco is treated as being on a wall
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = p1.pose.position.x, p1.pose.position.y, p0.pose.position.z
            
        else:
            # treated as being on the floor
            pass
        """
        
        self.follower_pub.publish(goal_aruco)

if __name__ == '__main__':
    rospy.init_node('aruco_follower')
    follower = ArucoFollower()
    while not rospy.is_shutdown():
        aruco_id = inp = raw_input("Aruco marker to follow: ")
        follower.follow_map(aruco_id)
    
    