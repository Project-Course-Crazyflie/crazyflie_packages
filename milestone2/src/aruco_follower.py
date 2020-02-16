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


rospy.init_node('aruco_follower')
follower_pub = rospy.Publisher('/cf1/move_to', PoseStamped, queue_size=2)

def main():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        inp = raw_input("Aruco marker to follow: ")
        goal = PoseStamped()
        goal.header.frame_id = "aruco/detected" + inp
        
        goal.pose.position.y = 1.0

        q = quaternion_from_euler(math.pi, 0, math.pi/2)
        print(q)

        goal.pose.orientation.x = q[0] 
        goal.pose.orientation.y = -q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]   

        follower_pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    main()