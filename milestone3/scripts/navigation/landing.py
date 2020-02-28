#!/usr/bin/env python
#import numpy as np
#import cv2
#from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
import tf2_ros
import tf2_msgs
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

land = None
pose = None

def callback(msg):
	global land
	land = msg
	return

def callback_pose(msg):
    global pose
    pose = msg 
    return

def publish(msg):
	land_pub.publish(msg)
	return

rospy.init_node("landing")
land_sub = rospy.Subscriber('stop', Empty, callback)
pose_sub = rospy.Subscriber('cf1/pose', PoseStamped, callback_pose)
land_pub = rospy.Publisher('cf1/move_to', PoseStamped, queue_size=10)


def main():
	global land
	rate = rospy.Rate(10) #Hz
	while not rospy.is_shutdown():
		if land:
			land = None

			while pose.pose.position.z > 0.15:
				next_pose = pose
				next_pose.pose.position.z -= 0.05
				publish(next_pose)
			next_pose = pose
			next_pose.pose.position.z = -9999


		rate.sleep()
	return

if __name__ == '__main__':
	main()
