#!/usr/bin/env python
#import numpy as np
#import cv2
#from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
import tf2_ros
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from aruco_follower import ArucoFollower
import math
from time import sleep
#def yaw_towards_frame(cf1_pose, target_frame, transform) returns list of quaternion point in frame cf1/odom

rotate_to = None
curr_pose = None

def check_callback(msg):
	global rotate_to
	rotate_to = msg
	return

def pose_callback(msg):
	global curr_pose
	curr_pose = msg
	return

def done_publish():
	msg = Bool()
	msg.data = True
	done_pub.publish(msg)
	return

def send_goal(checkpoint):
	goal = PoseStamped()
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = 'cf1/base_link'
	goal.pose.position.x = checkpoint[0]
	goal.pose.position.y = checkpoint[1]
	goal.pose.position.z = checkpoint[2]

	x,y,z,w = quaternion_from_euler(0,0,checkpoint[3])

	goal.pose.orientation.x = x
	goal.pose.orientation.y = y
	goal.pose.orientation.z = z
	goal.pose.orientation.w = w

	pose_pub.publish(goal)

rospy.init_node('checkpoint')
rospy.logwarn('-'*300)
check_sub = rospy.Subscriber('check_point', String, check_callback)
done_pub = rospy.Publisher('check_done', Bool, queue_size=10)

pose_sub = rospy.Subscriber('cf1/pose', PoseStamped, pose_callback)
pose_pub = rospy.Publisher('cf1/move_to', PoseStamped, queue_size=10)

tf_buf   = tf2_ros.Buffer()
tf_lis = tf2_ros.TransformListener(tf_buf, queue_size=100)

aruco_thing = ArucoFollower()


def main():
	rospy.logwarn('in main')
	global curr_pose
	global rotate_to
	sleep_time_in_rotation = 0.4

	rate = rospy.Rate(10) #Hz
	while not rospy.is_shutdown():
		if rotate_to and curr_pose:
			rospy.logwarn('Got a goal')
			rot_to = rotate_to
			rotate_to = None
			#then do the following

			#rotate 2 thirds of a full rotation
			rospy.logwarn('starting rotation')
			send_goal([0,0,0,math.pi*2/3])
			rospy.sleep(sleep_time_in_rotation)
			send_goal([0,0,0,math.pi*2/3])
			rospy.sleep(sleep_time_in_rotation)

			rospy.logwarn('constructing rotation message')
			rospy.logwarn(rot_to.data)
			"""
			if not tf_buf.can_transform('cf1/odom', rot_to.data, rospy.Time(0)):

				rospy.logwarn('couldn\'t transform to odom')
				continue


			try:
				transform = tf_buf.lookup_transform(rot_to.data,'cf1/odom', rospy.Time.now())
			except tf2_ros.LookupException:#, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logwarn((tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException))
				rate.sleep()
				continue
			"""
			transform = tf_buf.lookup_transform('cf1/base_link',rot_to.data, rospy.Time(0))
			quats = aruco_thing.yaw_towards_frame(curr_pose, rot_to.data, transform)

			rotate = PoseStamped()
			rotate.header.frame_id = 'cf1/base_link'
			rotate.header.stamp = rospy.Time.now()
			rotate.pose.orientation.x = quats[0]
			rotate.pose.orientation.y = quats[1]
			rotate.pose.orientation.z = quats[2]
			rotate.pose.orientation.w = quats[3]
			rotate.pose.position.x = 0
			rotate.pose.position.y = 0
			rotate.pose.position.z = 0
			rospy.logwarn('rotating back')
			pose_pub.publish(rotate)


			rospy.sleep(sleep_time_in_rotation*4)


			done_publish()
		rate.sleep()
	return

if __name__ == '__main__':
	main()
