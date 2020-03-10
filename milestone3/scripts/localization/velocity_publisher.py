#!/usr/bin/env python
import numpy as np
import rospy
#import tf2_ros
#import tf2_msgs
#import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class VelocityPublisher:
    def __init__(self, freq):
        self.freq = freq
        self.cf1_pose_sub = rospy.Subscriber("cf1/pose", PoseStamped, self.cf1_pose_callback)
        self.curr_pose = None
        self.prev_pose = None

        self.cf1_vel_top = rospy.Publisher("cf1/velocity", TwistStamped, queue_size=1)
        self.prev_vel = None

    def cf1_pose_callback(self, msg):
        if not self.prev_pose: 
            self.prev_pose = self.curr_pose
        self.curr_pose = msg

    def spin(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.prev_pose and self.prev_pose != self.curr_pose:
                
                delta_t = 1.0/self.freq
                twist = TwistStamped()
                twist.header.stamp = rospy.Time.now()
                twist.header.frame_id = "cf1/odom"
                twist.twist.linear.x = (self.curr_pose.pose.position.x - self.prev_pose.pose.position.x)/delta_t
                twist.twist.linear.y = (self.curr_pose.pose.position.y - self.prev_pose.pose.position.y)/delta_t

                _, _, curr_yaw = euler_from_quaternion([self.curr_pose.pose.orientation.x,
                                                        self.curr_pose.pose.orientation.y,
                                                        self.curr_pose.pose.orientation.z,
                                                        self.curr_pose.pose.orientation.w])
                _, _, prev_yaw = euler_from_quaternion([self.prev_pose.pose.orientation.x,
                                                        self.prev_pose.pose.orientation.y,
                                                        self.prev_pose.pose.orientation.z,
                                                        self.prev_pose.pose.orientation.w])

                # Angle between prev and curr yaw
                a = curr_yaw - prev_yaw
                a = (a + np.pi) % (2*np.pi) - np.pi

                twist.twist.angular.z = a/delta_t
                self.cf1_vel_top.publish(twist)
                self.prev_pose = self.curr_pose

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("velocity_publisher")
    vel_pub = VelocityPublisher(20)
    vel_pub.spin()
