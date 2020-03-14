#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray


class KalmanFilter:
    def __init__(self, initial_cov, R, delta_t):
        # position/mean not neaded since it will provided by the crazyflie
        self.cov = initial_cov

        # [xy, yaw]
        self.R = R#*delta_t
        self.delta_t = delta_t#*1000 # wtf?
        #rospy.loginfo("DELTA T: " + str(self.delta_t))
        #rospy.loginfo("RRRRRRRRRRRRRRRRRRRRRRRRRRRR: " + str(self.R))
        
        #self.std_xy = 0.1
        #self.std_yaw = 0.3

    def predict(self, A, u):
        # increase uncertainty depending on control input u
        # u is the velocity?
        self.cov[0] = self.cov[0]*A[0]**2 + self.R[0]*self.delta_t + u[0]*self.delta_t
        self.cov[1] = self.cov[1]*A[1]**2 + self.R[1]*self.delta_t + u[1]*self.delta_t
        self.cov[2] = self.cov[2]*A[2]**2 + self.R[2]*self.delta_t + u[2]*self.delta_t
        return self.cov

    def update(self, Q):
        K_x = self.cov[0]/(self.cov[0] + Q[0])
        K_y = self.cov[1]/(self.cov[1] + Q[1])
        K_yaw = self.cov[2]/(self.cov[2] + Q[2])
        self.cov[0] = (1-K_x)*self.cov[0]
        self.cov[1] = (1-K_y)*self.cov[1]
        self.cov[2] = (1-K_yaw)*self.cov[2]
        return K_x, K_y, K_yaw

class MapOdomUpdate:
    def __init__(self, init_trans, update_freq=10):
        # TODO: import MarkerArray
        self.aruco_detect_sub = rospy.Subscriber('/aruco/markers', MarkerArray, self.update_callback)
        self.cf1_pose_sub = rospy.Subscriber("cf1/pose", PoseStamped, self.cf1_pose_callback)
        self.cf1_pose_cov_pub = rospy.Publisher("cf1/pose_cov", PoseWithCovarianceStamped, queue_size=1)
        self.cf1_pose_sub = rospy.Subscriber("cf1/velocity", TwistStamped, self.cf1_vel_callback)

        init_trans.header.frame_id = "map"
        init_trans.child_frame_id = "cf1/odom"
        self.init_trans = init_trans

        self.cf1_pose = None
        self.cf1_vel = None

        self.measurement_msg = None
        self.old_msg = None
        self.last_transform = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, queue_size=1) # should there be a queue_size?
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
       
        self.update_freq = update_freq
        self.kf = KalmanFilter(initial_cov=np.array([0.01, 0.01, 0.01]), R=np.array([.0001, .0001, .0005]), delta_t=1.0/self.update_freq)
        #self.kf = KalmanFilter(initial_cov=np.array([0.1, 0.1, 0.1]), R=np.array([0.005, 1.0, 1.0]), delta_t=1.0/self.update_freq)
    
    def spin(self):
        rate = rospy.Rate(self.update_freq)
        while not rospy.is_shutdown():

            if self.last_transform == None:
                self.init_trans.header.stamp = rospy.Time.now()
                self.broadcaster.sendTransform(self.init_trans)
            else:
                self.last_transform.header.stamp = rospy.Time.now()
                self.broadcaster.sendTransform(self.last_transform)
            
            # determine u by checking if the drone is in motion
            if self.cf1_vel:
                K_vel = 0.5
                vx = self.cf1_vel.twist.linear.x * K_vel
                vy = self.cf1_vel.twist.linear.y * K_vel
                w = self.cf1_vel.twist.angular.z * K_vel
                dt = 1.0/self.update_freq
                A = [1 + abs(vx)*dt, 1 + abs(vy)*dt, 1 + abs(w)*dt]
                u = [0, 0, 0]
                self.kf.predict(A, u)
            else:
                A = [1, 1, 1]
                u = [0, 0, 0]
                self.kf.predict(A, u)

            if self.measurement_msg: self.update(self.measurement_msg)

            if self.cf1_pose: 
                p = PoseWithCovarianceStamped()
                p.header = self.cf1_pose.header # correct to use cf1/odom as frame_id???
                p.pose.pose = self.cf1_pose.pose
                p.pose.covariance[0] = self.kf.cov[0]
                p.pose.covariance[1] = self.kf.cov[0]*self.kf.cov[1]
                p.pose.covariance[6] = self.kf.cov[0]*self.kf.cov[1]
                p.pose.covariance[7] = self.kf.cov[1]
                p.pose.covariance[-1] = self.kf.cov[2]
                self.cf1_pose_cov_pub.publish(p)
            rate.sleep()

    def cf1_pose_callback(self, msg):
        self.cf1_pose = msg

    def cf1_vel_callback(self, msg):
        self.cf1_vel = msg

    def update_callback(self, msg):
        self.measurement_msg = msg

    def update(self, m_array):
        print("here")
        if self.old_msg == m_array:
            # Message old 
            #raise Exception("ITS TOO OLD")
            print("old")
            return
        else:
            if self.old_msg:
                print("not old")
                print(len(m_array.markers))
                print(len(self.old_msg.markers))
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
        self.static_broadcaster.sendTransform(detected_to_map_new)

        # Transform between map and measured map filtered using kalman filter
        while not rospy.is_shutdown():
            try: map_to_map_new = self.tf_buf.lookup_transform("map", "map_measured", rospy.Time(0)) # rospy.Time(0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                print(e)
                print("walla1")
            else: break
        Q=[0.1, 0.1, 0.1]
        #Q=[1, 1, 1]
        map_filtered = self.kalman_filter(map_to_map_new, Q)
        self.broadcaster.sendTransform(map_filtered)

        # Filtered map to cf1/odom redefines new map to cf1/odom
        while not rospy.is_shutdown():
            try: 
                map_filtered_to_odom = self.tf_buf.lookup_transform("map_filtered", "cf1/odom", rospy.Time(0)) # rospy.Time(0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                print(e)
                print("walla2")
            else: break
        map_filtered_to_odom.header.frame_id = "map"
        map_filtered_to_odom.child_frame_id = "cf1/odom"
        map_to_odom = map_filtered_to_odom
        map_to_odom.header.stamp = rospy.Time.now()

        rospy.loginfo("Created new transform between map and cf1/odom")
        self.last_transform = map_to_odom

    def kalman_filter(self, transform, Q):
        # TODO: Kalman filter, for now fixed kalman gain

        Kx, Ky, K_rot = self.kf.update(Q)

        t = transform
        
        new_t = TransformStamped()
        new_t.header.stamp = t.header.stamp
        new_t.header.frame_id = "map"
        new_t.child_frame_id = "map_filtered"
        new_t.transform.translation.x = Kx*t.transform.translation.x
        new_t.transform.translation.y = Ky*t.transform.translation.y
        #t.transform.translation.z = Kx*t.transform.translation.z *0 # Multiply by zero, no risk of drift

        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        
        a1, a2, a3 = euler_from_quaternion(q)
        a1 = K_rot*a1 #*0 # Multiply by zero, no risk of drift
        a2 = K_rot*a2 #*0 # Multiply by zero, no risk of drift
        a3 = K_rot*a3
        q_new = quaternion_from_euler(a1, a2, a3)
        new_t.transform.rotation.x = q_new[0]
        new_t.transform.rotation.y = q_new[1]
        new_t.transform.rotation.z = q_new[2]
        new_t.transform.rotation.w = q_new[3]
        
        return new_t
        
if __name__ == '__main__':
    rospy.init_node('update_map_to_odom')
    init_trans_str = rospy.get_param(rospy.get_name() + "/initial_map_to_odom")
    init_trans_ls = [float(s.strip()) for s in init_trans_str.split()]
    init_t = TransformStamped()
    init_t.transform.translation.x = init_trans_ls[0]
    init_t.transform.translation.y = init_trans_ls[1]
    init_t.transform.translation.z = init_trans_ls[2]

    (init_t.transform.rotation.x, 
    init_t.transform.rotation.y, 
    init_t.transform.rotation.z, 
    init_t.transform.rotation.w) = quaternion_from_euler(init_trans_ls[3], 
                                                         init_trans_ls[4], 
                                                         init_trans_ls[5])


    p = MapOdomUpdate(init_trans=init_t, update_freq=20)
    p.spin()