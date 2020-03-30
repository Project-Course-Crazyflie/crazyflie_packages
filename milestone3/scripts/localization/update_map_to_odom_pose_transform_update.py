#!/usr/bin/env python

import math
import sys
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped, Quaternion
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Vector3#, QuaternionStamped
from std_msgs.msg import Bool
from crazyflie_driver.msg import Position
from aruco_msgs.msg import Marker, MarkerArray

import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, initial_cov, R, delta_t):
        # position/mean not neaded since it will provided by the crazyflie
        self.cov = np.diag(initial_cov)
        self.R = np.diag(R)
        self.delta_t = delta_t

        self.n_updates = 0
        self.prev_cov = self.cov.copy()
        self.cov_rate_of_change = None

        self.converged_pub = rospy.Publisher("cf1/localization/", Bool, queue_size=1)

    def _predict(self, A, u):
        # increase uncertainty depending on control input u
        # u is the velocity?
        self.cov[0][0] = self.cov[0][0]*A[0][0]**2 + self.R[0][0]*self.delta_t + u[0]*self.delta_t
        self.cov[1][1] = self.cov[1][1]*A[1][1]**2 + self.R[1][1]*self.delta_t + u[1]*self.delta_t
        self.cov[2][2] = self.cov[2][2]*A[2][2]**2 + self.R[2][2]*self.delta_t + u[2]*self.delta_t
        return self.cov

    def _update(self, Q):
        K_x = self.cov[0]/(self.cov[0] + Q[0])
        K_y = self.cov[1]/(self.cov[1] + Q[1])
        K_yaw = self.cov[2]/(self.cov[2] + Q[2])
        self.cov[0] = (1-K_x)*self.cov[0]
        self.cov[1] = (1-K_y)*self.cov[1]
        self.cov[2] = (1-K_yaw)*self.cov[2]
        return K_x, K_y, K_yaw

    def converged(self):
        return 

    def predict(self, A, u):
        self.cov = np.matmul(np.matmul(A, self.cov), A) + self.R*self.delta_t

    def kalman_gain(self, Q):
        K = np.matmul(self.cov, np.linalg.inv(self.cov + Q))
        return K

    def update(self, mu, residual, Q):
        K = self.kalman_gain(Q)
        self.update_with_gain(K)
        return mu + np.matmul(K, residual)

    def update_with_gain(self, K):
        #K = self.kalman_gain(Q)
        self.cov = np.matmul((np.eye(K.shape[0])-K), self.cov)

        self.cov_rate_of_change = self.cov - self.prev_cov
        self.prev_cov = self.cov.copy()
        #print((self.cov_rate_of_change/self.cov*1000).round(1))
        #return K.diagonal()

class MapOdomUpdate:
    def __init__(self, init_trans, update_freq):
        #rospy.Subscriber('cf1/localization/detection', Marker, self.update_callback) # use this instead
        rospy.Subscriber('aruco/markers', MarkerArray, self.update_callback)
        rospy.Subscriber('sign_pose', MarkerArray, self.update_callback)
        
        rospy.Subscriber("cf1/pose", PoseStamped, self.cf1_pose_callback)
        rospy.Subscriber("cf1/velocity", TwistStamped, self.cf1_vel_callback)
        self.cf1_pose_cov_pub = rospy.Publisher("cf1/localizatiton/pose_cov", PoseWithCovarianceStamped, queue_size=1)
        self.valid_detection_pub = rospy.Publisher("cf1/localization/detection_valid", Marker, queue_size=1)

        init_trans.header.frame_id = "map"
        init_trans.child_frame_id = "cf1/odom"
        self.init_trans = init_trans # remove?
        self.last_transform = init_trans

        self.cf1_pose = None
        self.cf1_vel = None

        self.is_measuring = False
        self.measurement_msg = None
        self.old_msg = None
        # what translation/rotation variables to use in filtering
        self.filter_config = rospy.get_param("localization/measurement_config")

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, queue_size=1) # should there be a queue_size?
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
       
        self.update_freq = update_freq
        #self.kf = KalmanFilter(initial_cov=np.array([100000000.01, 100000000.01, 100000000.01]), R=np.array([.0005, .0005, .001]), delta_t=1.0/self.update_freq)
        self.kf = KalmanFilter(initial_cov=np.array([100000000.01, 100000000.01, 100000000.01, 100000000.01, 100000000.01, 100000000.01]), R=np.array([.005, .005, .005, .001, .001, .001]), delta_t=1.0/self.update_freq)


    def spin(self):
        rate = rospy.Rate(self.update_freq)
        while not rospy.is_shutdown():
            #A = np.diag([1, 1, 1])
            #u = [0, 0, 0]
            A = np.diag([1, 1, 1, 1, 1, 1])
            u = [0, 0, 0, 0, 0, 0]
            self.kf.predict(A, u)

            if self.measurement_msg: 
                map_to_odom = self.update(self.msg_to_measurement(self.measurement_msg))
                if map_to_odom: self.last_transform = map_to_odom
            #    if valid_marker: self.valid_detection_pub.publish(valid_marker)


            self.last_transform.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(self.last_transform)
            
            # determine u by checking if the drone is in motion
            """
            if False and self.cf1_vel:
                K_vel = 0.5
                vx = self.cf1_vel.twist.linear.x * K_vel
                vy = self.cf1_vel.twist.linear.y * K_vel
                w = self.cf1_vel.twist.angular.z * K_vel
                dt = 1.0/self.update_freq
                A = [1 + abs(vx)*dt, 1 + abs(vy)*dt, 1 + abs(w)*dt]
                u = [0, 0, 0]
                self.kf.predict(A, u)

            elif False and self.cf1_vel:
                A = np.eye(4)
                A[0][2] = 1.0/self.update_freq
                A[1][3] = 1.0/self.update_freq
                self.kf.predict(A, u)
            else:
                A = [1, 1, 1]
                u = [0, 0, 0]
                self.kf.predict(A, u)
            """


            if self.cf1_pose: 
                p = PoseWithCovarianceStamped()
                p.header = self.cf1_pose.header # correct to use cf1/odom as frame_id???
                p.pose.pose = self.cf1_pose.pose
                p.pose.covariance[0] = self.kf.cov[0][0]
                #p.pose.covariance[1] = self.kf.cov[0][0]*self.kf.cov[1][1]
                #p.pose.covariance[6] = self.kf.cov[0][0]*self.kf.cov[1][1]
                p.pose.covariance[7] = self.kf.cov[1][1]
                p.pose.covariance[-1] = self.kf.cov[5][5]
                
                self.cf1_pose_cov_pub.publish(p)
            rate.sleep()

    def cf1_pose_callback(self, msg):
        self.cf1_pose = msg

    def cf1_vel_callback(self, msg):
        self.cf1_vel = msg

    def msg_to_measurement(self, msg):
        # TODO
        return msg

    def update_callback(self, m_array):
        if self.measurement_msg != m_array:
            self.measurement_msg = m_array

    def update(self, m_array):
            """
            Using poses v1
            """
            if self.old_msg == m_array:
                # Message old 
                return
           
            self.old_msg = m_array
            n_markers = len(m_array.markers)
            kalman_gains = []
            odom_to_filtered_transforms = []
            for marker in m_array.markers:
                time_stamp = marker.header.stamp
                # TODO: make this general (no hardcoded Qs)            
                if marker.id > 9: 
                    frame_detected = "sign_detected/stop"
                    frame_marker = "sign/stop"
                    print("Using stop sign!!!")
                    Q = np.diag([0.5, 0.5, 0.5])
                else: 
                    frame_detected = "aruco/detected" + str(marker.id)
                    frame_marker = "aruco/marker" + str(marker.id)
                    Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

                base_to_measured = self.get_base_to_measured(frame_marker, frame_detected, time_stamp)
                if not base_to_measured: return
                # failes
                #base_to_measured_no_broadcast = self.get_base_to_measured_no_broadcast(frame_marker, frame_detected, time_stamp)
                #self.broadcaster.sendTransform(base_to_measured_no_broadcast) # just for visualization
                time_stamp = base_to_measured.header.stamp

                # translation to be filtered
                base_to_measured_trans_odom = self.get_trans_in_odom(base_to_measured)
                translation_tbf = base_to_measured_trans_odom

                # rotation to be filtered
                odom_to_base = self.get_odom_to_base(time_stamp)
                odom_to_odom_measured_rot = self.get_odom_to_odom_measured_rotation(odom_to_base.transform.rotation, 
                                                                                    base_to_measured.transform.rotation)
                rotation_tbf = odom_to_odom_measured_rot

                maha_dist = self.mahalanobis_dist(translation_tbf, rotation_tbf, Q)
                
                print("Mahalanobis dist (kinda): {}".format(maha_dist))
                if maha_dist > 200.7:
                    # outlier
                    
                    print("Outlier")
                    return
                
                #K = 0.5
                #K = np.eye(6)*K
                K = self.kf.kalman_gain(Q)
                #K = [True, True, True, False, False, True]*K
                kalman_gains.append(K)
                filtered_translation, filtered_rot = self.filter_trans_and_rot(base_to_measured_trans_odom, 
                                                                            odom_to_odom_measured_rot, 
                                                                            K.diagonal())

                odom_to_filtered = self.get_odom_to_filtered(odom_to_base.transform.translation, 
                                                             filtered_translation, 
                                                             filtered_rot, 
                                                             frame_marker)
                odom_to_filtered.header.stamp = time_stamp
                #self.broadcaster.sendTransform(odom_to_filtered)
                odom_to_filtered_transforms.append(odom_to_filtered)

            if odom_to_filtered_transforms:
                # filtered to cf1/odom_filtered
                odom_to_filtered = self.average_transforms(odom_to_filtered_transforms)
                K = sum(kalman_gains)/len(odom_to_filtered_transforms)
                #print(K.round(2))
                #K = kalman_gains[-1]
                #odom_to_filtered.transform.rotation = odom_to_filtered_transforms[-1].transform.rotation

                odom_to_filtered.header = odom_to_filtered_transforms[-1].header
                odom_to_filtered.header.frame_id = "cf1/odom"
                odom_to_filtered.child_frame_id = "cf1/base_link/filtered"
                self.broadcaster.sendTransform(odom_to_filtered)

                filtered_to_odom_filtered = TransformStamped() # use this when using all measurements
                filtered_to_odom_filtered.header.stamp = time_stamp
                filtered_to_odom_filtered.header.frame_id = "cf1/base_link/filtered"
                filtered_to_odom_filtered.child_frame_id = "cf1/odom/filtered"
                filtered_to_odom_filtered.transform.translation.x = -odom_to_base.transform.translation.x
                filtered_to_odom_filtered.transform.translation.y = -odom_to_base.transform.translation.y
                filtered_to_odom_filtered.transform.translation.z = -odom_to_base.transform.translation.z
                filtered_to_odom_filtered.transform.rotation.w = 1
                self.broadcaster.sendTransform(filtered_to_odom_filtered)

                # map to odom
                try: 
                     # Might need to use rospy.Time(0)
                    map_to_odom_filtered = self.tf_buf.lookup_transform("map", "cf1/odom/filtered", time_stamp, rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                    print(e)
                    print("Transform lookup between map and cf1/odom/filtered failed")
                    return
                else:
                    map_to_odom_filtered.header.frame_id = "map"
                    map_to_odom_filtered.child_frame_id = "cf1/odom"
                    map_to_odom = map_to_odom_filtered
                    map_to_odom.header.stamp = rospy.Time.now()
                    print("Used {}/{} markers measurements".format(len(odom_to_filtered_transforms), n_markers))
                    #rospy.loginfo("Created new transform between map and cf1/odom")
                    K = sum(kalman_gains)/len(odom_to_filtered_transforms)
                    self.kf.update_with_gain(K)
                    #self.last_transform = map_to_odom       

                    return map_to_odom

    def get_odom_to_filtered(self, odom_to_base_translation, filtered_translation, filtered_rot, frame_marker):
        tot_translation = Vector3()
        tot_translation.x = odom_to_base_translation.x + filtered_translation.x
        tot_translation.y = odom_to_base_translation.y + filtered_translation.y
        tot_translation.z = odom_to_base_translation.z + filtered_translation.z

        odom_to_filtered = TransformStamped()
        #odom_to_filtered.header.stamp = time_stamp
        odom_to_filtered.header.frame_id = "cf1/odom"
        odom_to_filtered.child_frame_id = frame_marker + "_cf1/base_link/filtered"
        odom_to_filtered.transform.translation = tot_translation
        odom_to_filtered.transform.rotation = filtered_rot
        return odom_to_filtered

    def get_odom_to_odom_measured_rotation(self, odom_to_base_rot, base_to_measured_rot):
        
        q_odom_to_base = [odom_to_base_rot.x, 
                          odom_to_base_rot.y, 
                          odom_to_base_rot.z, 
                          odom_to_base_rot.w]

        q_base_to_measured = [base_to_measured_rot.x, 
                              base_to_measured_rot.y, 
                              base_to_measured_rot.z, 
                              base_to_measured_rot.w]

        q_measured_to_odom_measured = q_odom_to_base[:3] + [-q_odom_to_base[3]]

        q = quaternion_multiply(q_odom_to_base, q_base_to_measured)
        q_odom_to_odom_measured = quaternion_multiply(q, q_measured_to_odom_measured)
        q_odom_to_odom_measured = Quaternion(*q_odom_to_odom_measured)
        
        return q_odom_to_odom_measured

    def get_trans_in_odom(self, transform):
        trans_odom = Vector3Stamped()
        trans_odom.header = transform.header
        trans_odom.vector = transform.transform.translation
        trans_odom = self.tf_buf.transform(trans_odom, "cf1/odom")
        return trans_odom.vector

    def get_odom_to_base(self, time_stamp):
        try:
            # Might need to use rospy.Time(0)
            odom_to_base = self.tf_buf.lookup_transform("cf1/odom", "cf1/base_link", time_stamp)
        except:
            print("Failed to lookup transform cf1/odom to cf1/base_link")
            return
        return odom_to_base


    def get_base_to_measured(self, frame_marker, frame_detected, time_stamp):
        """
        Simple bad version
        """
        try: 
            # Might need to use rospy.Time(0)
            detected_to_base = self.tf_buf.lookup_transform(frame_detected, "cf1/base_link", time_stamp, rospy.Duration(1.0))
        except:
            print("Failed to transform between {} and {}".format(frame_marker, frame_detected))
            return
        marker_to_measured = detected_to_base
        marker_to_measured.header.frame_id = frame_marker
        marker_to_measured.child_frame_id = frame_marker + "_cf1/base_link_measured"
        self.broadcaster.sendTransform(marker_to_measured)
        
        try:
            """rospy.Time(0) is not failing but it takes the wrong transform
            """
            time_stamp = marker_to_measured.header.stamp
            time_stamp = rospy.Time(0)
            base_to_measured = self.tf_buf.lookup_transform("cf1/base_link", frame_marker + "_cf1/base_link_measured", time_stamp, rospy.Duration(1.0))
        except:
            print("failed to get base to measured")
            return
        #print("Broadcasted: " + str(marker_to_measured.header.stamp))
        #print("Used: " + str(base_to_measured.header.stamp))
        return base_to_measured

    def get_base_to_measured_no_broadcast(self, frame_marker, frame_detected, time_stamp):

        detected_to_base = self.tf_buf.lookup_transform(frame_detected, "cf1/base_link", rospy.Time(0), rospy.Duration(1.0))
        time_stamp = detected_to_base.header.stamp
        base_to_marker = self.tf_buf.lookup_transform("cf1/base_link", frame_marker, rospy.Time(0), rospy.Duration(1.0))
        

        # Translation
        marker_to_measured = Vector3Stamped()
        marker_to_measured.header.stamp = time_stamp
        marker_to_measured.header.frame_id = frame_marker # pretend its in the marker frame
        marker_to_measured.vector = detected_to_base.transform.translation
        # transform it to base
        marker_to_measured = self.tf_buf.transform(marker_to_measured, "cf1/base_link", rospy.Duration(1.0)).vector
        
        base_to_measured_translation = Vector3()
        base_to_measured_translation.x = base_to_marker.transform.translation.x + marker_to_measured.x
        base_to_measured_translation.y = base_to_marker.transform.translation.y + marker_to_measured.y
        base_to_measured_translation.z = base_to_marker.transform.translation.z + marker_to_measured.z

        # Rotation
        q_base_to_marker = [base_to_marker.transform.rotation.x, 
                            base_to_marker.transform.rotation.y, 
                            base_to_marker.transform.rotation.z, 
                            base_to_marker.transform.rotation.w]

        q_detected_to_base = [detected_to_base.transform.rotation.x, 
                              detected_to_base.transform.rotation.y, 
                              detected_to_base.transform.rotation.z, 
                              detected_to_base.transform.rotation.w]

        q_base_to_measured = quaternion_multiply(q_base_to_marker, q_detected_to_base)
        base_to_measured_rotation = Quaternion(*q_base_to_measured)

        # Transform
        base_to_measured = TransformStamped()
        base_to_measured.header.stamp = time_stamp
        base_to_measured.header.frame_id = "cf1/base_link"
        base_to_measured.child_frame_id = frame_marker + "_cf1/base_link_measured_no_broadcast"
        base_to_measured.transform.translation = base_to_measured_translation
        base_to_measured.transform.rotation = base_to_measured_rotation
        return base_to_measured

    def filter_trans_and_rot(self, translation, rotation, K):
        #K = K.diagonal()
        #Kx, Ky, K_rot = 0.5, 0.5, 1
        #Kz = 0.5

        translation.x *= K[0]*self.filter_config[0]
        translation.y *= K[1]*self.filter_config[1]
        translation.z *= K[2]*self.filter_config[2]

        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        a1, a2, a3 = euler_from_quaternion(q)
        a1 = K[3]*a1*self.filter_config[3] #*0 # Multiply by zero, no risk of drift
        a2 = K[4]*a2*self.filter_config[4] #*0 # Multiply by zero, no risk of drift
        a3 = K[5]*a3*self.filter_config[5]
        q_new = quaternion_from_euler(a1, a2, a3)
        rotation.x = q_new[0]
        rotation.y = q_new[1]
        rotation.z = q_new[2]
        rotation.w = q_new[3]
        
        return translation, rotation

    def mahalanobis_dist(self, translation, rotation, Q):
        s = np.matmul(np.sqrt(self.kf.cov), np.sqrt(Q)) # test this to see if it works

        rx, ry, rz = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])

        diff = np.array([translation.x, translation.y, translation.z, rx, ry, rz])
        mahalanobis_dist = np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(s)), diff))
        return mahalanobis_dist

    def _average_trans_and_rot(self, transforms):
        """
        Average of angles based on https://rosettacode.org/wiki/Averages/Mean_angle
        transforms = [(translation1, rotation1),...]
        """
        n = len(transforms)
        avg_translation = Vector3()
        avg_rotation = Quaternion()
        sin_avg = 0
        cos_avg = 0
        for trans, rot in transforms:
            avg_translation.x += t.transform.translation.x/n
            avg_translation.y += t.transform.translation.y/n
            avg_translation.z += t.transform.translation.z/n

            a1, a2, a3 = euler_from_quaternion([t.transform.rotation.x,
                                               t.transform.rotation.y,
                                               t.transform.rotation.z,
                                               t.transform.rotation.w])

            a = np.array([a1, a2, a3])
            sin_avg += 1/n*np.sin(a)
            cos_avg += 1/n*np.cos(a)

        ax, ay, az = np.arctan2(sin_avg, cos_avg)
        curr1, curr2, curr3 = euler_from_quaternion([avg_transform.transform.rotation.x,
                                                avg_transform.transform.rotation.y,
                                                avg_transform.transform.rotation.z,
                                                avg_transform.transform.rotation.w])

        new1, new2, new3 = curr1+ax, curr2+ay, curr3+az
        qx, qy, qz, qw = quaternion_from_euler(new1, new2, new3)
        avg_rotation.x = qx
        avg_rotation.y = qy
        avg_rotation.z = qz
        avg_rotation.w = qw

        return avg_transform

    def average_transforms(self, transforms):
        """
        Average of angles based on https://rosettacode.org/wiki/Averages/Mean_angle
        """
        n = len(transforms)
        avg_transform = TransformStamped()
        sin_avg = 0
        cos_avg = 0
        for t in transforms:
            avg_transform.transform.translation.x += t.transform.translation.x/n
            avg_transform.transform.translation.y += t.transform.translation.y/n
            avg_transform.transform.translation.z += t.transform.translation.z/n
            a1, a2, a3 = euler_from_quaternion([t.transform.rotation.x,
                                               t.transform.rotation.y,
                                               t.transform.rotation.z,
                                               t.transform.rotation.w])

            a = np.array([a1, a2, a3])
            #print(a)
            a += 2*np.pi*(a < 0) # set angles between 0 and 2*pi, might not make a difference, havent tried
            sin_avg += 1.0/n*np.sin(a) # fucking python2, 1.0 (float) is really important here
            cos_avg += 1.0/n*np.cos(a) # fucking python2, 1.0 (float) is really important here

        ax, ay, az = np.arctan2(sin_avg, cos_avg)
        new1, new2, new3 = ax, ay, az
        qx, qy, qz, qw = quaternion_from_euler(new1, new2, new3)
        avg_transform.transform.rotation.x = qx
        avg_transform.transform.rotation.y = qy
        avg_transform.transform.rotation.z = qz
        avg_transform.transform.rotation.w = qw

        return avg_transform
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


    # update freq should be high (at least > 20 Hz) so transforms don't get extrapolation errors
    # should not be to high since the spin() method needs to keep up
    p = MapOdomUpdate(init_trans=init_t, update_freq=200)
    rospy.sleep(1)
    p.spin()
