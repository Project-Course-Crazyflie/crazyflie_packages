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
from geometry_msgs.msg import TransformStamped, Vector3Stamped#, QuaternionStamped
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

    def update(self, K):
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

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, queue_size=1) # should there be a queue_size?
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
       
        self.update_freq = update_freq
        self.kf = KalmanFilter(initial_cov=np.array([100000000.01, 100000000.01, 100000000.01]), R=np.array([.0005, .0005, .001]), delta_t=1.0/self.update_freq)
        #self.kf = KalmanFilter(initial_cov=np.array([0.1, 0.1, 0.1]), R=np.array([0.005, 1.0, 1.0]), delta_t=1.0/self.update_freq)

        

    def spin(self):
        rate = rospy.Rate(self.update_freq)
        while not rospy.is_shutdown():

            A = np.diag([1, 1, 1])
            u = [0, 0, 0]
            self.kf.predict(A, u)
            #if self.measurement_msg: 
            #    valid_marker = self.update(self.msg_to_measurement(self.measurement_msg))
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
                p.pose.covariance[-1] = self.kf.cov[2][2]
                self.cf1_pose_cov_pub.publish(p)
            rate.sleep()

    def cf1_pose_callback(self, msg):
        self.cf1_pose = msg

    def cf1_vel_callback(self, msg):
        self.cf1_vel = msg

    def msg_to_measurement(self, msg):
        # TODO
        return msg

    def _update_callback(self, m_array):
        """
        Calculates and filters the measurements given by the markers in m_array. The measurements will be averaged and result in ONE
        update of the kalman filter.
        Doing the averaging like follows because the observation noise can be different for each measurement.
        NOTE: When doing a quick turn while receiving measurements, the angle will jump. This jump depends on
        """
        if self.is_measuring:
            # Dont want computational overload
            return
        if self.old_msg == m_array:
            # Message old 
            return
        self.is_measuring = True
        self.old_msg = m_array
        
        map_to_map_filtered_transforms = []
        kalman_gains = []
        n_markers = len(m_array.markers)
        for marker in m_array.markers:
            # TODO: make this general (no hardcoded Qs)            
            if marker.id > 9: 
                frame_detected = "sign_detected/stop"
                frame_marker = "sign/stop"
                print("Using stop sign!!!")
                Q = np.diag([0.5, 0.5, 0.5])
            else: 
                frame_detected = "aruco/detected" + str(marker.id)
                frame_marker = "aruco/marker" + str(marker.id)
                Q = np.diag([0.1, 0.1, 0.2])

            marker_to_detected = self.get_measured_transform(frame_marker, frame_detected)
            self.get_detected_map(frame_marker, frame_detected) ### remove
            if not marker_to_detected:
                # continue?
                return

            if self.outlier(marker_to_detected, Q): 
                print("Mahalanobis dist (kinda): {}".format(self.mahalanobis_dist(marker_to_detected, Q)))
                continue
            
            K = self.kf.kalman_gain(Q)
            kalman_gains.append(K)
            marker_to_filtered = self.filter_transform(marker_to_detected, K)

            name_filtered = frame_marker + "_filtered"
            marker_to_filtered.header.frame_id = frame_marker
            marker_to_filtered.child_frame_id = name_filtered
            self.broadcaster.sendTransform(marker_to_filtered)
            
            filtered_to_filtered_map_name = name_filtered + "_map"
            filtered_to_filtered_map = self.tf_buf.lookup_transform(frame_marker, "map", marker_to_filtered.header.stamp, rospy.Duration(1.0))
            filtered_to_filtered_map.header.frame_id = name_filtered
            filtered_to_filtered_map.child_frame_id = filtered_to_filtered_map_name
            filtered_to_filtered_map.header.stamp =  marker_to_filtered.header.stamp
            self.static_broadcaster.sendTransform(filtered_to_filtered_map)
            
            try:
                map_to_map_filtered = self.tf_buf.lookup_transform("map", filtered_to_filtered_map_name, marker_to_filtered.header.stamp, rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("Measurement transform failed")
                continue
            map_to_map_filtered_transforms.append(map_to_map_filtered)

            
            m = self.mahalanobis_dist(marker_to_detected, Q)
            c = self.mahalanobis_dist(marker_to_filtered, Q)
            c2 = marker_to_filtered.transform.translation.x/marker_to_detected.transform.translation.x
            print("Mahalanobis dist (kinda): {}".format(m))
            print("Convergence factor: {}".format(c))
            print("Convergence ratio: {}".format(m/c))
            print("Convergence ratio2: {}".format(1-c2))
            

        if map_to_map_filtered_transforms:
            
            
            print("Using {}/{} markers measurements".format(len(map_to_map_filtered_transforms), n_markers))
            # Average all kalman gains
            K = sum(kalman_gains)/len(map_to_map_filtered_transforms)
            #self.kf.update(K)

            map_to_map_filtered = self.average_transforms(map_to_map_filtered_transforms)
            map_to_map_filtered.header = map_to_map_filtered_transforms[-1].header
            #map_to_map_filtered = map_to_map_filtered_transforms[-1] ### remove
            
            map_to_map_filtered.header.frame_id = "map"
            map_to_map_filtered.child_frame_id = "map_filtered"
            
            self.broadcaster.sendTransform(map_to_map_filtered)

            # Filtered map to cf1/odom redefines new map to cf1/odom
            try: 
                map_filtered_to_odom = self.tf_buf.lookup_transform("map_filtered", "cf1/odom", map_to_map_filtered.header.stamp, rospy.Duration(1.0)) # rospy.Time(0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                print(e)
                print("Transform lookup between map_filtered and cf1/odom failed")
            else:
                map_filtered_to_odom.header.frame_id = "map"
                map_filtered_to_odom.child_frame_id = "cf1/odom"
                map_to_odom = map_filtered_to_odom
                map_to_odom.header.stamp = rospy.Time.now()

                #rospy.loginfo("Created new transform between map and cf1/odom")
                #self.last_transform = map_to_odom

        self.is_measuring = False

    def _update_callback(self, m_array):
                """
                Trying to find axis of rotation
                Calculates and filters the measurements given by the markers in m_array. The measurements will be averaged and result in ONE
                update of the kalman filter.
                Doing the averaging like follows because the observation noise can be different for each measurement.
                NOTE: When doing a quick turn while receiving measurements, the angle will jump. This jump depends on
                """
                if self.is_measuring:
                    # Dont want computational overload
                    return
                if self.old_msg == m_array:
                    # Message old 
                    return
                #self.is_measuring = True
                self.old_msg = m_array
                
                map_to_map_filtered_transforms = []
                kalman_gains = []
                n_markers = len(m_array.markers)
                for marker in m_array.markers:
                    # TODO: make this general (no hardcoded Qs)            
                    if marker.id > 9: 
                        frame_detected = "sign_detected/stop"
                        frame_marker = "sign/stop"
                        print("Using stop sign!!!")
                        Q = np.diag([0.5, 0.5, 0.5])
                    else: 
                        frame_detected = "aruco/detected" + str(marker.id)
                        frame_marker = "aruco/marker" + str(marker.id)
                        Q = np.diag([0.1, 0.1, 0.2])

                    marker_to_detected = self.get_measured_transform(frame_marker, frame_detected)
                    #map_to_map_detected = self.get_detected_map(frame_marker, frame_detected)
                    time_stamp = marker_to_detected.header.stamp


                    translation_detected = Vector3Stamped()
                    translation_detected.header.frame_id = frame_marker
                    translation_detected.header.stamp = time_stamp
                    translation_detected.vector = marker_to_detected.transform.translation
                    translation_map = self.tf_buf.transform(translation_detected, "map").vector

                    # Filter
                    # TODO: change filter_transform to take a translation and a rotation instead

                    K = self.kf.kalman_gain(Q)
                    kalman_gains.append(K)
                    measured_transform = TransformStamped()
                    measured_transform.transform.translation = translation_map
                    measured_transform.transform.rotation = marker_to_detected.transform.rotation
                    marker_to_filtered = self.filter_transform(measured_transform, K)
                    #marker_to_filtered = measured_transform
                    #/Filter

                    translation_filtered_map = Vector3Stamped()
                    translation_filtered_map.header.stamp = time_stamp
                    translation_filtered_map.header.frame_id = "map"
                    translation_filtered_map.vector = marker_to_filtered.transform.translation

                    translation_filtered = self.tf_buf.transform(translation_filtered_map, frame_marker).vector
                    rotation_filtered = marker_to_filtered.transform.rotation

                    #Copied#######################################
                    name_filtered = frame_marker + "_filtered"
                    marker_to_filtered = TransformStamped()
                    marker_to_filtered.header.stamp = time_stamp
                    marker_to_filtered.header.frame_id = frame_marker
                    marker_to_filtered.child_frame_id = name_filtered
                    marker_to_filtered.transform.translation = translation_filtered
                    marker_to_filtered.transform.rotation = rotation_filtered
                    self.broadcaster.sendTransform(marker_to_filtered)
                    
                    filtered_to_filtered_map_name = name_filtered + "_map"
                    filtered_to_filtered_map = self.tf_buf.lookup_transform(frame_marker, "map", time_stamp, rospy.Duration(1.0))
                    filtered_to_filtered_map.header.frame_id = name_filtered
                    filtered_to_filtered_map.child_frame_id = filtered_to_filtered_map_name
                    #filtered_to_filtered_map.header.stamp =  time_stamp
                    self.static_broadcaster.sendTransform(filtered_to_filtered_map)
                    
                    try:
                        map_to_map_filtered = self.tf_buf.lookup_transform("map", filtered_to_filtered_map_name, time_stamp, rospy.Duration(1.0))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        print("Measurement transform failed")
                        continue

                    
                    map_to_map_filtered_transforms.append(map_to_map_filtered)
                    ########################################
                    
                    print("Mahalanobis dist (kinda): {}".format(self.mahalanobis_dist(marker_to_detected, Q)))
                    if self.outlier(marker_to_detected, Q): 
                        #print("Mahalanobis dist (kinda): {}".format(self.mahalanobis_dist(marker_to_detected, Q)))
                        continue
                    
                   

                    

                if map_to_map_filtered_transforms:
                    
                    print("Using {}/{} markers measurements".format(len(map_to_map_filtered_transforms), n_markers))
                    # Average all kalman gains
                    K = sum(kalman_gains)/len(map_to_map_filtered_transforms)
                    self.kf.update(K)

                    map_to_map_filtered = self.average_transforms(map_to_map_filtered_transforms)
                    map_to_map_filtered.header = map_to_map_filtered_transforms[-1].header
                    #map_to_map_filtered = map_to_map_filtered_transforms[-1] ### remove
                    
                    map_to_map_filtered.header.frame_id = "map"
                    map_to_map_filtered.child_frame_id = "map_filtered"

                    #map_to_map_filtered = self.remove_xy_rot(map_to_map_filtered)
                    self.broadcaster.sendTransform(map_to_map_filtered)

                    # Filtered map to cf1/odom redefines new map to cf1/odom
                    try: 
                        map_filtered_to_odom = self.tf_buf.lookup_transform("map_filtered", "cf1/odom", map_to_map_filtered.header.stamp, rospy.Duration(1.0)) # rospy.Time(0)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                        print(e)
                        print("Transform lookup between map_filtered and cf1/odom failed")
                    else:
                        map_filtered_to_odom.header.frame_id = "map"
                        map_filtered_to_odom.child_frame_id = "cf1/odom"
                        map_to_odom = map_filtered_to_odom
                        map_to_odom.header.stamp = rospy.Time.now()

                        #rospy.loginfo("Created new transform between map and cf1/odom")
                        self.last_transform = map_to_odom     
            

                self.is_measuring = False

    def __update_callback(self, m_array):
            """
            Complicated one
            Calculates and filters the measurements given by the markers in m_array. The measurements will be averaged and result in ONE
            update of the kalman filter.
            Doing the averaging like follows because the observation noise can be different for each measurement.
            NOTE: When doing a quick turn while receiving measurements, the angle will jump. This jump depends on
            """
            if self.is_measuring:
                # Dont want computational overload
                return
            if self.old_msg == m_array:
                # Message old 
                return
            #self.is_measuring = True
            self.old_msg = m_array
            
            map_to_map_filtered_transforms = []
            kalman_gains = []
            n_markers = len(m_array.markers)
            for marker in m_array.markers:
                # TODO: make this general (no hardcoded Qs)            
                if marker.id > 9: 
                    frame_detected = "sign_detected/stop"
                    frame_marker = "sign/stop"
                    print("Using stop sign!!!")
                    Q = np.diag([0.5, 0.5, 0.5])
                else: 
                    frame_detected = "aruco/detected" + str(marker.id)
                    frame_marker = "aruco/marker" + str(marker.id)
                    Q = np.diag([0.1, 0.1, 0.1])

                marker_to_detected = self.get_measured_transform(frame_marker, frame_detected)
                map_to_map_detected = self.get_detected_map(frame_marker, frame_detected)
                time_stamp = marker_to_detected.header.stamp

                map_to_map_detected_no_xy_rot = TransformStamped()
                map_to_map_detected_no_xy_rot.header.stamp = time_stamp
                map_to_map_detected_no_xy_rot.header.frame_id = "map"
                detected_no_xy_rot_name = map_to_map_detected.child_frame_id + "_no_xy_rot"
                map_to_map_detected_no_xy_rot.child_frame_id = detected_no_xy_rot_name
                rot = map_to_map_detected.transform.rotation
                _, _, a3 = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
                rotation = quaternion_from_euler(0, 0, a3)
                map_to_map_detected_no_xy_rot.transform.rotation = Quaternion(*rotation)
                self.broadcaster.sendTransform(map_to_map_detected_no_xy_rot)
                
                map_det_no_xy_rot_to_marker_no_xy_rot = self.tf_buf.lookup_transform("map", frame_marker, time_stamp)
                map_det_no_xy_rot_to_marker_no_xy_rot.header.frame_id = detected_no_xy_rot_name
                marker_no_xy_rot_name = frame_detected + "_no_xy_rot"
                map_det_no_xy_rot_to_marker_no_xy_rot.child_frame_id = marker_no_xy_rot_name
                self.static_broadcaster.sendTransform(map_det_no_xy_rot_to_marker_no_xy_rot)

                marker_no_xy_rot_to_detected_actual = map_det_no_xy_rot_to_marker_no_xy_rot
                marker_no_xy_rot_to_detected_actual.header.frame_id = marker_no_xy_rot_name
                detected_actual_name = frame_detected + "_actual"
                marker_no_xy_rot_to_detected_actual.child_frame_id = detected_actual_name
                marker_no_xy_rot_to_detected_actual.transform.translation = self.tf_buf.lookup_transform(marker_no_xy_rot_name, frame_detected, time_stamp, rospy.Duration(1.0)).transform.translation
                marker_no_xy_rot_to_detected_actual.transform.rotation.x = 0
                marker_no_xy_rot_to_detected_actual.transform.rotation.y = 0
                marker_no_xy_rot_to_detected_actual.transform.rotation.z = 0
                marker_no_xy_rot_to_detected_actual.transform.rotation.w = 1
                self.broadcaster.sendTransform(marker_no_xy_rot_to_detected_actual)


                translation_detected = Vector3Stamped()
                translation_detected.header = marker_to_detected.header
                translation_detected_map = self.tf_buf.transform(translation_detected, "map")

                marker_to_filtered = marker_to_detected



                #Copied#######################################
                """
                name_filtered = frame_marker + "_filtered"
                marker_to_filtered = TransformStamped()
                marker_to_filtered.header.stamp = time_stamp
                marker_to_filtered.header.frame_id = frame_marker
                marker_to_filtered.child_frame_id = name_filtered
                marker_to_filtered.transform.translation = translation_filtered
                marker_to_filtered.transform.rotation = rotation_filtered
                self.broadcaster.sendTransform(marker_to_filtered)
                
                filtered_to_filtered_map_name = name_filtered + "_map"
                filtered_to_filtered_map = self.tf_buf.lookup_transform(frame_marker, "map", time_stamp, rospy.Duration(1.0))
                filtered_to_filtered_map.header.frame_id = name_filtered
                filtered_to_filtered_map.child_frame_id = filtered_to_filtered_map_name
                #filtered_to_filtered_map.header.stamp =  time_stamp
                self.static_broadcaster.sendTransform(filtered_to_filtered_map)
                
                try:
                    map_to_map_filtered = self.tf_buf.lookup_transform("map", filtered_to_filtered_map_name, time_stamp, rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print("Measurement transform failed")
                    continue
                map_to_map_filtered_transforms.append(map_to_map_filtered)
                ########################################
                """


                if self.outlier(marker_to_detected, Q): 
                    print("Mahalanobis dist (kinda): {}".format(self.mahalanobis_dist(marker_to_detected, Q)))
                    continue
                
                K = self.kf.kalman_gain(Q)
                kalman_gains.append(K)

                

            if map_to_map_filtered_transforms:
                pass       
         

            self.is_measuring = False

    def update_callback(self, m_array):
            """
            Projected map method, finally working??!?
            """
            if self.is_measuring:
                # Dont want computational overload
                return
            if self.old_msg == m_array:
                # Message old 
                return
            #self.is_measuring = True
            self.old_msg = m_array
            
            map_to_map_filtered_proj_transforms = []
            kalman_gains = []
            n_markers = len(m_array.markers)
            for marker in m_array.markers:
                # TODO: make this general (no hardcoded Qs)            
                if marker.id > 9: 
                    frame_detected = "sign_detected/stop"
                    frame_marker = "sign/stop"
                    print("Using stop sign!!!")
                    Q = np.diag([0.5, 0.5, 0.5])
                else: 
                    frame_detected = "aruco/detected" + str(marker.id)
                    frame_marker = "aruco/marker" + str(marker.id)
                    Q = np.diag([0.1, 0.1, 0.1])

                marker_to_detected = self.get_measured_transform(frame_marker, frame_detected) # for translation
                map_to_map_detected = self.get_detected_map(frame_marker, frame_detected) # for rotation
                
                time_stamp = marker_to_detected.header.stamp


                ### marker to projected filtered map ###
                marker_to_proj_filt_map = TransformStamped()
                marker_to_proj_filt_map.header.stamp = time_stamp
                marker_to_proj_filt_map.header.frame_id = frame_marker

                translation = Vector3Stamped()
                translation.header.stamp = time_stamp
                translation.header.frame_id = frame_marker
                translation.vector = marker_to_detected.transform.translation
                
                translation = self.tf_buf.transform(translation, "map").vector
                
                # filtering
                # translation
                #translation.x /=2
                #translation.y /=2
                #translation.z = 0
                #rotation
                #rotation = self.remove_xy_rot(map_to_map_detected).transform.rotation
                K = self.kf.kalman_gain(Q)
                kalman_gains.append(K)
                translation, rotation = self.filter_translation_and_rot(translation, map_to_map_detected.transform.rotation, K)


                # Projected filtered map
                map_to_marker = self.tf_buf.lookup_transform("map", frame_marker, time_stamp)
                translation_origin = map_to_marker.transform.translation
                translation_origin.z = 0

                tot_translation = translation
                tot_translation.x += translation_origin.x
                tot_translation.y += translation_origin.y
                tot_translation.z += translation_origin.z

                map_to_map_filtered_proj = TransformStamped()
                map_to_map_filtered_proj.header.stamp = time_stamp
                map_to_map_filtered_proj.header.frame_id = "map"
                map_to_map_filtered_proj.child_frame_id = frame_marker + "_map_projected_filtered"
                map_to_map_filtered_proj.transform.translation = tot_translation
                map_to_map_filtered_proj.transform.rotation = rotation
                

                self.broadcaster.sendTransform(map_to_map_filtered_proj)
                map_to_map_filtered_proj_transforms.append((map_to_map_filtered_proj, translation_origin))
                

                if self.outlier(marker_to_detected, Q): 
                    print("Mahalanobis dist (kinda): {}".format(self.mahalanobis_dist(marker_to_detected, Q)))
                    continue
                
                

                

            if map_to_map_filtered_proj_transforms:
                    
                    print("Using {}/{} markers measurements".format(len(map_to_map_filtered_proj_transforms), n_markers))
                    # Average all kalman gains
                    K = sum(kalman_gains)/len(map_to_map_filtered_proj_transforms)
                    
                    self.kf.update(K)
                    
                    map_to_map_filtered_proj, translation_origin = map_to_map_filtered_proj_transforms[-1] # remove when using all measurements
                    # new map
                    map_filtered_proj_to_map_filtered = TransformStamped() # use this when using all measurements
                    map_filtered_proj_to_map_filtered.header.stamp = time_stamp
                    map_filtered_proj_to_map_filtered.header.frame_id = map_to_map_filtered_proj.child_frame_id
                    map_filtered_proj_to_map_filtered.child_frame_id = "map_filtered"
                    map_filtered_proj_to_map_filtered.transform.translation.x = -translation_origin.x
                    map_filtered_proj_to_map_filtered.transform.translation.y = -translation_origin.y
                    map_filtered_proj_to_map_filtered.transform.translation.z = -translation_origin.z
                    map_filtered_proj_to_map_filtered.transform.rotation.w = 1
                    self.broadcaster.sendTransform(map_filtered_proj_to_map_filtered)

                    # Filtered map to cf1/odom redefines new map to cf1/odom
                    try: 
                        map_filtered_to_odom = self.tf_buf.lookup_transform("map_filtered", "cf1/odom", time_stamp, rospy.Duration(1.0)) # rospy.Time(0)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                        print(e)
                        print("Transform lookup between map_filtered and cf1/odom failed")
                    else:
                        map_filtered_to_odom.header.frame_id = "map"
                        map_filtered_to_odom.child_frame_id = "cf1/odom"
                        map_to_odom = map_filtered_to_odom
                        map_to_odom.header.stamp = rospy.Time.now()

                        #rospy.loginfo("Created new transform between map and cf1/odom")
                        self.last_transform = map_to_odom       
         

            self.is_measuring = False

    def get_measured_transform(self, frame_marker, frame_detected):
        try: 
            transform = self.tf_buf.lookup_transform(frame_marker, frame_detected, rospy.Time(0), rospy.Duration(1.0))
        except:
            print("Failed to transform between {} and {}".format(frame_marker, frame_detected))
            return
        return transform


    def get_detected_map(self, frame_marker, frame_detected):
        """
        Returns the transform between 'map' and 'map_measured' where map_measured is the map corresponding to frame_detected.
        """
        # OBS!: Transform between aruco/markedX and aruco/detectedX is not the same as transform between map and map_measured!!! (aruco markers can have different orientation than map)
        # Transform between marker and map is the same as detected to measured map
        try: 
            detected_to_map_measured = self.tf_buf.lookup_transform(frame_marker, "map", rospy.Time(0), rospy.Duration(1.0))
        except: 
            print("walla0")
            return

        map_detected_frame = frame_detected + "_map"

        detected_to_map_measured.header.frame_id = frame_detected
        detected_to_map_measured.child_frame_id = map_detected_frame
        self.static_broadcaster.sendTransform(detected_to_map_measured) # Don't work with dynamic broadcaster, something with stamps?
        
        # Transform between map and measured map
        try: 
            map_to_map_measured = self.tf_buf.lookup_transform("map", map_detected_frame, rospy.Time(0), rospy.Duration(1.0)) # rospy.Time(0)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
            print(e)
            print("walla1")
            return
        
        return map_to_map_measured



    def remove_xy_rot(self, t):
        _,_,az = euler_from_quaternion([t.transform.rotation.x,
                                        t.transform.rotation.y,
                                        t.transform.rotation.z,
                                        t.transform.rotation.w])

        q = quaternion_from_euler(0, 0, az)
        t.transform.rotation = Quaternion(*q)
        return t

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
            sin_avg += 1/n*np.sin(a)
            cos_avg += 1/n*np.cos(a)

        ax, ay, az = np.arctan2(sin_avg, cos_avg)
        curr1, curr2, curr3 = euler_from_quaternion([avg_transform.transform.rotation.x,
                                                avg_transform.transform.rotation.y,
                                                avg_transform.transform.rotation.z,
                                                avg_transform.transform.rotation.w])

        new1, new2, new3 = curr1+ax, curr2+ay, curr1+az
        qx, qy, qz, qw = quaternion_from_euler(new1, new2, new3)
        avg_transform.transform.rotation.x = qx
        avg_transform.transform.rotation.y = qy
        avg_transform.transform.rotation.z = qz
        avg_transform.transform.rotation.w = qw

        return avg_transform

    def filter_transform(self, transform, K):
        # TODO: Kalman filter, for now fixed kalman gain
        
        Kx, Ky, K_rot = K.diagonal()
        #Kx, Ky, K_rot = 0.5, 0.5, 1
        t = transform
        
        new_t = TransformStamped()
        new_t.header.stamp = t.header.stamp
        new_t.header.frame_id = "map"
        new_t.child_frame_id = "map_filtered"
        new_t.transform = transform.transform # remove
        return new_t # remove
        new_t.transform.translation.x = Kx*t.transform.translation.x
        new_t.transform.translation.y = Ky*t.transform.translation.y
        new_t.transform.translation.z = (Kx+Ky)/2*t.transform.translation.z # Keep z in case aruco markers are ill placed irl


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
        
    def filter_translation_and_rot(self, translation, rotation, K):
        # TODO: Kalman filter, for now fixed kalman gain
        
        Kx, Ky, K_rot = K.diagonal()
        #Kx, Ky, K_rot = 0.5, 0.5, 1

        translation.x *= Kx
        translation.y *= Ky
        translation.z *= 0

        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        a1, a2, a3 = euler_from_quaternion(q)
        a1 = K_rot*a1 *0 # Multiply by zero, no risk of drift
        a2 = K_rot*a2 *0 # Multiply by zero, no risk of drift
        a3 = K_rot*a3
        q_new = quaternion_from_euler(a1, a2, a3)
        rotation.x = q_new[0]
        rotation.y = q_new[1]
        rotation.z = q_new[2]
        rotation.w = q_new[3]
        
        return translation, rotation

    def outlier(self, t, Q):
        # trying stop sign
        #s = self.kf.cov[:2, :2]
        #diff = np.array([t.transform.translation.x, t.transform.translation.y])
        #mahalanobis_dist = np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(s)), diff))
        #print("Mahalanobis dist: {}".format(mahalanobis_dist))
        #return mahalanobis_dist > 60

        # mahalanobis dist
        #return self.mahalanobis_dist(t, Q) > 0.5
        return self.mahalanobis_dist(t, Q) > 100

        # kullback-leiber divergence
        #s1 = self.kf.cov
        #s2 = np.array(Q)
        #d = 0.5*(np.matmul(np.linalg.inv(s1),s2).transpose() - len(s1.diagonal()) + np.log(np.linalg.det(s1) / np.linalg.det(s2)) )
        #print(d.diagonal())
        #[35, 35, 22] # thresholds
        #return False

    def mahalanobis_dist(self, t, Q):
        s = np.matmul(np.sqrt(self.kf.cov), np.sqrt(Q)) # test this to see if it works
        diff = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.rotation.z])
        mahalanobis_dist = np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(s)), diff))
        return mahalanobis_dist

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
    p = MapOdomUpdate(init_trans=init_t, update_freq=200)
    p.spin()
