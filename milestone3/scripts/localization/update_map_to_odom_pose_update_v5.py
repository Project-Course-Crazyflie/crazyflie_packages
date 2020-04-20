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
from geometry_msgs.msg import PoseArray, Point
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Vector3#, QuaternionStamped
from std_msgs.msg import Bool, Int32MultiArray
from crazyflie_driver.msg import Position
from aruco_msgs.msg import Marker, MarkerArray

import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, initial_cov, R, delta_t):
        # position/mean not neaded since it will provided by the crazyflie
        self.cov = initial_cov
        self.R = R
        self.delta_t = delta_t

        self.n_updates = 0
        self.prev_cov = self.cov.copy()

        

    def converged(self):
        return 

    def predict(self, A, v=None):
        # TODO: use dynamic process noise
        R = self.R
        if v is not None:
            v_mat = np.diag(abs(v/np.linalg.norm(v)))
            R = np.matmul(np.matmul(v_mat, self.R), v_mat.transpose())
        self.cov = np.matmul(np.matmul(A, self.cov), A.transpose()) + R*self.delta_t
        

    def kalman_gain(self, Q):
        K = np.matmul(self.cov, np.linalg.inv(self.cov + Q))
        return K

    def inovation(self, believed_state, measured_state):
        trans = (measured_state[:3]-believed_state[:3])
        rot = (measured_state[3:]-believed_state[3:])
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        return np.concatenate([trans, rot])

    def update(self, mu, inovation, Q):
        K = self.kalman_gain(Q)
        self.update_with_gain(K)
        return mu + np.matmul(K, inovation)

    def update_with_gain(self, K):
        self.cov = np.matmul((np.eye(K.shape[0])-K), self.cov)
        self.prev_cov = self.cov.copy()

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

        self.old_msg = None
        self.measurement_msg = None
        # what translation/rotation variables to use in filtering
        self.filter_config = rospy.get_param("localization/measurement_config")

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, queue_size=1) # should there be a queue_size?
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
       
        self.update_freq = update_freq
        self.kf = KalmanFilter(initial_cov=np.diag(rospy.get_param("localization/initial_cov")), 
                               R=np.diag(rospy.get_param("localization/process_noise")), 
                               delta_t=1.0/self.update_freq)
        self.converged_pub = rospy.Publisher("cf1/localization/converged", Bool, queue_size=1)  
        self.measurement_fb_pub = rospy.Publisher("cf1/localization/measurement_feedback", Int32MultiArray, queue_size=1)

        self.maha_dist_thres = rospy.get_param("localization/maha_dist_thres")
        self.cov_norm_thres =  rospy.get_param("localization/cov_norm_thres")

        self.odom_new_pub = rospy.Publisher("cf1/pose/odom_new", PoseStamped, queue_size=1)
        self.believed_pub = rospy.Publisher("cf1/pose/believed", PoseStamped, queue_size=1)
        self.measured_valid_pub = rospy.Publisher("cf1/pose/measured_valid", PoseArray, queue_size=1)
        self.measured_invalid_pub = rospy.Publisher("cf1/pose/measured_invalid", PoseArray, queue_size=1)
        self.filtered_pub = rospy.Publisher("cf1/pose/filtered", PoseStamped, queue_size=1)

    def spin(self):
        rate = rospy.Rate(self.update_freq)
        self.has_transformed = False # to check if initial transform with kalman gain 1 result in good transform
        while not rospy.is_shutdown():

            
            self.broadcaster.sendTransform(self.last_transform)
            self.last_transform.header.stamp = rospy.Time.now()
            
            A = np.diag([1, 1, 1, 1, 1, 1])
            if False and self.cf1_vel:
                v = np.array([self.cf1_vel.twist.linear.x,
                              self.cf1_vel.twist.linear.y,
                              self.cf1_vel.twist.linear.z,
                              self.cf1_vel.twist.angular.x,
                              self.cf1_vel.twist.angular.y,
                              self.cf1_vel.twist.angular.z])
                
                self.kf.predict(A, v)
            else:
                self.kf.predict(A)

            if self.measurement_msg:
                if not self.has_transformed:
                    map_to_odom = self.update(self.measurement_msg)
                    if map_to_odom: 
                        self.last_transform = map_to_odom
                        #self.has_transformed = True

            norm = np.linalg.norm(self.kf.cov)
            #print("Cov norm: {}".format(norm))
            self.converged_pub.publish(Bool(norm < self.cov_norm_thres))

            if self.cf1_pose: 
                p = PoseWithCovarianceStamped()
                p.header = self.cf1_pose.header # correct to use cf1/odom as frame_id???
                p.pose.pose = self.cf1_pose.pose
                p.pose.covariance[0] = self.kf.cov[0][0]
                #p.pose.covariance[1] = self.kf.cov[0][1]
                #p.pose.covariance[6] = self.kf.cov[1][0]
                p.pose.covariance[7] = self.kf.cov[1][1]
                p.pose.covariance[-1] = self.kf.cov[5][5]
                self.cf1_pose_cov_pub.publish(p)
            rate.sleep()

    def cf1_pose_callback(self, msg):
        self.cf1_pose = msg

    def cf1_vel_callback(self, msg):
        self.cf1_vel = msg
        
    def update_callback(self, m_array):
        if self.measurement_msg != m_array:
            self.measurement_msg = m_array

    def update(self, m_array):
        """
        Average all measurements
        """
        if self.old_msg == m_array:
            # Message old 
            return
        self.old_msg = m_array
        
        kalman_gains = []
        filtered_poses = []
        measured_valid_poses = PoseArray()
        measured_invalid_poses = PoseArray()
        measured_valid_poses.header.frame_id = "map"
        measured_invalid_poses.header.frame_id = "map"
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
                Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
            

            try:
                # just to get correct time stamp
                time_stamp = self.tf_buf.lookup_transform(frame_marker, frame_detected, rospy.Time(0)).header.stamp
            except:
                print("Wait a bit...")
                return

            measurement_fb = Int32MultiArray()
            # could this fail?
            believed_trans = self.tf_buf.lookup_transform("map", "cf1/base_link", time_stamp)
            believed_pose = PoseStamped()
            believed_pose.header = believed_trans.header
            believed_pose.pose.position = Point(*[believed_trans.transform.translation.x,
                                                  believed_trans.transform.translation.y,
                                                  believed_trans.transform.translation.z])
            believed_pose.pose.orientation = believed_trans.transform.rotation

            believed_state = self.pose_stamped_to_state(believed_pose)

            measured_pose = self.get_measured_pose_filtered(believed_pose, frame_marker, frame_detected)
            measured_state = self.pose_stamped_to_state(measured_pose)
            
            diff = self.kf.inovation(believed_state*self.filter_config, 
                                     measured_state*self.filter_config)
            maha_dist = self.maha_dist(diff, Q)
            print("Mahalanobis dist: {}".format(maha_dist))
            if maha_dist > self.maha_dist_thres:
                # outlier
                print("Outlier")
                measured_invalid_poses.poses.append(measured_pose.pose)
                measurement_fb.data = [marker.id, 0]
                self.measurement_fb_pub.publish(measurement_fb)
                #return
                continue
            else:
                measured_valid_poses.poses.append(measured_pose.pose)
                measurement_fb.data = [marker.id, 1]
                self.measurement_fb_pub.publish(measurement_fb)
                
            K = self.kf.kalman_gain(Q)
            kalman_gains.append(K)
            filtered_state = self.filter_state(believed_state, measured_state, K)
            # for testing with K=1
            #filtered_state = believed_state + self.kf.inovation(believed_state, measured_state)*self.filter_config*1

            filtered_pose = self.state_to_pose_stamped(filtered_state, believed_pose.header.frame_id, time_stamp)
            filtered_poses.append(filtered_pose)

        self.measured_valid_pub.publish(measured_valid_poses)
        self.measured_invalid_pub.publish(measured_invalid_poses)

        print("Using {}/{} markers measurements".format(len(filtered_poses), n_markers))
        if len(filtered_poses) > 0:
            K = sum(kalman_gains)/len(filtered_poses)
            self.kf.update_with_gain(K)
            filtered_pose = self.average_poses(filtered_poses)
            self.filtered_pub.publish(filtered_pose) # for visualization
            map_to_odom = self.get_map_to_odom(filtered_pose)

            #print("Updated")
            return map_to_odom


    def get_map_to_odom(self, filtered_pose):
        time_stamp = filtered_pose.header.stamp
        base_to_odom = self.tf_buf.lookup_transform("cf1/base_link", "cf1/odom", time_stamp)
        filtered_trans = TransformStamped()
        filtered_trans.header = filtered_pose.header
        filtered_trans.child_frame_id = "cf1/base_link/filtered"
        filtered_trans.transform.translation = Vector3(*[filtered_pose.pose.position.x,
                                                         filtered_pose.pose.position.y,
                                                         filtered_pose.pose.position.z])
        filtered_trans.transform.rotation = filtered_pose.pose.orientation
        self.tf_buf.set_transform(filtered_trans, "gandalfs_authority")
        filtered_to_odom = base_to_odom
        filtered_to_odom.header.frame_id = "cf1/base_link/filtered"
        filtered_to_odom.child_frame_id = "cf1/odom_new"
        self.tf_buf.set_transform(filtered_to_odom, "gandalfs_authority")
        map_to_odom = self.tf_buf.lookup_transform_core("map", "cf1/odom_new", time_stamp)
        map_to_odom.child_frame_id = "cf1/odom"
        return map_to_odom

    def get_measured_pose_filtered(self, believed_pose, frame_marker, frame_detected):
        time_stamp = believed_pose.header.stamp

        map_to_marker = self.tf_buf.lookup_transform_core("map", frame_marker, time_stamp)
        detected_map_frame_ref = frame_detected + "_map_reference"
        # not use _core here?
        map_to_detected = self.tf_buf.lookup_transform_core("map", frame_detected, time_stamp)
        map_to_detected.child_frame_id = detected_map_frame_ref
        map_to_detected.header.stamp = time_stamp

        # Disregard translational diplacement of marker detection
        if not self.filter_config[0]: map_to_detected.transform.translation.x = map_to_marker.transform.translation.x
        if not self.filter_config[1]: map_to_detected.transform.translation.y = map_to_marker.transform.translation.y
        if not self.filter_config[2]: map_to_detected.transform.translation.z = map_to_marker.transform.translation.z

        detected_angles = list(euler_from_quaternion([map_to_detected.transform.rotation.x,
                                                      map_to_detected.transform.rotation.y,
                                                      map_to_detected.transform.rotation.z,
                                                      map_to_detected.transform.rotation.w]))
        marker_angles = euler_from_quaternion([map_to_marker.transform.rotation.x,
                                               map_to_marker.transform.rotation.y,
                                               map_to_marker.transform.rotation.z,
                                               map_to_marker.transform.rotation.w])
        # Disregard rotational diplacement of marker detection
        if not self.filter_config[3]: detected_angles[0] = marker_angles[0]
        if not self.filter_config[4]: detected_angles[1] = marker_angles[1]
        if not self.filter_config[5]: detected_angles[2] = marker_angles[2]
        
        map_to_detected.transform.rotation = Quaternion(*quaternion_from_euler(*detected_angles))
        
        self.tf_buf.set_transform(map_to_detected, "gandalfs_authority")
        self.broadcaster.sendTransform(map_to_detected) # for visualization
        
        pose_in_detected = self.tf_buf.transform(believed_pose, detected_map_frame_ref)

        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = frame_marker
        measured_pose = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)

        return self.tf_buf.transform(measured_pose, "map")

    def _get_measured_pose_filtered(self, believed_pose, frame_marker, frame_detected):
        time_stamp = believed_pose.header.stamp
        measured_orientation = self.get_map_to_map_detected_rotation(frame_marker, frame_detected, time_stamp)

        # Disregard rotational diplacement of marker detection
        ax, ay, az = euler_from_quaternion([measured_orientation.x,
                                            measured_orientation.y,
                                            measured_orientation.z,
                                            measured_orientation.w])
        ax *= self.filter_config[3]
        ay *= self.filter_config[4]
        az *= self.filter_config[5]
        measured_orientation = Quaternion(*quaternion_from_euler(ax, ay, az))

        detected_map_frame = frame_detected + "_map_reference"
        # not use _core here?
        map_to_detected = self.tf_buf.lookup_transform("map", frame_detected, time_stamp)
        map_to_detected.child_frame_id = detected_map_frame
        map_to_detected.header.stamp = time_stamp
        map_to_detected.transform.rotation = measured_orientation
        self.tf_buf.set_transform(map_to_detected, "gandalfs_authority")
        self.broadcaster.sendTransform(map_to_detected) # for visualization

        marker_map_frame = frame_marker + "_map_reference"
        map_to_marker = self.tf_buf.lookup_transform_core("map", frame_marker, time_stamp)
        map_to_marker.child_frame_id = marker_map_frame
        map_to_marker.header.stamp = time_stamp
        map_to_marker.transform.rotation = Quaternion(*[0,0,0,1])
        self.tf_buf.set_transform(map_to_marker, "gandalfs_authority")
        self.broadcaster.sendTransform(map_to_marker) # for visualization
        
        pose_in_detected = self.tf_buf.transform(believed_pose, detected_map_frame)

        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = marker_map_frame
        measured_pose = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)

        # Disregard translational diplacement of marker detection
        if not self.filter_config[0]: measured_pose.pose.position.x = believed_pose.pose.position.x
        if not self.filter_config[1]: measured_pose.pose.position.y = believed_pose.pose.position.y
        if not self.filter_config[2]: measured_pose.pose.position.z = believed_pose.pose.position.z
        return self.tf_buf.transform(measured_pose, "map")

    def get_map_to_map_detected_rotation(self, frame_marker, frame_detected, time_stamp):
        # can this fail?
        map_to_marker = self.tf_buf.lookup_transform(frame_marker, "map", time_stamp)
        marker_to_detected = self.tf_buf.lookup_transform(frame_marker, frame_detected, time_stamp)

        map_to_marker_rot = [map_to_marker.transform.rotation.x,
                             map_to_marker.transform.rotation.y,
                             map_to_marker.transform.rotation.z,
                             map_to_marker.transform.rotation.w]
        detected_to_map_detected = map_to_marker_rot[:3] + [-map_to_marker_rot[3]]
        marker_to_detected_rot = [marker_to_detected.transform.rotation.x,
                                  marker_to_detected.transform.rotation.y,
                                  marker_to_detected.transform.rotation.z,
                                  marker_to_detected.transform.rotation.w]

        rotation = quaternion_multiply(marker_to_detected_rot, map_to_marker_rot)
        rotation = quaternion_multiply(detected_to_map_detected, rotation)
        
        return Quaternion(*rotation)


    def filter_state(self, believed_state, measured_state, K):
        K = K.diagonal()
        pos = believed_state[:3] + (measured_state[:3]-believed_state[:3])*K[:3]

        rot = (measured_state[3:]-believed_state[3:])
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        rot = believed_state[3:] + rot*K[3:]
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        
        return np.concatenate([pos, rot])

    def pose_stamped_to_state(self, pose):
        rx, ry, rz = euler_from_quaternion([pose.pose.orientation.x, 
                                            pose.pose.orientation.y, 
                                            pose.pose.orientation.z, 
                                            pose.pose.orientation.w])
        return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, rx, ry, rz])

    def state_to_pose_stamped(self, state, frame_id, time_stamp):
        p = PoseStamped()
        p.header.stamp = time_stamp
        p.header.frame_id = frame_id
        (p.pose.position.x,
         p.pose.position.y,
         p.pose.position.z) = state[:3]

        q_ls = quaternion_from_euler(*state[3:])
        q = Quaternion(*q_ls)
        p.pose.orientation = q
        return p

    def maha_dist(self, diff, Q):
        #cov = self.kf.cov*self.filter_config
        s = self.kf.cov.copy()
        return np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(s)), diff))
    

    def average_poses(self, poses):
        avg_pose = PoseStamped()
        avg_pose.header = poses[0].header
        n = len(poses)
        sin_avg = 0
        cos_avg = 0
        for p in poses:
            avg_pose.pose.position.x += p.pose.position.x/n
            avg_pose.pose.position.y += p.pose.position.y/n
            avg_pose.pose.position.z += p.pose.position.z/n
            a1, a2, a3 = euler_from_quaternion([p.pose.orientation.x,
                                                p.pose.orientation.y,
                                                p.pose.orientation.z,
                                                p.pose.orientation.w])

            a = np.array([a1, a2, a3])
            sin_avg += 1.0/n*np.sin(a)
            cos_avg += 1.0/n*np.cos(a)

        ax, ay, az = np.arctan2(sin_avg, cos_avg)
        curr1, curr2, curr3 = euler_from_quaternion([avg_pose.pose.orientation.x,
                                                     avg_pose.pose.orientation.y,
                                                     avg_pose.pose.orientation.z,
                                                     avg_pose.pose.orientation.w])

        new1, new2, new3 = curr1+ax, curr2+ay, curr3+az
        qx, qy, qz, qw = quaternion_from_euler(new1, new2, new3)
        avg_pose.pose.orientation.x = qx
        avg_pose.pose.orientation.y = qy
        avg_pose.pose.orientation.z = qz
        avg_pose.pose.orientation.w = qw
        return avg_pose

if __name__ == '__main__':
    rospy.init_node('update_map_to_odom')
    
    init_trans_ls = rospy.get_param("localization/initial_map_to_odom")
    #init_trans_ls = [float(s.strip()) for s in init_trans_str.split()]
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


    # update freq should be high (at least > 20 Hz) so transforms don't get extrapolation errors in other files
    # OBS: high frequency requires outlier detection for the kalman filter to work (high freq detects noisy measurements)
    p = MapOdomUpdate(init_trans=init_t, update_freq=40)
    
    p.spin()
