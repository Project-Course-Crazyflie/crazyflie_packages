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

from milestone3.srv import ResetKalmanFilter, ResetKalmanFilterResponse

import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, initial_cov, R, maha_dist_thres, cov_norm_thres, delta_t):
        # position/mean not neaded since it will provided by the crazyflie
        self.initial_cov = initial_cov
        self.cov = initial_cov
        self.R = R
        self.maha_dist_thres = maha_dist_thres
        self.cov_norm_thres = cov_norm_thres
        self.delta_t = delta_t

        self.n_updates = 0
        self.prev_cov = self.cov.copy()

    def reset(self):
        self.cov = self.initial_cov.copy()

    def has_converged(self):
        return np.linalg.norm(self.cov) < self.cov_norm_thres

    def handle_measurements(self, belief, measurements, Qs):
        assert len(measurements) == len(Qs), "measurements and !s need to have the same length"
        valids = []
        kalman_gains = []
        new_beliefs = []
        for m, Q in zip(measurements, Qs):
            valid, K, new_belief = self.handle_measurement(belief, m, Q)
            valids.append(valid)
            if valid:
                kalman_gains.append(K)
                new_beliefs.append(new_belief)
        if len(new_beliefs) == 0:
            new_belief = None
            K = None
        else:
            new_belief = self.average_beliefs(new_beliefs)
            K = sum(kalman_gains)/len(kalman_gains)
        return valids, K, new_belief

    def handle_measurement(self, belief, measurement, Q):
        """returns Kalman
        """
        innovation = self.innovation(belief, measurement)
        valid = self.maha_dist(innovation, Q) < self.maha_dist_thres
        #print("MAHA: " + str(self.maha_dist(innovation, Q)))
        K = self.kalman_gain(Q)
        new_belief = self.new_belief(belief, innovation, K)
        return valid, K, new_belief

    def average_beliefs(self, beliefs):
        avg_belief = [0]*6
        n = len(beliefs)
        sin_avg = 0
        cos_avg = 0
        for b in beliefs:
            avg_belief[0] += b[0]/n
            avg_belief[1] += b[1]/n
            avg_belief[3] += b[2]/n

            a = np.array(b[3:])
            sin_avg += 1.0/n*np.sin(a)
            cos_avg += 1.0/n*np.cos(a)

        avg_belief[3:] = np.arctan2(sin_avg, cos_avg)
        return avg_belief

    def maha_dist(self, diff, Q):
        return np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(self.cov)), diff))

    def innovation(self, believed_state, measured_state):
        trans = (measured_state[:3]-believed_state[:3])
        rot = (measured_state[3:]-believed_state[3:])
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        return np.concatenate([trans, rot])

    def kalman_gain(self, Q):
        K = np.matmul(self.cov, np.linalg.inv(self.cov + Q))
        return K

    def new_belief(self, belief, innovation, K):
        K = K.diagonal()
        pos = belief[:3] + innovation[:3]*K[:3]

        rot = belief[3:] + innovation[3:]*K[3:]
        rot = (rot + np.pi) % (2*np.pi) - np.pi

        return np.concatenate([pos, rot])

    def predict(self, A, v=None):
        # TODO: use dynamic process noise
        R = self.R
        if v is not None:
            v_mat = np.diag(abs(v/np.linalg.norm(v)))
            R = np.matmul(np.matmul(v_mat, self.R), v_mat.transpose())
        self.cov = np.matmul(np.matmul(A, self.cov), A.transpose()) + R*self.delta_t

    def update_with_gain(self, K):
        self.cov = np.matmul((np.eye(K.shape[0])-K), self.cov)
        self.prev_cov = self.cov.copy()

class MapOdomUpdate:
    def __init__(self, init_trans, update_freq):
        rospy.Subscriber('aruco/markers', MarkerArray, self.update_callback)
        #rospy.Subscriber('sign_pose', MarkerArray, self.update_callback)
        #rospy.Subscriber('marker_measurements', MarkerArray, self.update_callback)

        rospy.Subscriber("cf1/pose", PoseStamped, self.cf1_pose_callback)
        rospy.Subscriber("cf1/velocity", TwistStamped, self.cf1_vel_callback)
        self.cf1_pose_cov_pub = rospy.Publisher("cf1/localizatiton/pose_cov", PoseWithCovarianceStamped, queue_size=1)

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
                               maha_dist_thres= rospy.get_param("localization/maha_dist_thres"),
                               cov_norm_thres=rospy.get_param("localization/cov_norm_thres"),
                               delta_t=1.0/self.update_freq)

        self.maha_dist_thres = rospy.get_param("localization/maha_dist_thres")
        self.cov_norm_thres =  rospy.get_param("localization/cov_norm_thres")

        self.converged_pub = rospy.Publisher("cf1/localization/converged", Bool, queue_size=1)
        self.measurement_fb_pub = rospy.Publisher("cf1/localization/measurement_feedback", Int32MultiArray, queue_size=1)

        self.odom_new_pub = rospy.Publisher("cf1/pose/odom_new", PoseStamped, queue_size=1)
        self.believed_pub = rospy.Publisher("cf1/pose/believed", PoseStamped, queue_size=1)
        self.measured_valid_pub = rospy.Publisher("cf1/pose/measured_valid", PoseArray, queue_size=1)
        self.measured_invalid_pub = rospy.Publisher("cf1/pose/measured_invalid", PoseArray, queue_size=1)
        self.filtered_pub = rospy.Publisher("cf1/pose/filtered", PoseStamped, queue_size=1)

        rospy.Service("cf1/localization/reset_kalman_filter", ResetKalmanFilter, self.reset_kalman_filter)

    def spin(self):

        rate = rospy.Rate(self.update_freq)
        self.has_transformed = False # to check if initial transform with kalman gain 1 result in good transform
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(self.last_transform)
            self.last_transform.header.stamp = rospy.Time.now()

            A = np.diag([1, 1, 1, 1, 1, 1])
            if False:#self.cf1_vel:
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
                map_to_odom = self.update(self.measurement_msg)
                if map_to_odom:
                    self.last_transform = map_to_odom

            self.converged_pub.publish(Bool(self.kf.has_converged()))

            if self.cf1_pose:
                p = PoseWithCovarianceStamped()
                p.header = self.cf1_pose.header
                p.pose.pose = self.cf1_pose.pose
                #self.tf_buf.transform(p, "map")
                p.pose.covariance[0] = self.kf.cov[0][0] # x
                p.pose.covariance[1] = self.kf.cov[0][1] # xy
                p.pose.covariance[6] = self.kf.cov[1][0] # yx
                p.pose.covariance[7] = self.kf.cov[1][1] # y
                p.pose.covariance[-1] = self.kf.cov[5][5] # z angle
                self.cf1_pose_cov_pub.publish(p)

            rate.sleep()

    def reset_kalman_filter(self, _):
        self.kf.reset()
        return ResetKalmanFilterResponse()

    def cf1_pose_callback(self, msg):
        self.cf1_pose = msg

    def cf1_vel_callback(self, msg):
        self.cf1_vel = msg

    def update_callback(self, m_array):
        if self.measurement_msg != m_array:
            self.measurement_msg = m_array

    def _update(self, m_array):
        """
        Giving kalman filter the responsibility to handle measurements
        """
        if self.old_msg == m_array:
            # Message old
            return
        self.old_msg = m_array


        try:
            believed_trans = self.tf_buf.lookup_transform("map", "cf1/base_link", rospy.Time(0))
        except:
            print("Is map to cf1/odom being published?")
            return
        believed_pose = PoseStamped()
        believed_pose.header = believed_trans.header
        believed_pose.pose.position = Point(*[believed_trans.transform.translation.x,
                                               believed_trans.transform.translation.y,
                                               believed_trans.transform.translation.z])
        believed_pose.pose.orientation = believed_trans.transform.rotation
        believed_state = self.pose_stamped_to_state(believed_pose)

        measurements = []
        measured_poses = []
        Qs = []
        for marker in m_array.markers:
            # TODO: make this general (no hardcoded Qs)
            Q = np.diag(rospy.get_param("perception/aruco_cov"))

            measured_pose = self.get_measured_pose_filtered(believed_pose, marker)
            measured_state = self.pose_stamped_to_state(measured_pose)
            measured_poses.append(measured_pose.pose)
            measurements.append(measured_state)
            Qs.append(Q)

        valids, K, filtered_state = self.kf.handle_measurements(believed_state, measurements, Qs)

        measured_valid_poses = PoseArray()
        measured_invalid_poses = PoseArray()
        measured_valid_poses.header.frame_id = "map"
        measured_invalid_poses.header.frame_id = "map"

        for v, meas_pose, mark in zip(valids, measured_poses, m_array.markers):
            feedback = Int32MultiArray()
            feedback.data = [int(v), mark.id]
            self.measurement_fb_pub.publish(feedback)
            if v:
                measured_valid_poses.poses.append(meas_pose)
            else:
                measured_invalid_poses.poses.append(meas_pose)

        self.measured_valid_pub.publish(measured_valid_poses) # visualization
        self.measured_invalid_pub.publish(measured_invalid_poses) # visualization

        #print("Using {}/{} markers measurements".format(len(filtered_poses), n_markers))
        if filtered_state:
            #self.kf.update_with_gain(K)
            filtered_pose = self.state_to_pose_stamped(filtered_state, "map", believed_pose.header.stamp)
            self.filtered_pub.publish(filtered_pose) # for visualization
            map_to_odom = self.get_map_to_odom(filtered_pose)
            #return map_to_odom

    def update(self, m_array):
        """
        Old working version
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
        try:
            believed_trans = self.tf_buf.lookup_transform("map", "cf1/base_link", rospy.Time(0))
        except:
            print("Is map to cf1/odom being published?")
            return
        believed_pose = PoseStamped()
        believed_pose.header = believed_trans.header
        believed_pose.pose.position = Point(*[believed_trans.transform.translation.x,
                                               believed_trans.transform.translation.y,
                                               believed_trans.transform.translation.z])
        believed_pose.pose.orientation = believed_trans.transform.rotation
        believed_state = self.pose_stamped_to_state(believed_pose)
        time_stamp = believed_pose.header.stamp

        for marker in m_array.markers:
            # TODO: make this general (no hardcoded Qs)
            Q = np.diag(rospy.get_param("perception/aruco_cov"))

            measurement_fb = Int32MultiArray()
            measured_pose = self.get_measured_pose_filtered(believed_pose, marker)
            if not measured_pose:
                continue
            measured_state = self.pose_stamped_to_state(measured_pose)

            valid, K, filtered_state = self.kf.handle_measurement(believed_state, measured_state, Q)
            # for testing with K=1

            #valid = True
            #filtered_state = believed_state + self.kf.innovation(believed_state, measured_state)*1

            measurement_fb.data = [marker.id, int(valid)]
            self.measurement_fb_pub.publish(measurement_fb)
            if valid:
                kalman_gains.append(K)
                measured_valid_poses.poses.append(measured_pose.pose)
                filtered_pose = self.state_to_pose_stamped(filtered_state, believed_pose.header.frame_id, time_stamp)
                filtered_poses.append(filtered_pose)
            else:
                measured_invalid_poses.poses.append(measured_pose.pose)
                continue

        self.measured_valid_pub.publish(measured_valid_poses) # visualization
        self.measured_invalid_pub.publish(measured_invalid_poses) # visualization

        #print("Using {}/{} markers measurements".format(len(filtered_poses), n_markers))
        if len(filtered_poses) > 0:
            K = sum(kalman_gains)/len(filtered_poses)
            self.kf.update_with_gain(K)
            filtered_pose = self.average_poses(filtered_poses)
            self.filtered_pub.publish(filtered_pose) # for visualization
            map_to_odom = self.get_map_to_odom(filtered_pose)

            #print("Updated")
            return map_to_odom

    def get_measured_pose_filtered(self, believed_pose, marker):
        time_stamp = believed_pose.header.stamp
        frame_detected = "tjululu/detected" + str(marker.id)
        frame_marker = "aruco/marker" + str(marker.id)
        t = self.transform_from_marker(marker, frame_detected, time_stamp)
        self.tf_buf.set_transform(t, "gandalfs_authority")
        #self.broadcaster.sendTransform(t) # for vizualization
        measured_orientation = self.get_map_to_map_detected_rotation(frame_marker, frame_detected, time_stamp)
        if not measured_orientation:
            return

        # Disregard rotational diplacement of marker detection
        ax, ay, az = euler_from_quaternion([measured_orientation.x,
                                            measured_orientation.y,
                                            measured_orientation.z,
                                            measured_orientation.w])
        ax *= self.filter_config[3]
        ay *= self.filter_config[4]
        az *= self.filter_config[5]
        measured_orientation = Quaternion(*quaternion_from_euler(ax, ay, az))

        marker_map_frame = frame_marker + "_map_reference"
        map_to_marker = self.tf_buf.lookup_transform_core("map", frame_marker, time_stamp)
        map_to_marker.child_frame_id = marker_map_frame
        map_to_marker.header.stamp = time_stamp
        map_to_marker.transform.rotation = Quaternion(*[0,0,0,1])
        self.tf_buf.set_transform(map_to_marker, "gandalfs_authority")
        #self.broadcaster.sendTransform(map_to_marker) # for visualization

        detected_map_frame = frame_detected + "_map_reference"
        # not use _core here?
        map_to_detected = self.tf_buf.lookup_transform_core("map", frame_detected, time_stamp)
        map_to_detected.child_frame_id = detected_map_frame
        map_to_detected.header.stamp = time_stamp
        # Disregard translational diplacement of marker detection
        if not self.filter_config[0]: map_to_detected.transform.translation.x = map_to_marker.transform.translation.x
        if not self.filter_config[1]: map_to_detected.transform.translation.y = map_to_marker.transform.translation.y
        if not self.filter_config[2]: map_to_detected.transform.translation.z = map_to_marker.transform.translation.z
        map_to_detected.transform.rotation = measured_orientation
        self.tf_buf.set_transform(map_to_detected, "gandalfs_authority")
        #self.broadcaster.sendTransform(map_to_detected) # for visualization

        pose_in_detected = self.tf_buf.transform(believed_pose, detected_map_frame)

        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = marker_map_frame
        measured_pose = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)

        # visualization

        #map_to_marker = self.tf_buf.lookup_transform_core("map", frame_marker, time_stamp)
        #marker_to_detected = self.tf_buf.lookup_transform_core(marker_map_frame, detected_map_frame, time_stamp)
        #map_to_detected = self.tf_buf.lookup_transform_core("map", frame_detected, time_stamp)
        ref_to_ref_det = self.tf_buf.lookup_transform_core(marker_map_frame, detected_map_frame, time_stamp)
        map_to_detected_ref = self.tf_buf.lookup_transform_core("map", detected_map_frame, time_stamp)
        filt_det = TransformStamped()
        filt_det.header.stamp = time_stamp
        filt_det.header.frame_id = "map"
        filt_det.child_frame_id = frame_detected + "_filtered"
        filt_det.transform.translation = map_to_detected_ref.transform.translation
        filt_det.transform.rotation = ref_to_ref_det.transform.rotation
        #q1 = [map_to_marker.transform.rotation.x, map_to_marker.transform.rotation.y,
        #      map_to_marker.transform.rotation.z, map_to_marker.transform.rotation.w]
        #q2 = [measured_orientation.x, measured_orientation.y, measured_orientation.z, measured_orientation.w]
        #q = quaternion_multiply(q1, q2)
        #filt_det.transform.rotation = Quaternion(*q)
        #self.broadcaster.sendTransform(filt_det)

        # end visualization

        return self.tf_buf.transform(measured_pose, "map")

    def transform_from_marker(self, m, frame, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'cf1/camera_link'
        t.child_frame_id = frame
        t.transform.translation = Vector3(*[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        t.transform.rotation = m.pose.pose.orientation
        return t

    def get_map_to_map_detected_rotation(self, frame_marker, frame_detected, time_stamp):
        # This fails when an aruco marker that doesn't exist is detected
        try:
            map_to_marker = self.tf_buf.lookup_transform_core(frame_marker, "map", time_stamp)
        except:
            return None
        marker_to_detected = self.tf_buf.lookup_transform_core(frame_marker, frame_detected, time_stamp)

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
