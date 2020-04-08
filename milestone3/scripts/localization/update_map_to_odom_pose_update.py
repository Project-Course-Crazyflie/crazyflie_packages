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

    def inovation(self, believed_state, measured_state):
        trans = (measured_state[:3]-believed_state[:3])
        rot = (measured_state[3:]-believed_state[3:])
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        return np.concatenate([trans, rot])

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
        self.kf = KalmanFilter(initial_cov=np.array([100000000.01, 100000000.01, 100000000.01, 100000000.01, 100000000.01, 100000000.01]), R=np.array([.0001, .0001, .0001, .0001, .0001, .0001]), delta_t=1.0/self.update_freq)

        self.odom_new_pub = rospy.Publisher("cf1/pose/odom_new", PoseStamped, queue_size=1)
        self.believed_pub = rospy.Publisher("cf1/pose/believed", PoseStamped, queue_size=1)
        self.measurement_pub = rospy.Publisher("cf1/pose/measured", PoseStamped, queue_size=1)
        self.filtered_pub = rospy.Publisher("cf1/pose/filtered", PoseStamped, queue_size=1)

    def spin(self):
        rate = rospy.Rate(self.update_freq)
        self.has_transformed = False # to check if initial transform with kalman gain 1 result in good transform
        while not rospy.is_shutdown():

            self.last_transform.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(self.last_transform)

            A = np.diag([1, 1, 1, 1, 1, 1])
            u = [0, 0, 0, 0, 0, 0]
            self.kf.predict(A, u)
            #if self.measurement_msg:
            #    if not self.has_transformed:
            #        map_to_odom = self.update(self.msg_to_measurement(self.measurement_msg))
            #        if map_to_odom: 
            #            self.last_transform = map_to_odom
                        #self.has_transformed = True
                #if valid_marker: self.valid_detection_pub.publish(valid_marker)
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
        
    def _update_callback(self, m_array):
        if self.measurement_msg != m_array:
            self.measurement_msg = m_array

    def update_callback(self, m_array):
        """
        Using poses v1
        """
        if self.old_msg == m_array:
            # Message old 
            return
        if not self.cf1_pose:
            # need pose
            return
        if self.is_measuring:
            return
        self.is_measuring = True
        self.old_msg = m_array
        

        kalman_gains = []
        n_markers = len(m_array.markers)
        #for marker in m_array.markers:
        marker = m_array.markers[0]
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
            Q = np.diag([0.1, 0.1, 1, 1, 1, 0.1])

        
        #believed_pose = self.tf_buf.lookup_transform("cf1/odom", "cf1/base_link", rospy.Time(0), rospy.Duration(1.0))
        #believed_pose = self.transform_to_pose_stamped(believed_pose)
        print("Dansa")
        try:
            believed_pose = self.tf_buf.transform(self.cf1_pose, "map", rospy.Duration(1.0))
            believed_pose = self.filter_pose(believed_pose)
            #believed_pose.header.stamp = rospy.Time(0)
            self.believed_pub.publish(believed_pose)
        except:
            print("FailedX")
            self.is_measuring = False
            return
        believed_state = self.pose_stamped_to_state(believed_pose)
        
        #measured_pose = self.get_measured_pose(frame_marker, frame_detected, time_stamp)
        #measured_pose = self.get_measured_pose_no_broadcast(believed_pose, frame_marker, frame_detected)
        measured_pose = self.get_measured_pose_no_broadcast_new(believed_pose, frame_marker, frame_detected)
        print("Pausa")
        if not measured_pose: 
            print("Failed to get measured pose")
            self.is_measuring = False
            return
        self.measurement_pub.publish(measured_pose) # for vizualisation
        
        if measured_pose is None: 
            print("Failed1")
            self.is_measuring = False
            return
        
        time_stamp = measured_pose.header.stamp
        measured_state = self.pose_stamped_to_state(measured_pose)
 
        diff = self.kf.inovation(believed_state, measured_state)
        maha_dist = self.maha_dist(diff, Q)
        print("Mahalanobis dist (kinda): {}".format(maha_dist))
        if maha_dist > 0.7:
            # outlier
            print("Outlier")
            self.is_measuring = False
            return

        K = self.kf.kalman_gain(Q)
        filtered_state = self.filter_state(believed_state, measured_state, K)
        #filtered_state = believed_state + self.residual(believed_state, measured_state)*1

        filtered_pose = self.state_to_pose_stamped(filtered_state, believed_pose.header.frame_id, time_stamp)
        self.filtered_pub.publish(filtered_pose) # for visualization
        self.broadcast_pose_frame(filtered_pose, "cf1/base_link/filtered")
        self.broadcast_pose_frame(self.filter_pose(believed_pose), "cf1/base_link/projection")

        #odom_new_pose = self.get_odom_new_pose_no_broadcast(self.filter_pose(believed_pose), filtered_pose)
        odom_new_pose = self.get_odom_new_pose(self.filter_pose(believed_pose))
        if not odom_new_pose: 
            print("Failed3")
            self.is_measuring = False
            return
        self.odom_new_pub.publish(odom_new_pose)

        map_to_odom = self.get_map_to_odom_transform(odom_new_pose)
        if map_to_odom:
            print("Updated")
            self.kf.update_with_gain(K)
            self.last_transform = map_to_odom # remove if not using this as callback
            self.is_measuring = False
            return map_to_odom
        self.is_measuring = False

    def transform_to_pose_stamped(self, t):
        p = PoseStamped()
        p.header = t.header
        p.pose.position = Point(*[t.transform.translation.x,
                                     t.transform.translation.y,
                                     t.transform.translation.z])
        p.pose.orientation = t.transform.rotation
        return p


    def get_map_to_odom_transform(self, odom_new_pose):
        try:
            #odom_new_pose.header.stamp = rospy.Time(0)
            odom_new_in_map = self.tf_buf.transform(odom_new_pose, "map")#, rospy.Duration(1.0))
        except:
            print("Failed2")
            return

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "cf1/odom"
        t.transform.translation = Vector3(*[odom_new_in_map.pose.position.x, 
                                            odom_new_in_map.pose.position.y, 
                                            odom_new_in_map.pose.position.z])
        t.transform.rotation = odom_new_in_map.pose.orientation
        return t
        

    def get_odom_new_pose_no_broadcast(self, believed_pose, filtered_pose):
        odom_new_pose = PoseStamped()
        odom_new_pose.header.frame_id = believed_pose.header.frame_id
        odom_new_pose.header.stamp = believed_pose.header.stamp
        odom_new_pose.pose.position.x = filtered_pose.pose.position.x - believed_pose.pose.position.x
        odom_new_pose.pose.position.y = filtered_pose.pose.position.y - believed_pose.pose.position.y
        odom_new_pose.pose.position.z = filtered_pose.pose.position.z - believed_pose.pose.position.z
        q_bel = [believed_pose.pose.orientation.x,
                 believed_pose.pose.orientation.y,
                 believed_pose.pose.orientation.z,
                 -believed_pose.pose.orientation.w]
        q_filt = [filtered_pose.pose.orientation.x,
                  filtered_pose.pose.orientation.y,
                  filtered_pose.pose.orientation.z,
                  filtered_pose.pose.orientation.w]
        
        odom_new_pose.pose.orientation = Quaternion(*quaternion_multiply(q_bel, q_filt))
        
        return odom_new_pose

    def get_odom_new_pose(self, believed_pose):
        try:
            base_to_odom = self.tf_buf.lookup_transform("cf1/base_link/projection", "cf1/odom", rospy.Time(0), rospy.Duration(1.0))
            #believed_pose
        except:
            return
        odom_new_pose = PoseStamped()
        odom_new_pose.header.frame_id = "cf1/base_link/filtered"
        odom_new_pose.header.stamp = believed_pose.header.stamp
        odom_new_pose.pose.position.x = base_to_odom.transform.translation.x
        odom_new_pose.pose.position.y = base_to_odom.transform.translation.y
        odom_new_pose.pose.position.z = base_to_odom.transform.translation.z
        odom_new_pose.pose.orientation = base_to_odom.transform.rotation
        #q = [believed_pose.pose.orientation.x,
        #     believed_pose.pose.orientation.y,
        #     believed_pose.pose.orientation.z,
        #     -believed_pose.pose.orientation.w]
        #odom_new_pose.pose.orientation = Quaternion(*q)

        return odom_new_pose


    def get_measured_pose_no_broadcast_new(self, believed_pose, frame_marker, frame_detected):
        measurement = self.get_measured_pose_orientation(believed_pose, frame_marker, frame_detected)
        if not measurement: 
            print("failmeas1")
            return

        measurement = self.filter_pose(measurement) # OBS only filters orientation, dont care about this position
        believed_pose = self.filter_pose(believed_pose)
        
        ## for visualization
        marker_map_frame = frame_marker + "/map_rot"
        map_to_marker = self.tf_buf.lookup_transform("map", frame_marker, measurement.header.stamp)
        map_to_marker.child_frame_id = marker_map_frame
        map_to_marker.header.stamp = measurement.header.stamp #rospy.Time.now()
        map_to_marker.transform.rotation = measurement.pose.orientation
        self.broadcaster.sendTransform(map_to_marker)

        detected_map_frame = frame_detected + "/map_rot"
        map_to_detected = self.tf_buf.lookup_transform("map", frame_detected, measurement.header.stamp)
        map_to_detected.child_frame_id = detected_map_frame
        map_to_detected.header.stamp = measurement.header.stamp
        map_to_detected.transform.rotation = believed_pose.pose.orientation
        self.broadcaster.sendTransform(map_to_detected)

        ## /for visualization
        
        #believed_pose.header.stamp = rospy.Time(0)
        measured_pose = self.tf_buf.transform(believed_pose, detected_map_frame, rospy.Duration(1.0))
        measured_pose.header.frame_id = marker_map_frame
        measured_pose.header.stamp = measurement.header.stamp #rospy.Time(0)
        try:
            measured_pose = self.tf_buf.transform(measured_pose, "map", rospy.Duration(1.0))
            measurement = measured_pose
            measurement = self.filter_pose(measurement) # Filter translation aswell (orientation was already filtered)
            measurement.header.stamp = rospy.Time.now()
        except:
            print("Nehe")
            return

        return measurement

    def get_measurement(self, frame_detected):
        self.tf_buf.lookup_transform(frame_detected, )

    def get_measured_pose_orientation(self, believed_pose, frame_marker, frame_detected):
        # This is wrong
        try:
            # the wait here helps alot
            #believed_pose.header.stamp = rospy.Time(0)
            pose_in_detected = self.tf_buf.transform(believed_pose, frame_detected, rospy.Duration(1.0))
        except:
            return
        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = frame_marker
        try:
            measurement = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)#, rospy.Duration(1.0))
        except:
            return
        return measurement

    def get_measured_pose_no_broadcast(self, believed_pose, frame_marker, frame_detected):
        # This is wrong
        try:
            # the wait here helps alot
            pose_in_detected = self.tf_buf.transform(believed_pose, frame_detected)#, rospy.Duration(1.0))
        except:
            return
        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = frame_marker
        try:
            measurement = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)#, rospy.Duration(1.0))
        except:
            return
        return measurement

    def get_measured_pose(self, frame_marker, frame_detected, time_stamp):
        # This is wrong
        try:
            detected_to_belief = self.tf_buf.lookup_transform(frame_detected, "cf1/base_link", rospy.Time(0), rospy.Duration(1.0))
        except:
            return
        time_stamp = detected_to_belief.header.stamp

        measured_frame = frame_marker + "_cf1/base_link"
        marker_to_measurement = detected_to_belief
        marker_to_measurement.header.frame_id = frame_marker
        marker_to_measurement.header.stamp = rospy.Time.now() ### changed to.now()
        marker_to_measurement.child_frame_id = measured_frame
        self.broadcaster.sendTransform(marker_to_measurement)
        measurement = PoseStamped()
        measurement.header.stamp = rospy.Time(0) # changed to .Tim(0) from time_stamp
        measurement.header.frame_id = measured_frame
        measurement.pose.orientation.w = 1
        
        try:
            measurement = self.tf_buf.transform(measurement, "map")#, rospy.Duration(1.0))
        except:
            return None
        return measurement

    def modify_pose(self, pose, x=None, y=None, z=None, rx=None, ry=None, rz=None):

        new = PoseStamped()
        new.header = pose.header
        new.pose.position.x = pose.pose.position.x if x is None else x
        new.pose.position.y = pose.pose.position.y if y is None else y
        new.pose.position.z = pose.pose.position.z if z is None else z
        ax, ay, az = euler_from_quaternion([pose.pose.orientation.x, 
                                            pose.pose.orientation.y, 
                                            pose.pose.orientation.z,
                                            pose.pose.orientation.w])

        ax = ax if rx is None else rx
        ay = ay if ry is None else ry
        az = az if rz is None else rz
        new.pose.orientation = Quaternion(*quaternion_from_euler(ax, ay, az))
        return new

    def filter_pose(self, odom_pose):
        # change parameter name, its not odom_pose
        proj = PoseStamped()
        proj.header = odom_pose.header
        proj.pose.position.x = odom_pose.pose.position.x*self.filter_config[0]
        proj.pose.position.y = odom_pose.pose.position.y*self.filter_config[1]
        proj.pose.position.z = odom_pose.pose.position.z*self.filter_config[2]
        rx, ry, rz = euler_from_quaternion([odom_pose.pose.orientation.x, 
                                            odom_pose.pose.orientation.y, 
                                            odom_pose.pose.orientation.z,
                                            odom_pose.pose.orientation.w])

        rx *= self.filter_config[3]
        ry *= self.filter_config[4]
        rz *= self.filter_config[5]
        proj.pose.orientation = Quaternion(*quaternion_from_euler(rx, ry, rz))
        return proj

    def filter_state(self, believed_state, measured_state, K):
        K = K.diagonal()
        believed_state *= self.filter_config
        measured_state *= self.filter_config
        trans = believed_state[:3] + (measured_state[:3]-believed_state[:3])*K[:3]

        rot = (measured_state[3:]-believed_state[3:])
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        rot = believed_state[3:] + rot*K[3:]
        rot = (rot + np.pi) % (2*np.pi) - np.pi
        
        return np.concatenate([trans, rot])



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

    def broadcast_pose_frame(self, pose, child_frame_id):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = pose.header.frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation = pose.pose.position
        t.transform.rotation = pose.pose.orientation
        self.broadcaster.sendTransform(t)

    def get_map_to_odom(self, time_stamp):
        try: 
            map_filtered_to_odom = self.tf_buf.lookup_transform("map", "cf1/odom_new", time_stamp)#, rospy.Duration(1.0)) # rospy.Time(0)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
            print(e)
            print("Transform lookup between map and cf1/odom_new failed")
            return
        else:
            map_filtered_to_odom.header.frame_id = "map"
            map_filtered_to_odom.child_frame_id = "cf1/odom"
            map_to_odom = map_filtered_to_odom
            map_to_odom.header.stamp = time_stamp
        return map_to_odom

    def maha_dist(self, diff, Q):
        #cov = self.kf.cov*self.filter_config
        cov = self.kf.cov.copy()
        s = np.matmul(np.sqrt(cov), np.sqrt(Q)) # test this to see if it works
        #s = Q
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
