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

            
            self.broadcaster.sendTransform(self.last_transform)
            self.last_transform.header.stamp = rospy.Time.now()
            
            A = np.diag([1, 1, 1, 1, 1, 1])
            u = [0, 0, 0, 0, 0, 0]
            self.kf.predict(A, u)
            if self.measurement_msg:
                if not self.has_transformed:
                    map_to_odom = self.update(self.msg_to_measurement(self.measurement_msg))
                    if map_to_odom: 
                        self.last_transform = map_to_odom
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
        #time_stamp = marker.header.stamp
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

        # just to get the correct time stamp
        try:
            # This might result in processing the wrong time_stamp???
            time_stamp = self.tf_buf.lookup_transform(frame_marker, frame_detected, rospy.Time(0)).header.stamp
        except:
            # Failes because 
            print("Wait a bit...")
            self.is_measuring = False
            return
        #believed_trans = self.tf_buf.lookup_transform("cf1/odom", "cf1/base_link", time_stamp, rospy.Duration(1.0))
        believed_trans = self.tf_buf.lookup_transform("map", "cf1/base_link", time_stamp, rospy.Duration(1.0))
        believed_pose = self.transform_to_pose_stamped(believed_trans)
        #believed_pose = self.filter_pose(believed_pose) #makes no difference?
        self.believed_pub.publish(believed_pose)
        believed_state = self.pose_stamped_to_state(self.filter_pose(believed_pose))
        
        #measured_pose = self.get_measured_pose(believed_pose, frame_marker, frame_detected)
        measured_pose = self.get_measured_pose_filtered(believed_pose, frame_marker, frame_detected) #DEBUG THIS

        #self.is_measuring = False
        #return
        #TESTING
        if not measured_pose:
            print("Nah")
            self.is_measuring = False
            return
        #measured_pose = self.filter_pose(measured_pose)
        self.measurement_pub.publish(self.filter_pose(measured_pose)) # for vizualisation
        if measured_pose is None: 
            print("Failed1")
            self.is_measuring = False
            return

        measured_state = self.pose_stamped_to_state(measured_pose)
 
        diff = self.kf.inovation(believed_state*self.filter_config, 
                                 measured_state*self.filter_config)
        maha_dist = self.maha_dist(diff, Q)
        print("Mahalanobis dist (kinda): {}".format(maha_dist))
        if maha_dist > 0.7:
            # outlier
            print("Outlier")
            #self.is_measuring = False
            #return

        K = self.kf.kalman_gain(Q)
        #print(K)
        filtered_state = self.filter_state(believed_state, measured_state, K)
        #filtered_state = believed_state + self.kf.inovation(believed_state, measured_state)*0.5

        filtered_pose = self.state_to_pose_stamped(filtered_state, believed_pose.header.frame_id, time_stamp)
        self.filtered_pub.publish(filtered_pose) # for visualization


        self.broadcast_pose_frame(filtered_pose, "cf1/base_link/filtered")
        self.broadcast_pose_frame(self.filter_pose(believed_pose), "cf1/base_link/projection")

        #odom_new_pose = self.get_odom_new_pose(self.filter_pose(believed_pose))
        odom_new_pose = self.get_odom_new_pose(believed_pose)

        if not odom_new_pose: 
            print("Failed3")
            self.is_measuring = False
            return
        self.odom_new_pub.publish(odom_new_pose)

        map_to_odom = self.get_map_to_odom_transform(odom_new_pose)

        if map_to_odom:
            print("Updated")
            self.kf.update_with_gain(K)
            #self.last_transform = map_to_odom # remove if not using this as callback
            self.is_measuring = False
            return map_to_odom
        print("SOMETHING WENT WRONG")
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
        time_stamp = odom_new_pose.header.stamp
        try:
            #odom_new_pose.header.stamp = rospy.Time(0)
            odom_new_in_map = self.tf_buf.transform(odom_new_pose, "map")#, rospy.Duration(1.0))
        except:
            print("Failed2")
            return

        t = TransformStamped()
        t.header.stamp = time_stamp #rospy.Time.now()? doesnt matter prob
        t.header.frame_id = "map"
        t.child_frame_id = "cf1/odom"
        t.transform.translation = Vector3(*[odom_new_in_map.pose.position.x, 
                                            odom_new_in_map.pose.position.y, 
                                            odom_new_in_map.pose.position.z])
        t.transform.rotation = odom_new_in_map.pose.orientation
        return t

    def get_odom_new_pose(self, believed_pose):
        time_stamp = believed_pose.header.stamp
        try:
            base_to_odom = self.tf_buf.lookup_transform("cf1/base_link/projection", "cf1/odom", time_stamp, rospy.Duration(1.0))
        except:
            return
        odom_new_pose = PoseStamped()
        odom_new_pose.header.frame_id = "cf1/base_link/filtered"
        odom_new_pose.header.stamp = time_stamp
        odom_new_pose.pose.position.x = base_to_odom.transform.translation.x
        odom_new_pose.pose.position.y = base_to_odom.transform.translation.y
        odom_new_pose.pose.position.z = base_to_odom.transform.translation.z
        odom_new_pose.pose.orientation = base_to_odom.transform.rotation

        return odom_new_pose

    def get_measured_pose_filtered(self, believed_pose, frame_marker, frame_detected):
        time_stamp = believed_pose.header.stamp
        measured_orientation = self.get_map_to_map_detected_rotation_calc(frame_marker, frame_detected, time_stamp)
        #measured_orientation = self.get_map_to_map_detected_rotation_broadcast(believed_pose, frame_marker, frame_detected)
        
        # SHOULD THIS BE FILTERED?!?!?! - yes, think so
        measured_orientation = self.filter_quat(measured_orientation)


        detected_map_frame = "map_ref_detected"
        map_to_detected = self.tf_buf.lookup_transform("map", frame_detected, time_stamp)
        map_to_detected.child_frame_id = detected_map_frame
        map_to_detected.header.stamp = time_stamp
        map_to_detected.transform.rotation = measured_orientation
        self.broadcaster.sendTransform(map_to_detected)
        

        marker_map_frame = "map_ref_marker"
        map_to_marker = self.tf_buf.lookup_transform("map", frame_marker, time_stamp)
        map_to_marker.child_frame_id = marker_map_frame
        map_to_marker.header.stamp = time_stamp #rospy.Time.now()
        map_to_marker.transform.rotation = Quaternion(*[0,0,0,1])
        self.broadcaster.sendTransform(map_to_marker)
        
        try:
            # the wait here helps alot, or does it?
            pose_in_detected = self.tf_buf.transform(believed_pose, detected_map_frame)
        except:
            print("f1")
            return
        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = marker_map_frame
        try:
            measured_pose = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)
        except:
            print("f2")
            return
        return measured_pose

    def get_map_to_map_detected_rotation_broadcast(self, believed_pose, frame_marker, frame_detected):
        time_stamp = believed_pose.header.stamp
        det_to_map = self.tf_buf.lookup_transform(frame_marker, "map", time_stamp)
        det_to_map.header.frame_id = frame_detected
        det_to_map.child_frame_id = "map_measured"
        self.broadcaster.sendTransform(det_to_map)
        map_map_meas_rot = self.tf_buf.lookup_transform("map", "map_measured", time_stamp, rospy.Duration(1.0)).transform.rotation
        map_map_meas_rot = self.filter_quat(map_map_meas_rot)
        return map_map_meas_rot

    def get_map_to_map_detected_rotation_calc(self, frame_marker, frame_detected, time_stamp):
        #COPIED FROM MARKER UPDATE. REMOVE IF NOT NEEDED
        map_to_marker = self.tf_buf.lookup_transform(frame_marker, "map", time_stamp, rospy.Duration(1.0))
        marker_to_detected = self.tf_buf.lookup_transform(frame_marker, frame_detected, time_stamp, rospy.Duration(1.0))


        rotation = [0, 0, 0, 1]
        map_to_marker_rot = [map_to_marker.transform.rotation.x,
                             map_to_marker.transform.rotation.y,
                             map_to_marker.transform.rotation.z,
                             map_to_marker.transform.rotation.w]
        detected_to_map_detected = map_to_marker_rot[:3] + [-map_to_marker_rot[3]]
        marker_to_detected_rot = [marker_to_detected.transform.rotation.x,
                                  marker_to_detected.transform.rotation.y,
                                  marker_to_detected.transform.rotation.z,
                                  marker_to_detected.transform.rotation.w]

        rotation = quaternion_multiply(map_to_marker_rot, rotation)
        rotation = quaternion_multiply(marker_to_detected_rot, rotation)
        rotation = quaternion_multiply(detected_to_map_detected, rotation)
        
        return Quaternion(*rotation)


    def quat_to_ls(self, q):
        return [q.x, q.y, q.z, q.w]


    def get_measured_pose(self, believed_pose, frame_marker, frame_detected):
        try:
            # the wait here helps alot, or does it?
            #believed_pose.header.stamp = rospy.Time(0)
            pose_in_detected = self.tf_buf.transform(believed_pose, frame_detected, rospy.Duration(1.0))
        except:
            print("f1")
            return
        pose_in_marker = pose_in_detected
        pose_in_marker.header.frame_id = frame_marker
        try:
            measured_pose = self.tf_buf.transform(pose_in_marker, believed_pose.header.frame_id)#, rospy.Duration(1.0))
        except:
            print("f2")
            return
        return measured_pose


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

    def filter_quat(self, q):
        ax, ay, az = euler_from_quaternion(self.quat_to_ls(q))
        ax *= self.filter_config[3]
        ay *= self.filter_config[4]
        az *= self.filter_config[5]
        return Quaternion(*quaternion_from_euler(ax, ay, az))

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
        t.header.stamp = pose.header.stamp
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
    p = MapOdomUpdate(init_trans=init_t, update_freq=200)
    
    p.spin()
