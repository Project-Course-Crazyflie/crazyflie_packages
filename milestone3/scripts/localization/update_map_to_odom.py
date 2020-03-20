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
from aruco_msgs.msg import Marker, MarkerArray


class KalmanFilter:
    def __init__(self, initial_cov, R, delta_t):
        # position/mean not neaded since it will provided by the crazyflie
        self.cov = np.diag(initial_cov)
        self.R = np.diag(R)
        self.delta_t = delta_t

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

    def predict(self, A, u):
        self.cov = np.matmul(np.matmul(A, self.cov), A) + self.R*self.delta_t

    def update(self, Q):
        K = np.matmul(self.cov, np.linalg.inv(self.cov + Q))
        self.cov = np.matmul((np.eye(K.shape[0])-K), self.cov)
        return K.diagonal()

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

        self.measurement_msg = None
        self.old_msg = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, queue_size=1) # should there be a queue_size?
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
       
        self.update_freq = update_freq
        self.kf = KalmanFilter(initial_cov=np.array([500.01, 500.01, 100.01]), R=np.array([.0005, .0005, .001]), delta_t=1.0/self.update_freq)
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

    def _update_callback(self, msg):
        self.measurement_msg = msg

    def msg_to_measurement(self, msg):
        # TODO
        return msg

    def update_callback(self, m_array):
        if self.old_msg == m_array:
            # Message old 
            return
        self.old_msg = m_array
        
        # TODO: Make possible to update multiple markers?
        #marker = m_array.markers[0]
        map_to_map_new_transforms = []
        for marker in m_array.markers:
            map_to_map_measured = self.get_map_to_map_measured_trans(marker)
            
            # TODO: make this general (no hardcoded Qs)
            if marker.id > 9: 
                Q = np.diag([0.5, 0.5, 0.5])
            else: 
                Q = np.diag([0.1, 0.1, 0.1])
            if self.outlier(map_to_map_measured, Q): 
                continue
            
            map_filtered = self.kalman_filter(map_to_map_measured, Q)
            map_to_map_new_transforms.append(map_filtered)

        if not map_to_map_new_transforms:
            print("Measurements not valid!")
            return
        map_filtered = self.average_map_to_map_new(map_to_map_new_transforms)

        """
        if not self.tf_buf.can_transform(frame_map, frame_detected, rospy.Time(0), rospy.Duration(1.0)):
            rospy.logwarn_throttle(5.0, 'No transform from {} to {}'.format(frame_map, frame_detected))
            return

        # OBS!: Transform between aruco/markedX and aruco/detectedX is not the same as transform between map and map_measured!!! (aruco markers can have different orientation than map)
        # Transform between marker and map is the same as detected to measured map
        while not rospy.is_shutdown():
            try: detected_to_map_new = self.tf_buf.lookup_transform(frame_map, "map", rospy.Time(0), rospy.Duration(1.0))
            except: print("walla0")
            else: break
        #detected_to_map_new.header.stamp = rospy.Time.now()
        detected_to_map_new.header.frame_id = frame_detected
        detected_to_map_new.child_frame_id = "map_measured"
        self.static_broadcaster.sendTransform(detected_to_map_new) # Don't work with dynamic broadcaster, something with stamps?
        

        # Transform between map and measured map filtered using kalman filter
        while not rospy.is_shutdown():
            try: map_to_map_new = self.tf_buf.lookup_transform("map", "map_measured", rospy.Time(0), rospy.Duration(1.0)) # rospy.Time(0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                print(e)
                print("walla1")
            else: break
        

        if self.outlier(map_to_map_new, Q): 
            print("Outlier detected!")
            return None
        map_filtered = self.kalman_filter(map_to_map_new, Q)
        """
        
        self.broadcaster.sendTransform(map_filtered)

        # Filtered map to cf1/odom redefines new map to cf1/odom
        while not rospy.is_shutdown():
            try: 
                map_filtered_to_odom = self.tf_buf.lookup_transform("map_filtered", "cf1/odom", rospy.Time(0), rospy.Duration(1.0)) # rospy.Time(0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                print(e)
                print("walla2")
            else: break
        map_filtered_to_odom.header.frame_id = "map"
        map_filtered_to_odom.child_frame_id = "cf1/odom"
        map_to_odom = map_filtered_to_odom
        map_to_odom.header.stamp = rospy.Time.now()

        #rospy.loginfo("Created new transform between map and cf1/odom")
        self.last_transform = map_to_odom

    def get_map_to_map_measured_trans(self, marker):
        # TODO: make this general (no hardcoded signs)
        if marker.id > 9:
            frame_detected = "sign_detected/stop"
            frame_map = "sign/stop"
            print("Using stop sign!!!")
        else:
            frame_detected = "aruco/detected" + str(marker.id)
            frame_map = "aruco/marker" + str(marker.id)

        if not self.tf_buf.can_transform(frame_map, frame_detected, rospy.Time(0), rospy.Duration(1.0)):
            rospy.logwarn_throttle(5.0, 'No transform from {} to {}'.format(frame_map, frame_detected))
            return

        # OBS!: Transform between aruco/markedX and aruco/detectedX is not the same as transform between map and map_measured!!! (aruco markers can have different orientation than map)
        # Transform between marker and map is the same as detected to measured map
        while not rospy.is_shutdown():
            try: detected_to_map_measured = self.tf_buf.lookup_transform(frame_map, "map", rospy.Time(0), rospy.Duration(1.0))
            except: print("walla0")
            else: break
        #detected_to_map_new.header.stamp = rospy.Time.now()
        detected_to_map_measured.header.frame_id = frame_detected
        detected_to_map_measured.child_frame_id = "map_measured"
        self.static_broadcaster.sendTransform(detected_to_map_measured) # Don't work with dynamic broadcaster, something with stamps?
        

        # Transform between map and measured map filtered using kalman filter
        while not rospy.is_shutdown():
            try: map_to_map_new = self.tf_buf.lookup_transform("map", "map_measured", rospy.Time(0), rospy.Duration(1.0)) # rospy.Time(0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
                print(e)
                print("walla1")
            else: break
        
        return map_to_map_new

    def average_map_to_map_new(self, map_to_map_new_transforms):
        n = len(map_to_map_new_transforms)
        final_map_to_map_new = TransformStamped()
        final_map_to_map_new.header = map_to_map_new_transforms[-1].header
        final_map_to_map_new.header.frame_id = "map"
        final_map_to_map_new.child_frame_id = "map_filtered"
        
        for t in map_to_map_new_transforms:
            final_map_to_map_new.transform.translation.x += t.transform.translation.x/n
            final_map_to_map_new.transform.translation.y += t.transform.translation.y/n
            final_map_to_map_new.transform.translation.z += t.transform.translation.z/n
            a1, a2, a3 = euler_from_quaternion([t.transform.rotation.x,
                                               t.transform.rotation.y,
                                               t.transform.rotation.z,
                                               t.transform.rotation.w])

            curr1, curr2, curr3 = euler_from_quaternion([final_map_to_map_new.transform.rotation.x,
                                                         final_map_to_map_new.transform.rotation.y,
                                                         final_map_to_map_new.transform.rotation.z,
                                                         final_map_to_map_new.transform.rotation.w])

            new1, new2, new3 = curr1+a1/n, curr2+a2/n, curr3+a3/n
            qx, qy, qz, qw = quaternion_from_euler(new1, new2, new3)
            final_map_to_map_new.transform.rotation.x = qx
            final_map_to_map_new.transform.rotation.y = qy
            final_map_to_map_new.transform.rotation.z = qz
            final_map_to_map_new.transform.rotation.w = qw
        return final_map_to_map_new

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
        t.transform.translation.z = 0.5*t.transform.translation.z # Keep z in case aruco markers are ill placed irl

        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        
        a1, a2, a3 = euler_from_quaternion(q)
        a1 = K_rot*a1 *0 # Multiply by zero, no risk of drift
        a2 = K_rot*a2 *0 # Multiply by zero, no risk of drift
        a3 = K_rot*a3
        q_new = quaternion_from_euler(a1, a2, a3)
        new_t.transform.rotation.x = q_new[0]
        new_t.transform.rotation.y = q_new[1]
        new_t.transform.rotation.z = q_new[2]
        new_t.transform.rotation.w = q_new[3]
        
        return new_t
        
    def outlier(self, t, Q):
        # trying stop sign
        #s = self.kf.cov[:2, :2]
        #diff = np.array([t.transform.translation.x, t.transform.translation.y])
        #mahalanobis_dist = np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(s)), diff))
        #print("Mahalanobis dist: {}".format(mahalanobis_dist))
        #return mahalanobis_dist > 60

        # mahalanobis dist
        s = np.matmul(np.sqrt(self.kf.cov), np.sqrt(Q)) # test this to see if it works
        diff = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.rotation.z])
        mahalanobis_dist = np.sqrt(np.matmul(np.matmul(diff.transpose(), np.linalg.inv(s)), diff))
        print("Mahalanobis dist (kinda): {}".format(mahalanobis_dist))
        return mahalanobis_dist > 2.5

        # kullback-leiber divergence
        #s1 = self.kf.cov
        #s2 = np.array(Q)
        #d = 0.5*(np.matmul(np.linalg.inv(s1),s2).transpose() - len(s1.diagonal()) + np.log(np.linalg.det(s1) / np.linalg.det(s2)) )
        #print(d.diagonal())
        #[35, 35, 22] # thresholds
        #return False

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
