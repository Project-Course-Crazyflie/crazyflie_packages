#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import String, Empty, Int16
from crazyflie_driver.msg import Position

from milestone3.srv import MoveTo, MoveToResponse, MoveToRequest

class NavigationServer:
    def __init__(self):
        self.cf1_pose_top = rospy.Subscriber('/cf1/pose', PoseStamped, self.cf1_pose_callback)
        self.cf1_pose = None
        self.tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buf)

        self.navgoal_client = rospy.ServiceProxy('cf1/move_to_service', MoveTo)
        rospy.Subscriber('cf1/navigation/follow_path', PoseArray, self.follow_path_callback)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.move_to_callback)
        rospy.Subscriber("cf1/navigation/takeoff", Empty, self.takeoff_callback)
        rospy.Subscriber("cf1/navigation/land", Empty, self.land_callback)
        rospy.Subscriber("cf1/navigation/spin", Empty, self.spin_motion_callback)
        rospy.Subscriber("cf1/navigation/marker_goal", Int16, self.marker_goal_callback)

    def cf1_pose_callback(self, pose):
        self.cf1_pose = pose

    def navgoal_call(self, goal, pos_thres=0.2, yaw_thres=0.1, vel_thres=0.1, vel_yaw_thres=0.05, duration=0):
    #def navgoal_call(self, goal, pos_thres, yaw_thres, vel_thres, vel_yaw_thres, duration):
        rospy.wait_for_service('cf1/move_to_service')
        try:
            #navgoal_client = rospy.ServiceProxy('cf1/move_to_service', MoveTo)
            req = MoveToRequest()
            req.goal = goal
            req.pos_thres = pos_thres
            req.yaw_thres = yaw_thres
            req.vel_thres = vel_thres
            req.vel_yaw_thres = vel_yaw_thres
            req.duration = duration
            resp = self.navgoal_client(goal=goal, 
                                       pos_thres=pos_thres, 
                                       yaw_thres=yaw_thres, 
                                       vel_thres=vel_thres, 
                                       vel_yaw_thres=vel_yaw_thres, 
                                       duration=float(duration))
            #resp = self.navgoal_client(req)
            return resp.at_goal
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

    def follow_path_callback(self, pose_array):
        for p in pose_array.poses:
            # do something with the orientation
            goal = p
            self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.1, vel_thres=0.1, vel_yaw_thres=0.05, duration=0)

    def move_to_callback(self, goal):
        # Convenience method to be able to pusblish navgoal via the topic cf1/move_to without using the service
        resp = self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.05, duration=0.1)
        print("Moving: {}".format(resp.data))

    def takeoff_callback(self, _):
        if self.cf1_pose is None or abs(self.cf1_pose.pose.position.z) < 0.1:
            goal = PoseStamped()
            goal.header.frame_id = "cf1/base_link"
            goal.pose.position.z = 0.5
            goal.pose.orientation.w = 1 # this keeps the orientation unchanged!
            resp = self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.1, duration=5)
            print("Takeoff: {}".format(resp.data))
        else:
            print("I'm already flying you greedy pancake!")

    def land_callback(self, _):
        if self.cf1_pose is None or abs(self.cf1_pose.pose.position.z) > 0.1:
            goal = PoseStamped()
            goal.header.frame_id = "cf1/base_link"
            goal.pose.position.z = -(self.cf1_pose.pose.position.z)
            goal.pose.orientation.w = 1
            resp = self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.5, vel_thres=0.01, vel_yaw_thres=0.01, duration=5)
            print("Landing: {}".format(resp.data))
        else:
            print("Are you serious!?")

    def spin_motion_callback(self, _):
        # Obviously spins the drone XD
        angle_inc = 2*np.pi/3
        goal = PoseStamped()
        goal.header.frame_id = "cf1/base_link"
        (goal.pose.orientation.x, 
         goal.pose.orientation.y, 
         goal.pose.orientation.z, 
         goal.pose.orientation.w) = quaternion_from_euler(0, 0, angle_inc)

         # in case of crazy drift during spin, we define final pose in 
         # map so that it returns to original pose (shouldnt be necessary irl)
        while not rospy.is_shutdown():
            try: final_pose = self.tf_buf.transform(self.cf1_pose, 'map')
            except: print("Can't transform cf1/pose to map yet...")
            else: break
        
        # spin 2 thirds
        print("Spinning...")
        r1 = self.navgoal_call(goal, pos_thres=0.5, yaw_thres=0.5, vel_thres=1, vel_yaw_thres=100, duration=3.0)
        print("Spin 1/3: {}".format(r1.data))
        r2 = self.navgoal_call(goal, pos_thres=0.5, yaw_thres=0.5, vel_thres=1, vel_yaw_thres=100, duration=2.0)
        print("Spin 2/3: {}".format(r2.data))
        # final goal
        # it might take a long time (over 20 sec) for the drone to reach the final orientation in simulation. Could be that its spinning really fast...
        r3 = self.navgoal_call(final_pose, pos_thres=0.2, yaw_thres=0.1, vel_thres=0.1, vel_yaw_thres=0.1, duration=3.0)
        print("Spin 3/3: {}".format(r3.data))

    def marker_goal_callback(self, marker_id):
        # TODO: make useful for signs as well
        aruco_frame = "aruco/marker" + str(marker_id.data)

        while not self.cf1_pose: rospy.loginfo("Waiting for cf1/pose...")

        if not self.tf_buf.can_transform(aruco_frame, 'map', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % aruco_frame)
            return

        aruco_pose = PoseStamped()
        aruco_pose.header.frame_id = aruco_frame
        aruco_pose.header.stamp = rospy.Time.now()
        p0 = self.tf_buf.transform(aruco_pose, "map")
        aruco_pose.pose.position.y = rospy.get_param("navigation/dist_to_marker")
        p1 = self.tf_buf.transform(aruco_pose, "map")

        goal = PoseStamped()
        goal.header.frame_id = aruco_frame

        q = self.yaw_towards_frame(p1, target_frame=aruco_frame, T=None)

        goal_map = PoseStamped()
        goal_map.header.stamp = rospy.Time.now()
        goal_map.header.frame_id = "map"
        goal_map.pose.position.x = p1.pose.position.x
        goal_map.pose.position.y = p1.pose.position.y
        goal_map.pose.position.z = p1.pose.position.z # use p0 here to keep the crazyflie on the same height as the aruco marker
        goal_map.pose.orientation.x = q[0]
        goal_map.pose.orientation.y = q[1]
        goal_map.pose.orientation.z = q[2]
        goal_map.pose.orientation.w = q[3]


        """
        aruco_y_odom = np.array([p1.pose.position.x-p0.pose.position.x,
                                 p1.pose.position.y-p0.pose.position.y,
                                 p1.pose.position.z-p0.pose.position.z])

        if y_odom.pose.position.z < 0.1:
            # y axle is parallell with x,y odom and aruco is treated as being on a wall
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = p1.pose.position.x, p1.pose.position.y, p0.pose.position.z

        else:
            # treated as being on the floor
            pass
        """

        resp = self.navgoal_call(goal_map)
        print("Moved to marker {}: {}".format(marker_id.data, resp.data))

    def yaw_towards_frame(self, pose, target_frame, T=None):
        """Return the yaw towards frame represented in a quaternion (list)
        """
        # TODO: send in transform T as argument instead of calculating here
        if not self.tf_buf.can_transform(target_frame, 'map', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % target_frame)
            return

        if pose.header.frame_id != "map":
            pose = self.tf_buf.transform(pose, 'map')

        frame_pose = PoseStamped() # [0, 0, 0]
        frame_pose.header.stamp = rospy.Time(0) # .now()?
        frame_pose.header.frame_id = target_frame

        p = self.tf_buf.transform(frame_pose, 'map')

        frame_pos = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
        curr = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        diff = frame_pos-curr
        theta = np.arctan2(diff[1], diff[0])

        q_rot = quaternion_from_euler(0, 0, theta,"sxyz")

        return q_rot

if __name__ == '__main__':
    rospy.init_node('navigation_server')
    NavigationServer()
    rospy.spin()
