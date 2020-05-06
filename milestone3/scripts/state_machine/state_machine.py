#!/usr/bin/env python

import math
import time
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import String, Empty, Int16, Int32MultiArray, Bool
from crazyflie_driver.msg import Position


from milestone3.srv import MoveTo, MoveToRequest
from milestone3.srv import PlanPath, PlanPathRequest
from milestone3.srv import PlanAndFollowPath, PlanAndFollowPathRequest
from milestone3.srv import Spin, SpinRequest
from milestone3.srv import Land, LandRequest
from milestone3.srv import TakeOff, TakeOffRequest
from milestone3.srv import MoveToMarker, MoveToMarkerRequest
from milestone3.srv import MoveTo, MoveToRequest
from milestone3.srv import GetMarkerGoal, GetMarkerGoalRequest
from milestone3.srv import ResetKalmanFilter, ResetKalmanFilterRequest




class StateMachine:
    def __init__(self):
        """
        States:
        InitState - plan marker order, localize and takeoff
        Localize - localize drone by resetting the covariance of the kalman filter and spin if necessary
        PlanGoCheck - plan a path, go to marker, check it (spin)
        DoneState - land
        """
        self.path_plan_client = rospy.ServiceProxy('cf1/path_planning/plan', PlanPath)
        self.plan_and_follow_path_client = rospy.ServiceProxy('cf1/navigation/plan_and_follow_path', PlanAndFollowPath)
        self.takeoff_client = rospy.ServiceProxy("cf1/navigation/takeoff", TakeOff)
        self.land_client = rospy.ServiceProxy("cf1/navigation/land", Land)
        self.spin_client = rospy.ServiceProxy("cf1/navigation/spin", Spin)
        self.move_to_marker_client = rospy.ServiceProxy("cf1/navigation/move_to_marker", MoveToMarker)
        self.reset_kalman_client = rospy.ServiceProxy("cf1/localization/reset_kalman_filter", ResetKalmanFilter)
        self.get_marker_goal_client = rospy.ServiceProxy("cf1/navigation/get_marker_goal", GetMarkerGoal)
        self.move_to_client = rospy.ServiceProxy("cf1/navigation/move_to", MoveTo)
        #self.get_marker_pose_client = rospy.ServiceProxy("cf1/navigation/get_marker_pose", GetMarkerPose)

        rospy.Subscriber("cf1/localization/measurement_feedback", Int32MultiArray, self.measurement_fb_callback)
        self.measurement_fb_msg = []

        rospy.Subscriber("cf1/localization/converged", Bool, self.convergence_callback)
        self.convergence_msg = None

        rospy.Subscriber("cf1/pose", PoseStamped, self.cf1_pose_callback)
        self.cf1_pose = None

        self.checked_markers = []
        self.unchecked_markers = [16,17,18] # hardcoding for world hunger
        self.current_state = None
        self.abort = False
        self.is_localizing = False
        self.localize_start = None

    def measurement_fb_callback(self, msg):
        self.measurement_fb_msg.append(msg)

    def convergence_callback(self, msg):
        self.convergence_msg = msg

    def cf1_pose_callback(self, msg):
        self.cf1_pose = msg

    def verify_marker(self, marker_id, duration):
        self.measurement_fb_msg = []
        time_start = time.time()
        while not rospy.is_shutdown():
            if self.measurement_fb_msg:
                for m in self.measurement_fb_msg:
                    corr_mark = marker_id == m.data[0]
                    valid = m.data[1]
                    if corr_mark and valid:
                        return True

            if (time.time()-time_start) > duration:
                return False

    def verify_convergence(self, duration):
        self.convergence_msg = None
        time_start = time.time()
        while not rospy.is_shutdown():
            if self.convergence_msg:
                #print("Convergence msg: {}".format(self.convergence_msg.data))
                if self.convergence_msg.data == True:
                    return True
            if (time.time()-time_start) > duration:
                return False

    def abort_callback(self):
        self.abort = True

    def run_newnew(self):
        state = "init"
        while not rospy.is_shutdown():

            if state == "init":
                self.reset_kalman_client()
                print("Taking off")
                self.takeoff_client()
                #rospy.sleep(1) #pause remove
                state = "localize"

            elif state == "localize":
                print("Localizing")
                if not self.is_localizing:
                    self.is_localizing = True
                    self.localize_start = time.time()
                    self.convergence_msg = None
                else:
                    if self.convergence_msg:
                        if self.convergence_msg.data == True:
                            print("Localization complete!")
                            state = "go_to_next_marker"
                    elif time.time() - self.localize_start >= 5:
                        print("Localization failed")
                        state = "abort"

            elif state == "go_to_next_marker":
                if not self.unchecked_markers:
                    state = "done"
                    continue
                marker = self.unchecked_markers.pop(0)
                # do something with resp
                print("Planning to marker {}".format(marker))
                start_pos = self.cf1_pose
                try:
                    start_pos = self.tf_buf.transform(start_pos, "map", rospy.Duration(0.1))
                    end_pos = self.get_marker_goal_client(GetMarkerGoal(marker_id))
                except:
                    state = "abort"
                    continue
                # not necessary if we don't wanna look at the marker while flying
                #marker_pos = self.get_marker_pose_client(req.marker_id)
                resp = self.path_plan_client(start_pos, end_pos)
                if not len(resp.path.poses):
                    print("No path found!")
                    state = "abort"
                    continue

                rospy.wait_for_service('cf1/navgoal/move_to')
                print("Going to marker {}".format(marker))
                poses = list(reversed(resp.path.poses))
                for p_next in zip(poses):
                    if self.abort:
                        state = "abort"
                        break
                    p_next.pose.orientation = start_pos.pose.orientation
                    req = MoveToRequest(p_next, pos_thres=0.05, yaw_thres=0.3, vel_thres=0.1, vel_yaw_thres=0.05, duration=5)
                    self.move_to_client(req)

                if not self.abort:
                    print("End goal")
                    req = MoveToRequest(end_pos, pos_thres=0.1, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.05, duration=5)
                    resp = self.move_to_client(req)
                    print("At goal:" + str(resp.at_goal.data))
                    state = "verify_marker"

            elif state == "verify_marker":
                rospy.sleep(1)
                print("Looking for marker {}".format(marker))
                if self.verify_marker(marker, 3):
                    print("Found it!")
                    state = "spin"
                else:
                    print("Failed to find marker")
                    state = "abort"
                    continue
                #rospy.sleep(1) #pause remove
            elif state == "spin":
                print("Spinning")
                resp = self.spin_client()
                #rospy.sleep(1)
                #startspin = time.time()
                print("Wait for convergence after spinning")
                state = "localize"

            elif state == "abort":
                print("Aborting")
                print("Landing")
                self.land_client()
                return

            elif state == "done":
                print("Landing")
                self.land_client()

                print("State machine done!")
                return

    def run(self):
        state = "init"
        while not rospy.is_shutdown():
            if state == "init":
                self.reset_kalman_client()
                print("Taking off")
                self.takeoff_client()
                #rospy.sleep(1) #pause remove
                print("Localizing")
                if not self.verify_convergence(5):
                    print("Localization failed")
                    state = "abort"
                    continue
                print("Localization complete!")
                state = "go_to_next_marker"

            if state == "go_to_next_marker":
                if not self.unchecked_markers:
                    state = "done"
                    continue
                marker = self.unchecked_markers.pop(0)
                # do something with resp
                print("Planning to marker {}".format(marker))
                resp = self.plan_and_follow_path_client(marker)
                """
                while True:
                    try:
                        resp = self.plan_and_follow_path_client(marker)
                        #rospy.sleep(1) #pause remove
                    except:
                        pass #print("Failed to plan...")
                    else:
                        break
                """
                print("Going to marker {}".format(marker))
                # do something with resp
                # verify that marker is detected
                rospy.sleep(1)
                print("Looking for marker {}".format(marker))
                if self.verify_marker(marker, 3):
                    print("Found it!")
                else:
                    print("Failed to find marker")
                    state = "abort"
                    continue
                #rospy.sleep(1) #pause remove
                print("Spinning")
                resp = self.spin_client()
                rospy.sleep(1)
                print("Wait for convergence after spinning")
                if not self.verify_convergence(5):
                    print("Localization failed")
                    state = "abort"

            if state == "abort":
                print("Aborting")
                print("Landing")
                self.land_client()
                return

            if state == "done":
                print("Landing")
                self.land_client()

                print("State machine done!")
                return

    def run_test(self):
        state = "init"
        while not rospy.is_shutdown():
            if state == "init":
                print("Taking off")
                #self.takeoff_client()
                rospy.sleep(1) #pause remove
                print("Localizing")
                if not self.verify_convergence(10):
                    print("Localization failed")
                    state = "abort"
                    continue
                print("Localization complete!")
                state = "go_to_next_marker"

            if state == "go_to_next_marker":
                if not self.unchecked_markers:
                    state = "done"
                    continue
                marker = self.unchecked_markers.pop(0)
                # do something with resp
                print("Planning to marker {}".format(marker))
                while True:
                    try:
                        resp = self.plan_and_follow_path_client(marker)
                        #rospy.sleep(1) #pause remove
                    except:
                        print("Failed to plan...")
                    else:
                        break
                print("Going to marker {}".format(marker))
                # do something with resp
                # verify that marker is detected
                rospy.sleep(1)
                print("Looking for marker {}".format(marker))
                if self.verify_marker(marker, 10):
                    print("Found it!")
                else:
                    print("Failed to find marker")
                    state = "abort"
                    continue
                rospy.sleep(1) #pause remove
                print("Spinning")
                #resp = self.spin_client()
                rospy.sleep(1)
                print("Wait for convergence after spinning")
                if not self.verify_convergence(10):
                    print("Localization failed")
                    state = "abort"

            if state == "abort":
                print("Aborting")
                print("Landing")
                #self.land_client()
                rospy.sleep(1)
                return

            if state == "done":
                print("Landing")
                #self.land_client()
                rospy.sleep(1)

                print("State machine done!")
                return

if __name__ == '__main__':
    rospy.init_node('state_machine')
    sm = StateMachine()
    sm.run()
