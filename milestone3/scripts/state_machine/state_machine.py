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




class StateMachine:
    def __init__(self):
        """
        States:
        InitState - plan marker order, localize and takeoff
        Localize - localize drone by resetting the covariance of the kalman filter and spin if necessary
        PlanGoCheck - plan a path, go to marker, check it (spin)
        DoneState - land
        """
        self.plath_plan_client = rospy.ServiceProxy('cf1/path_planning/plan', PlanPath)
        self.plan_and_follow_path_client = rospy.ServiceProxy('cf1/navigation/plan_and_follow_path', PlanAndFollowPath)
        self.takeoff_client = rospy.ServiceProxy("cf1/navigation/takeoff", TakeOff)
        self.land_client = rospy.ServiceProxy("cf1/navigation/land", Land)
        self.spin_client = rospy.ServiceProxy("cf1/navigation/spin", Spin)
        self.move_to_marker_client = rospy.ServiceProxy("cf1/navigation/move_to_marker", MoveToMarker)

        rospy.Subscriber("cf1/localization/measurement_feedback", Int32MultiArray, self.measurement_fb_callback)
        self.measurement_fb_msg = None

        rospy.Subscriber("cf1/localization/converged", Bool, self.convergence_callback)
        self.convergence_msg = None

        self.checked_markers = []
        self.unchecked_markers = [1, 2,3,4,5,6,7]
        self.current_state = None

    def measurement_fb_callback(self, msg):
        self.measurement_fb_msg = msg

    def convergence_callback(self, msg):
        self.convergence_msg = msg

    def verify_marker(self, marker_id, duration):
        self.measurement_fb_msg = None
        time_start = time.time()
        while not rospy.is_shutdown():
            if self.measurement_fb_msg:
                corr_mark = self.measurement_fb_msg.data[0]
                valid = self.measurement_fb_msg.data[1]
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

    def run(self):
        state = "init"
        while not rospy.is_shutdown():
            if state == "init":
                print("Taking off")
                self.takeoff_client()
                #rospy.sleep(1) #pause remove
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
                if self.verify_marker(marker, 3):
                    print("Found it!")
                else:
                    print("Failed to find marker")
                    state = "abort"
                    continue
                rospy.sleep(1) #pause remove
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

if __name__ == '__main__':
    rospy.init_node('state_machine')
    sm = StateMachine()
    sm.run()
