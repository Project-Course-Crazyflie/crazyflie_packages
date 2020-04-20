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


from milestone3.srv import MoveTo, MoveToRequest
from milestone3.srv import PlanPath, PlanPathRequest
from milestone3.srv import PlanAndFollowPath, PlanAndFollowPathRequest
from milestone3.srv import Spin, SpinRequest
from milestone3.srv import Land, LandRequest
from milestone3.srv import TakeOff, TakeOffRequest
from milestone3.srv import MoveToMarker, MoveToMarkerRequest

class NavigationInput:
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

    def spin(self):
        while not rospy.is_shutdown():
            inp = input()
            try:
                inp = int(inp)
            except:
                pass
            else:
                self.plan_and_follow_path_client(inp)
                continue

            if inp == "l":
                self.land_client()
            elif inp == "t":
                self.takeoff_client()
            elif inp == "s":
                self.spin_client()

if __name__ == "__main__":
    NavigationInput().spin()