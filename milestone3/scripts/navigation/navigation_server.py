#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, TransformStamped, Vector3
from std_msgs.msg import String, Empty, Int8, Bool, Int32MultiArray
from crazyflie_driver.msg import Position

from aruco_msgs.msg import Marker, MarkerArray

from milestone3.srv import MoveTo, MoveToRequest
from milestone3.srv import TurnTowardsMarker, TurnTowardsMarkerResponse
from milestone3.srv import EmergencyMoveTo, EmergencyMoveToResponse, EmergencyMoveToRequest
from milestone3.srv import PlanPath, PlanPathResponse
from milestone3.srv import PlanAndFollowPath, PlanAndFollowPathResponse
from milestone3.srv import Spin, SpinResponse
from milestone3.srv import Land, LandResponse
from milestone3.srv import SearchAruco, SearchArucoResponse
from milestone3.srv import EmergencyLand, EmergencyLandResponse
from milestone3.srv import TakeOff, TakeOffResponse
from milestone3.srv import MoveToMarker, MoveToMarkerResponse

class NavigationServer:
    def __init__(self):
        self.cf1_pose_top = rospy.Subscriber('/cf1/pose', PoseStamped, self.cf1_pose_callback)
        self.cf1_pose = None
        self.tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buf)

        self.plath_plan_client = rospy.ServiceProxy('cf1/path_planning/plan', PlanPath)
        self.navgoal_client = rospy.ServiceProxy('cf1/navgoal/move_to', MoveTo)
        self.navgoal_emergency_client = rospy.ServiceProxy('cf1/navgoal/move_to_emergency', EmergencyMoveTo)
        self.stop_pub = rospy.Publisher("cf1/navgoal/stop", Empty, queue_size=1)

        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.move_to_callback)
        rospy.Service('cf1/navigation/plan_and_follow_path', PlanAndFollowPath, self.plan_and_follow_path_callback)
        rospy.Service("cf1/navigation/takeoff", TakeOff, self.takeoff_callback)
        rospy.Service("cf1/navigation/land", Land, self.land_callback)
        rospy.Service("cf1/navigation/emergency_land", EmergencyLand, self.emergency_land_callback)
        rospy.Service("cf1/navigation/spin", Spin, self.spin_motion_callback)
        rospy.Service("cf1/navigation/turn_towards_marker", TurnTowardsMarker, self.turn_towards_marker_callback)
        rospy.Service("cf1/navigation/search", SearchAruco, self.aruco_search_callback)



        rospy.Subscriber("cf1/localization/converged", Bool, self.convergence_callback)
        self.convergence_msg = None
        rospy.Subscriber("aruco/markers", MarkerArray, self.see_marker_callback)
        self.see_marker = None

        rospy.Subscriber("cf1/localization/measurement_feedback", Int32MultiArray, self.measurement_fb_callback)
        self.measurement_fb = None

    def measurement_fb_callback(self, msg):
        self.measurement_fb = msg

    def see_marker_callback(self, markers):
        self.see_marker = len(markers.markers) > 0

    def convergence_callback(self, msg):
        self.convergence_msg = msg

    def cf1_pose_callback(self, pose):
        self.cf1_pose = pose

    def navgoal_call(self, goal, pos_thres=0.2, yaw_thres=0.1, vel_thres=0.1, vel_yaw_thres=0.05, duration=0, emergency=False):
    #def navgoal_call(self, goal, pos_thres, yaw_thres, vel_thres, vel_yaw_thres, duration):
        #rospy.wait_for_service('cf1/navgoal/move_to')
        try:
            #navgoal_client = rospy.ServiceProxy('cf1/move_to_service', MoveTo)
            req = MoveToRequest()
            req.goal = goal
            req.pos_thres = pos_thres
            req.yaw_thres = yaw_thres
            req.vel_thres = vel_thres
            req.vel_yaw_thres = vel_yaw_thres
            req.duration = duration
            if not emergency:
                resp = self.navgoal_client(goal=goal,
                                           pos_thres=pos_thres,
                                           yaw_thres=yaw_thres,
                                           vel_thres=vel_thres,
                                           vel_yaw_thres=vel_yaw_thres,
                                           duration=float(duration))
            else:
                resp = self.navgoal_emergency_client(goal=goal,
                                                     pos_thres=pos_thres,
                                                     yaw_thres=yaw_thres,
                                                     vel_thres=vel_thres,
                                                     vel_yaw_thres=vel_yaw_thres,
                                                     duration=float(duration))

            #print(resp)
            return resp.at_goal.data
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            return False

    def plan_and_follow_path_callback(self, req):
        # TODO: change to just follow path
        start_pos = self.cf1_pose
        start_pos.header.stamp = rospy.Time(0)
        if not self.tf_buf.can_transform("cf1/odom", 'map', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from cf1/odom to map')
            return PlanAndFollowPathResponse(Bool(False))
        start_pos = self.tf_buf.transform(start_pos, "map", rospy.Duration(1.0))
        #start_trans = self.tf_buf.look(start_pos, "map", rospy.Duration(1.0))
        if not start_pos:
            print("Need a cf1/pose...")
            return PlanAndFollowPathResponse(Bool(False))
        end_pos = self.get_marker_goal(req.marker_id)
        marker_pos = self.get_marker_pose(req.marker_id)
        rospy.wait_for_service('cf1/navgoal/move_to')
        #print("Planning")
        resp = self.plath_plan_client(start_pos, end_pos)
        if not len(resp.path.poses):
            print("No path found!")
            return PlanAndFollowPathResponse(Bool(False))

        #print("Going")
        poses = list(resp.path.poses)
        for p_curr, p_next in zip(poses, poses[1:]):
            # old
            #(p_next.pose.orientation.x,
            # p_next.pose.orientation.y,
            # p_next.pose.orientation.z,
            # p_next.pose.orientation.w) = self.yaw_towards_pose(p_curr, marker_pos)

            # new
            self.turn_towards_pose(p_curr, marker_pos)

             # TODO: currently ignoring if not reached position, change that
            self.navgoal_call(p_next, pos_thres=0.1, yaw_thres=0.3, vel_thres=0.1, vel_yaw_thres=0.05, duration=5)
        #print("End goal")
        at_goal = self.navgoal_call(end_pos, pos_thres=0.1, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.05, duration=5)
        print("Done")
        return PlanAndFollowPathResponse(Bool(at_goal))

    def move_to_callback(self, req):
        # Convenience method to be able to pusblish navgoal via the topic cf1/move_to without using the service
        resp = self.navgoal_call(req, pos_thres=0.2, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.05, duration=0.1)
        print("Moving: {}".format(resp))
        return resp

    def takeoff_callback(self, _):
        if self.cf1_pose is None or abs(self.cf1_pose.pose.position.z) < 0.1:
            goal = PoseStamped()
            goal.header.frame_id = "cf1/base_link"
            goal.pose.position.z = 0.5
            goal.pose.orientation.w = 1 # this keeps the orientation unchanged!
            resp = self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.1, duration=10)
            print("Takeoff: {}".format(resp))
            return TakeOffResponse(Bool(resp))
        else:
            print("FU, I do not wanna flie!")
            return TakeOffResponse(Bool(False))

    def land_callback(self, _):
        if self.cf1_pose is None or abs(self.cf1_pose.pose.position.z) > 0.1:
            goal = PoseStamped()
            goal.header.frame_id = "cf1/base_link"
            goal.pose.position.z = -(self.cf1_pose.pose.position.z)
            goal.pose.orientation.w = 1
            resp = self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.5, vel_thres=0.01, vel_yaw_thres=0.01, duration=5)
            self.stop_pub.publish()
            #goal.pose.position.z = -10
            #resp = self.navgoal_call(goal, pos_thres=100, yaw_thres=5, vel_thres=0.01, vel_yaw_thres=0.01, duration=5)

            print("Landing: {}".format(resp))
            return LandResponse(Bool(resp))
        else:
            rospy.logwarn("Do not have pose or is already landed")
            return LandResponse(Bool(False))

    def emergency_land_callback(self, _):
        if self.cf1_pose is None or abs(self.cf1_pose.pose.position.z) > 0.1:
            goal = PoseStamped()
            goal.header.frame_id = "cf1/base_link"
            goal.pose.position.z = -(self.cf1_pose.pose.position.z)
            goal.pose.orientation.w = 1
            resp = self.navgoal_call(goal, pos_thres=0.2, yaw_thres=0.5, vel_thres=0.01, vel_yaw_thres=0.01, duration=2, emergency=True)
            self.stop_pub.publish()
            #goal.pose.position.z = -10
            #resp = self.navgoal_call(goal, pos_thres=100, yaw_thres=5, vel_thres=0.01, vel_yaw_thres=0.01, duration=5)

            print("Landing: {}".format(resp))
            resp = EmergencyLandResponse(Bool(resp))
            #print(resp)
            return resp
        else:
            rospy.logwarn("Emergency: Do not have pose or is already landed")
            return EmergencyLandResponse(Bool(False))

    def turn_towards_marker_callback(self, req):
        """Mostly for testing turn_towards_pose
        """
        start_pos = self.cf1_pose
        marker_pos = self.get_marker_pose(req.marker_id)
        self.turn_towards_pose(start_pos, marker_pos)
        return TurnTowardsMarkerResponse(Bool(True)) # always returns true


    def turn_towards_pose(self, p_from, p_towards):
        """Returns a list of goals with incremental angles to restrict the spinning speed.
        """
        
        q_yaw = self.yaw_towards_pose(p_from, p_towards)
        _, _, final_yaw = euler_from_quaternion(q_yaw)
        _, _, curr_yaw = euler_from_quaternion([p_from.orientation.x, p_from.orientation.y,
                                                p_from.orientation.z, p_from.orientation.w])

        diff = final_yaw - curr_yaw
        diff = (diff + 180) % 360 - 180

        n = 25
        yaw_inc = 2*np.pi/float(n)

        ratio = float(diff)/float(yaw_inc)

        yaw_inc *= np.sign(ratio) # might be wrong
        n_semi_turns = abs(int(ratio))
        q_inc = quaternion_from_euler(0, 0, yaw_inc)
        
        goals = [p_from]
        while n_semi_turns >= 0:
            
            goal = PoseStamped()
            goal.header = p_from.header
            goal.pose.position = p_from.pose.position
            
            if n_semi_turns == 0:
                goal.pose.orientation = Quaternion(*q_yaw)
            else:
                goal.pose.orientation = Quaternion(*quaternion_multiply([goals[-1].pose.orientation.x,
                                                                        goals[-1].pose.orientation.y,
                                                                        goals[-1].pose.orientation.z,
                                                                        goals[-1].pose.orientation.w],
                                                                        q_inc))
            #goals.append(goal)
            self.navgoal_call(goal, pos_thres=0.1, yaw_thres=0.2, vel_thres=0.1, vel_yaw_thres=0.05, duration=5)
        #return goals
            

    def spin_motion_callback(self, _):
        # Obviously spins the drone XD
        n = 20
        angle_inc = 2*np.pi/float(n)

        q_inc = quaternion_from_euler(0, 0, angle_inc)

         # in case of crazy drift during spin, we define final pose in
         # map so that it returns to original pose (shouldnt be necessary irl)
        try: final_pose = self.tf_buf.transform(self.cf1_pose, 'map',rospy.Duration(1))
        except:
            rospy.logerr("Can't transform cf1/pose to map yet...")
            return SpinResponse(Bool(False))

        # spin 2 thirds
        #print("Spinning...")
        goal = self.cf1_pose
        for i in range(n-1):
            goal.pose.orientation = Quaternion(*quaternion_multiply([goal.pose.orientation.x,
                                                                    goal.pose.orientation.y,
                                                                    goal.pose.orientation.z,
                                                                    goal.pose.orientation.w],
                                                                    q_inc))
            r = self.navgoal_call(goal, pos_thres=0.1, yaw_thres=0.1, vel_thres=1, vel_yaw_thres=100, duration=1.0)
            #print("Spin {}/{}: {}".format(i+1, n, r))
        # final goal
        # it might take a long time (over 20 sec) for the drone to reach the final orientation in simulation. Could be that its spinning really fast...
        r = self.navgoal_call(final_pose, pos_thres=0.2, yaw_thres=0.1, vel_thres=0.1, vel_yaw_thres=0.1, duration=3.0)
        #print("Spin {}/{}: {}".format(n, n, r))
        #print("Spin goal: {}".format(final_pose))
        return SpinResponse(Bool(r))

    def aruco_search_callback(self, _):
        # find closest aruco marker
        # find the sign of marker's angle (from base)
        #

        self.convergence_msg = None
        #self.convergence_msg
        # Spin the drone and stop to watch the beautiful view(aruco markers)
        n = 20
        angle_inc = 2*np.pi/float(n)

        q_inc = quaternion_from_euler(0, 0, angle_inc)

         # in case of crazy drift during spin, we define final pose in
         # map so that it returns to original pose (shouldnt be necessary irl)
        try: final_pose = self.tf_buf.transform(self.cf1_pose, 'map',rospy.Duration(1))
        except:
            rospy.logerr("Can't transform cf1/pose to map yet...")
            return SpinResponse(Bool(False))

        #print("Spinning...")

        goal = self.cf1_pose
        self.see_marker = False
        rospy.sleep(0.1)
        for i in range(n):
            if self.see_marker == True:
                self.see_marker = False
                rospy.sleep(0.1)
                if self.see_marker:
                    print('I see a marker')
                    return SearchArucoResponse(Bool(True))
            #print('spinning ', i)
            goal.pose.orientation = Quaternion(*quaternion_multiply([goal.pose.orientation.x,
                                                                    goal.pose.orientation.y,
                                                                    goal.pose.orientation.z,
                                                                    goal.pose.orientation.w],
                                                                    q_inc))
            r = self.navgoal_call(goal, pos_thres=0.1, yaw_thres=0.1, vel_thres=1, vel_yaw_thres=100, duration=1.0)



        return SearchArucoResponse(Bool(False))

    def marker_goal_callback(self, req):

        # TODO: make useful for signs as well
        # Remove and use get_marker_goal instead?
        aruco_frame = "aruco/marker" + str(req.marker_id)

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

        q = self.yaw_towards_frame(p1, target_frame=aruco_frame)

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
        print("Moved to marker {}: {}".format(req.marker_id, resp))
        return MoveToMarkerResponse(Bool(resp))


    def get_marker_goal(self, marker_id):
        # TODO: make useful for signs as well
        if marker_id == 16:
            object_frame = "sign/" + 'dangerous_curve_left'
        elif marker_id == 17:
            object_frame = "sign/" + 'no_bicycle'
        elif marker_id == 18:
            object_frame = "sign/" + 'warning_roundabout'
        else:
            object_frame = "aruco/marker" + str(marker_id)

        while not self.cf1_pose: rospy.loginfo("Waiting for cf1/pose...")

        if not self.tf_buf.can_transform(object_frame, 'map', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % object_frame)
            return

        aruco_pose = PoseStamped()
        aruco_pose.header.frame_id = object_frame
        aruco_pose.header.stamp = rospy.Time.now()
        p0 = self.tf_buf.transform(aruco_pose, "map")
        aruco_pose.pose.position.y = rospy.get_param("navigation/dist_to_marker")
        p1 = self.tf_buf.transform(aruco_pose, "map")

        goal = PoseStamped()
        goal.header.frame_id = object_frame

        q = self.yaw_towards_frame(p1, target_frame=object_frame)

        goal_map = PoseStamped()
        goal_map.header.stamp = rospy.Time.now()
        goal_map.header.frame_id = "map"
        goal_map.pose.position.x = p1.pose.position.x
        goal_map.pose.position.y = p1.pose.position.y
        goal_map.pose.position.z = p0.pose.position.z # use p0 here to keep the crazyflie on the same height as the aruco marker
        goal_map.pose.orientation.x = q[0]
        goal_map.pose.orientation.y = q[1]
        goal_map.pose.orientation.z = q[2]
        goal_map.pose.orientation.w = q[3]
        return goal_map

    def get_marker_pose(self, marker_id):
        # TODO: make useful for signs as well
        if marker_id == 16:
            object_frame = "sign/" + 'dangerous_curve_left'
        elif marker_id == 17:
            object_frame = "sign/" + 'no_bicycle'
        elif marker_id == 18:
            object_frame = "sign/" + 'warning_roundabout'
        else:
            object_frame = "aruco/marker" + str(marker_id)

        while not self.cf1_pose: rospy.loginfo("Waiting for cf1/pose...")

        if not self.tf_buf.can_transform(object_frame, 'map', rospy.Time(0), rospy.Duration(1.0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % object_frame)
            raise Exception("WTF?!?!?{}".format(object_frame))


        aruco_pose = PoseStamped()
        aruco_pose.header.frame_id = object_frame
        aruco_pose.header.stamp = rospy.Time.now()
        p0 = self.tf_buf.transform(aruco_pose, "map")
        return p0


    def yaw_towards_frame(self, pose, target_frame):
        """Return the yaw towards frame (in map frame) represented in a quaternion (list)
        """
        if not self.tf_buf.can_transform(target_frame, 'map', rospy.Time(0), rospy.Duration(1.0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % target_frame)
            return

        if pose.header.frame_id != "map":
            pose = self.tf_buf.transform(pose, 'map')

        frame_pose = PoseStamped() # [0, 0, 0]
        frame_pose.header.stamp = rospy.Time(0) # .now()?
        frame_pose.header.frame_id = target_frame

        frame_pose = self.tf_buf.transform(frame_pose, 'map')

        return self.yaw_towards_pose(pose, frame_pose)

    def yaw_towards_pose(self, pose, target_pose):
        """
        Calculates the yaw between pose and target_pose in map
        """
        if pose.header.frame_id != target_pose.header.frame_id:
            rospy.loginfo("Pose and target pose must have the same frame_id!")
            return

        target_pos = np.array([target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z])
        curr = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        diff = target_pos-curr
        theta = np.arctan2(diff[1], diff[0])

        q_rot = quaternion_from_euler(0, 0, theta,"sxyz")

        return q_rot

if __name__ == '__main__':
    rospy.init_node('navigation_server')
    NavigationServer()
    rospy.spin()
