#!/usr/bin/env python

#Tools
import math
from random import uniform, randint
import numpy as np
import os

#ROS tools
import rospy
import json

#ROS messages
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, ColorRGBA, Bool
from crazyflie_driver.msg import Position
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from milestone3.srv import PlanPath, PlanPathResponse

#TF
from tf.transformations import * 
import tf2_geometry_msgs
import tf2_ros

#Plotting with matplotlib
import ast
from shapely.geometry import LineString, box, Polygon, Point



xlb = -10
ylb = -10
xub = 10
yub = 10

animation = True
plot_dilated_walls = []


def path_callback(req):
    start_pos, end_pos = req.start_pos, req.end_pos
    global plot_dilated_walls
    
    path_array = Path()
    path_array.header.frame_id = 'map'

    current_start = start_pos
    current_start.header.stamp = rospy.Time.now()
   
    start_point = [current_start.pose.position.x, current_start.pose.position.y]
    rho = rospy.get_param("/path_planning/rho")
    inflation = rospy.get_param("/path_planning/inflation")
    try:
        rrt = RRT(start_point, 
                  goal_point=[end_pos.pose.position.x, end_pos.pose.position.y], 
                  obstacles=plot_dilated_walls, 
                  rho=rho, 
                  inflation=inflation,
                  sample_rate=5)
    except Exception as e:
        print(e)
        return PlanPathResponse(Path())


    path = rrt.rrt_planning(animation=False)

    if path == None:
        print('Cannot find path')
    else:
        for p in path:
            
            point = PoseStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = 'map'

            point.pose.position.x = p[0]
            point.pose.position.y = p[1]
            point.pose.position.z = 0.5

            path_array.poses.append(point)
        path_pub.publish(path_array)
        resp = PlanPathResponse()
        resp.path = path_array
        return resp


class RRT:
    """
    RRT PLANNING
    """

    class RRTnode:
        """
        RRT planning node
        """

        def __init__(self, x, y, yaw=0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.x_path = []
            self.y_path = []
            self.yaw_path = []
            self.parent = None

    def __init__(self, start_point, goal_point, obstacles, sample_rate, \
                 rho, inflation, path_resolution=0.05):
        
        self.start_node = self.RRTnode(start_point[0], start_point[1])
        self.start_node.x_path = [self.start_node.x]
        self.start_node.y_path = [self.start_node.y]
        self.inflation = inflation
        if not self.safe(self.start_node, obstacles):
            raise Exception("Start node is not safe!")
        self.goal_node = self.RRTnode(goal_point[0], goal_point[1])
        self.obstacles = obstacles
        
        self.min_rand_area = [xlb + self.inflation, ylb + self.inflation]
        self.max_rand_area = [xub - self.inflation, yub - self.inflation]
        self.rho = rho
        self.inflation = inflation
        self.path_resolution = path_resolution
        self. sample_rate = sample_rate
        self.node_list = []
    
    def rrt_planning(self, animation=True):
        self.node_list = [self.start_node]
        is_planning = True
        #i = 150 ,plot counter

        while is_planning is True:

            random_node = self.generate_random_node()

            #Get nearest node
            nearest_index = self.nearest_node_index(self.node_list, random_node)
            nearest_node = self.node_list[nearest_index]

            #Steer towards node c from b
            new_node = self.steer(nearest_node, random_node, self.rho)
    

            if self.safe(new_node, self.obstacles) == True:
                self.node_list.append(new_node)
            else: 
                continue
            if self.calculate_distance_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.rho:
                final_node = self.steer(self.node_list[-1], self.goal_node, self.rho)
                if self.safe(final_node, self.obstacles) == True:
                    is_planning = False
                    return self.generate_final_course(len(self.node_list) - 1)

        return None #Cannot find path

    
    def generate_random_node(self):
        if randint(0, 100) > self.sample_rate:
            random_node = self.RRTnode(uniform(self.min_rand_area[0], self.max_rand_area[0]), \
                uniform(self.min_rand_area[1], self.max_rand_area[1]))
        
        else: #weight towards goal with sample rate
            random_node = self.RRTnode(self.goal_node.x, self.goal_node.y, self.goal_node.yaw)
        
        return random_node
    
    def steer(self, from_node, to_node, expand_rho = float('inf')):
        new_node = self.RRTnode(from_node.x, from_node.y, from_node.yaw)
        to_distance, yaw = self.distance_and_angle(new_node, to_node)

        new_node.x_path = [new_node.x]        

        new_node.y_path = [new_node.y]
        new_node.yaw_path = [new_node.yaw]

        if expand_rho > to_distance:
            expand_rho = to_distance
        
        #floor: returns the floor of x as a float. Largest integer leq x
        node_expand = math.floor(expand_rho/self.path_resolution)

        for _ in range(int(node_expand)):
            new_node.x += self.path_resolution * math.cos(yaw)
            new_node.y += self.path_resolution * math.sin(yaw)
            new_node.yaw += self.path_resolution * math.tan(yaw)



            new_node.x_path.append(new_node.x)
            new_node.y_path.append(new_node.y)
            new_node.yaw_path.append(new_node.yaw)

        close_distance, to_yaw = self.distance_and_angle(new_node, to_node)

        if close_distance <= self.path_resolution:
            new_node.x_path.append(to_node.x)
            new_node.y_path.append(to_node.y)
            new_node.yaw_path.append(to_yaw) #eventually to_node's yaw but that can be tested

        new_node.parent = from_node

        return new_node
    
    def generate_final_course(self, goal_index):
        path = [[self.goal_node.x, self.goal_node.y]]
        node = self.node_list[goal_index]
        
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        
        return path
        
        
    def calculate_distance_to_goal(self, x, y):
        
        dist_x = x - self.goal_node.x
        dist_y = y - self.goal_node.y
        distance_g = math.hypot(dist_x, dist_y)

        return distance_g

    @staticmethod
    def safe_old(node, obstacles):
        if node == None:
            return False

        #TODO fix the damn thing
        for o in obstacles:
            o_list = o.exterior.xy
            for (xx, yy) in zip(o_list[0], o_list[1]):

                distx_list = [xx - x for x in node.x_path]
                disty_list = [yy - y for y in node.y_path]
                dist_list = [math.hypot(distx, disty) for (distx, disty) in zip(distx_list, disty_list)]

                for x,y in zip(node.x_path, node.y_path):
                    if 5.3 < x < 5.7 and 3.3 < y < 3.8:
                        print("Obs x,y: {},{}".format(xx, yy))
                        print("Dist: {}".format(math.sqrt((xx-x)**2 + (yy-y)**2)))
                            
                
                if min(dist_list) < 0.6:
                    return False #collision
        return True #safe

    def safe(self, node, obstacles):
        if node == None:
            return False

        for o in obstacles:
            for x,y in zip(node.x_path, node.y_path):
                dist = o.distance(Point(x,y,0))
                if dist < self.inflation:
                    return False #collision

        return True #safe

    @staticmethod
    def distance_and_angle(from_node, to_node):
        distx = to_node.x - from_node.x
        disty = to_node.y - from_node.y
        dist = math.hypot(distx, disty)

        yaw = math.atan2(disty, distx)

        return dist, yaw

    @staticmethod
    def nearest_node_index(node_list, random_node):
        #calculate nearest index
        distance_list = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2 for node in node_list]

        nearest_node_index = distance_list.index(min(distance_list))
        
        return nearest_node_index


def generate_and_publish_obstacles():
    global plot_dilated_walls
    """
    args = rospy.myargv(argv=argv)
    # Load world JSON
    w_name = rospy.get_param(rospy.get_name() + "/world_name")
    rospy.loginfo(w_name)

    with open(w_name, 'rb') as f:
    world = json.load(f)

    """
    global xlb, xub, ylb, yub
    #mapFilePath = "../../maps/tutorial_1.world.json"
    mapFilePath = rospy.get_param(rospy.get_name() + "/world_name")
    mapString = ""

    with open(os.path.join(os.path.dirname(__file__), mapFilePath), "r") as file:
        for line in file:  #for each row
            l = line.strip().replace(" ", "")  #remove all blankspace
            mapString += l
    
    
    world = ast.literal_eval(mapString)#convert string representation of read file into dictionary through some kind of black magic
    xlb, ylb = world["airspace"]["min"][:2] 
    xub, yub = world["airspace"]["max"][:2]

    plot_walls_list = []
    plot_dilated_walls = []

    for i ,o in enumerate(world['walls']):
        start_walls = o['plane']['start'][:2]
        stop_walls = o['plane']['stop'][:2]
        plot_walls_list.append(LineString([tuple(start_walls), tuple(stop_walls)]))
        plot_dilated_walls.append(plot_walls_list[i].buffer(0.1))
    print(plot_walls_list)
    obstacles_array = MarkerArray()
    
    """
    LineString.coords contains a list with two tuples within, start and stop points, respectively
    """
    for index, line in enumerate(plot_walls_list):
        obstacle = Marker()
        obstacle.header.stamp = rospy.Time.now()

        dy = line.coords[0][1] - line.coords[1][1]
        dx = line.coords[0][0] - line.coords[1][0]
        wall_yaw = math.atan2(dy, dx)

        cy = line.coords[0][1] + line.coords[1][1]
        cx = line.coords[0][0] + line.coords[1][0]
        obstacle.header.frame_id = 'map'
        obstacle.id = 40+index
        obstacle.type = obstacle.CUBE
        obstacle.action = obstacle.ADD
        
        obstacle.pose.position.x = cx/2
        obstacle.pose.position.y = cy/2
        obstacle.pose.position.z = 1.5

        (obstacle.pose.orientation.x, obstacle.pose.orientation.y, obstacle.pose.orientation.z, obstacle.pose.orientation.w) = quaternion_from_euler(0, 0 , wall_yaw)
        obstacle.scale.x = math.hypot(dx, dy)
        obstacle.scale.y = 0.1
        obstacle.scale.z = 3.0

        obstacle.color=ColorRGBA(0.0, 1.0, 0.0, 0.8)

        obstacles_array.markers.append(obstacle)
    
    obstacles_pub.publish(obstacles_array)


#Init publisher, subscriber, node 
rospy.init_node("RRT_ROS")

#Obstacles
obstacles_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

#Path
path_pub = rospy.Publisher('/move_base/TrajectoryPlanner', Path, queue_size=2)
rospy.Service('/cf1/path_planning/plan', PlanPath, path_callback)

rospy.sleep(2)

def main():
    generate_and_publish_obstacles()
    rospy.spin()

if __name__ == '__main__':
    main()
