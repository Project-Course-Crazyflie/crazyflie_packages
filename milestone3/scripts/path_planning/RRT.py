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
from std_msgs.msg import Header, ColorRGBA
from crazyflie_driver.msg import Position
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

#TF
import tf.transformations
import tf2_geometry_msgs
import tf2_ros

#Plotting in Python
import ast
from shapely.geometry import LineString, box, Polygon, Point
import matplotlib.pyplot as plt


xlb = -10
ylb = -10
xub = 10
yub = 10

animation = True

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
                 rho=0.5, path_resolution=0.1):
        
        self.start_node = self.RRTnode(start_point[0], start_point[1])
        self.goal_node = self.RRTnode(goal_point[0], goal_point[1])
        self.obstacles = obstacles
        self.min_rand_area = [xlb, ylb]
        self.max_rand_area = [xub, yub]
        self.rho = rho
        self.path_resolution = path_resolution
        self. sample_rate = sample_rate
        self.node_list = []
    
    def rrt_planning(self, animation=True):
        self.node_list = [self.start_node]
        is_planning = True
        i = 150 #plot counter

        while is_planning is True:

            random_node = self.generate_random_node()

            #Get nearest node
            nearest_index = self.nearest_node_index(self.node_list, random_node)
            nearest_node = self.node_list[nearest_index]

            #Steer towards node c from b
            new_node = self.steer(nearest_node, random_node, self.rho)
    

            if self.safe(new_node, self.obstacles) == True:
                self.node_list.append(new_node)
            
            if animation and i%5 == 0:
                self.draw_graph(random_node)
                i -=1
            
            if self.calculate_distance_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.rho:
                final_node = self.steer(self.node_list[-1], self.goal_node, self.rho)
                if self.safe(final_node, self.obstacles) == True:
                    is_planning = False
                    return self.generate_final_course(len(self.node_list) - 1)
            
            if animation and i%5==0:
                self.draw_graph(random_node)
                i -= 1

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
        
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        
        return path
    
    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event: [exit(0) if event.key == 'q' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.x_path, node.y_path, "-g")

        #outer points of the polygon
        for o in self.obstacles:
            xx, yy = o.exterior.xy
            plt.scatter(xx, yy)

        plt.plot(self.start_node.x, self.start_node.y, "xr")
        plt.plot(self.goal_node.x, self.goal_node.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand_area[0], self.max_rand_area[0],\
                  self.min_rand_area[1], self.max_rand_area[1]])
        plt.grid(True)
        plt.pause(0.01)


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
                dist_list = [math.sqrt(distx**2 + disty**2) for(distx, disty) in zip(distx_list, disty_list)]

                if min(dist_list) < 0.4:
                    return False #collision

        return True #safe

    @staticmethod
    def safe(node, obstacles):
        if node == None:
            return False

        #TODO fix the damn thing
        for o in obstacles:
            for x,y in zip(node.x_path, node.y_path):
                dist = o.distance(Point(x,y))
                if dist < 0.4:
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

def main(): #argv = sys.argv):
    global world
    """
    rospy.init_node('RRT')
  
    args = rospy.myargv(argv=argv)
    # Load world JSON
    w_name = rospy.get_param(rospy.get_name() + "/world_name")
    rospy.loginfo(w_name)

    with open(w_name, 'rb') as f:
        world = json.load(f)

    """
    mapFilePath = "../../maps/tutorial_1.world.json"
    mapString = ""

    with open(os.path.join(os.path.dirname(__file__), mapFilePath), "r") as file:
        for line in file:  #for each row
            l = line.strip().replace(" ", "")  #remove all blankspace
            mapString += l
    
    
    world = ast.literal_eval(mapString)#convert string representation of read file into dictionary through some kind of black magic
    

    plot_walls_list = []
    plot_dilated_walls = []

    for i,o in enumerate(world['walls']):
        start_walls = o['plane']['start'][:2]
        stop_walls = o['plane']['stop'][:2]
        plot_walls_list.append(LineString([tuple(start_walls), tuple(stop_walls)]))
        plot_dilated_walls.append(plot_walls_list[i].buffer(0.1))


    rrt = RRT(start_point=[0,0], goal_point=[7, 4], obstacles=plot_dilated_walls, sample_rate=5)
    path = rrt.rrt_planning(animation)

    if path == None:
        print('Cannot find path')
    else:
        print('Found path')
        print(path)

         # Draw final path
        if animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01) 
            plt.show()

if __name__ == '__main__':
    main()
