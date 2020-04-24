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

animation = True
walls = []


def path_callback(req):
	start_pos, end_pos = req.start_pos, req.end_pos
	global walls
	
	path_array = Path()
	path_array.header.frame_id = 'map'

	current_start = start_pos
	current_start.header.stamp = rospy.Time.now()
   
	start_point = [current_start.pose.position.x, current_start.pose.position.y, current_start.pose.position.z]
	rho = rospy.get_param("/path_planning/rho")
	inflation = rospy.get_param("/path_planning/inflation")
	try:
		rrt = RRT(start_point, 
				  goal_point=[end_pos.pose.position.x, end_pos.pose.position.y, end_pos.pose.position.z], 
				  obstacles=walls, 
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
			point.pose.position.z = p[2]

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

		def __init__(self, x, y, z, yaw=0):
			self.x = x
			self.y = y
			self.z = z
			self.yaw = yaw
			self.x_path = []
			self.y_path = [] 
			self.z_path = []
			self.yaw_path = []
			self.parent = None

	def __init__(self, start_point, goal_point, obstacles, sample_rate, \
				 rho, inflation, path_resolution=0.05):
		
		self.start_node = self.RRTnode(start_point[0], start_point[1], start_point[2])
		self.start_node.x_path = [self.start_node.x]
		self.start_node.y_path = [self.start_node.y]
		self.start_node.z_path = [self.start_node.z]
		self.inflation = inflation
		if not self.safe(self.start_node, obstacles):
			raise Exception("Start node is not safe!")
		self.goal_node = self.RRTnode(goal_point[0], goal_point[1], goal_point[2])
		self.obstacles = obstacles
		
		self.min_rand_area = [xlb + self.inflation, ylb + self.inflation, zlb + self.inflation]
		self.max_rand_area = [xub - self.inflation, yub - self.inflation, zub - self.inflation]
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
			if self.calculate_distance_to_goal(self.node_list[-1].x, self.node_list[-1].y, self.node_list[-1].z) <= self.rho:
				final_node = self.steer(self.node_list[-1], self.goal_node, self.rho)
				if self.safe(final_node, self.obstacles) == True:
					is_planning = False
					return self.generate_final_course(len(self.node_list) - 1)

		return None #Cannot find path

	
	def generate_random_node(self):
		if randint(0, 50) > self.sample_rate:
			random_node = self.RRTnode(uniform(self.min_rand_area[0], self.max_rand_area[0]), \
									uniform(self.min_rand_area[1], self.max_rand_area[1]), \
									uniform(self.min_rand_area[2], self.max_rand_area[2]))
		
		else: #weight towards goal with sample rate
			random_node = self.RRTnode(self.goal_node.x, self.goal_node.y, self.goal_node.z, self.goal_node.yaw)
		
		return random_node
	
	def steer(self, from_node, to_node, expand_rho = float('inf')):
		new_node = self.RRTnode(from_node.x, from_node.y, from_node.z, from_node.yaw)
		to_distance, yaw = self.distance_and_angle(new_node, to_node)
		step_z = self.path_resolution

		new_node.x_path = [new_node.x]        
		new_node.y_path = [new_node.y]
		new_node.z_path = [new_node.z]
		new_node.yaw_path = [new_node.yaw]

		if expand_rho > to_distance:
			expand_rho = to_distance

		if from_node.z > to_node.z:
			step_z = -self.path_resolution
		
		#floor: returns the floor of x as a float. Largest integer leq x
		node_expand = math.floor(expand_rho/self.path_resolution)

		for _ in range(int(node_expand)):
			new_node.x += self.path_resolution * math.cos(yaw)
			new_node.y += self.path_resolution * math.sin(yaw)
			new_node.z += step_z
			new_node.yaw += self.path_resolution * math.tan(yaw)

			new_node.x_path.append(new_node.x)
			new_node.y_path.append(new_node.y)
			new_node.z_path.append(new_node.z)
			new_node.yaw_path.append(new_node.yaw)

		close_distance, to_yaw = self.distance_and_angle(new_node, to_node)

		if close_distance <= self.path_resolution:
			new_node.x_path.append(to_node.x)
			new_node.y_path.append(to_node.y)
			new_node.z_path.append(to_node.z)
			new_node.yaw_path.append(to_yaw) #eventually to_node's yaw but that can be tested

		new_node.parent = from_node

		return new_node
	
	def generate_final_course(self, goal_index):
		path = [[self.goal_node.x, self.goal_node.y, self.goal_node.z]]
		node = self.node_list[goal_index]
		
		while node is not None:
			path.append([node.x, node.y, node.z])
			node = node.parent
		
		return path
		
		
	def calculate_distance_to_goal(self, x, y, z):
		
		dist_x = x - self.goal_node.x
		dist_y = y - self.goal_node.y
		dist_z = z - self.goal_node.z
		distance_g = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)

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

	def safe_joar(self, node, obstacles):
		if node == None:
			return False

		for o in obstacles:
			for x,y in zip(node.x_path, node.y_path):
				dist = o.distance(Point(x,y,0))
				if dist < self.inflation:
					return False #collision

		return True #safe
	
	@staticmethod
	def safe(node, obstacles):
		if node == None:
			return False
		for o in obstacles:
			start = o[0]
			stop = o[1]
			corner = np.array(stop)
			corner[2] = 0.0

			u = start - corner
			v = stop - corner
			#print(u, v)
			normal = np.cross(u,v)
			#print(normal)
			d = -corner.dot(normal)

			for xx, yy, zz in zip(node.x_path, node.y_path, node.z_path):
				point = np.array([xx, yy, zz])
				
				check_plane = np.dot(point, normal) + d

				point_plane_distance = abs((np.dot(normal, point) + d)/math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2))
				#print("distance: " + str(point_plane_distance))
				if check_plane == 0.0 or point_plane_distance < 0.1:
					return False #point in plane, collision
		
		return True #safe

	

	@staticmethod
	def distance_and_angle(from_node, to_node):
		distx = to_node.x - from_node.x
		disty = to_node.y - from_node.y
		distz = to_node.z - from_node.z

		dist = math.sqrt(distx**2 + disty**2 + distz**2)

		yaw = math.atan2(disty, distx)

		return dist, yaw

	@staticmethod
	def nearest_node_index(node_list, random_node):
		#calculate nearest index
		distance_list = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2 + (node.z - random_node.z)**2 for node in node_list]

		nearest_node_index = distance_list.index(min(distance_list))
		
		return nearest_node_index


def generate_and_publish_obstacles():
	global walls
	"""
	args = rospy.myargv(argv=argv)
	# Load world JSON
	w_name = rospy.get_param(rospy.get_name() + "/world_name")
	rospy.loginfo(w_name)

	with open(w_name, 'rb') as f:
	world = json.load(f)

	"""
	global xlb, xub, ylb, yub, zlb, zub
	#mapFilePath = "../../maps/tutorial_1.world.json"
	mapFilePath = rospy.get_param(rospy.get_name() + "/world_name")
	mapString = ""

	with open(os.path.join(os.path.dirname(__file__), mapFilePath), "r") as file:
		for line in file:  #for each row
			l = line.strip().replace(" ", "")  #remove all blankspace
			mapString += l
	
	
	world = ast.literal_eval(mapString)#convert string representation of read file into dictionary through some kind of black magic
	xlb, ylb, zlb = world["airspace"]["min"]
	xub, yub, zub = world["airspace"]["max"]

	for o in world['walls']:
		walls.append([o['plane']['start'], o['plane']['stop']])

	obstacles_array = MarkerArray()
	
	"""
	LineString.coords contains a list with two tuples within, start and stop points, respectively
	"""
	for index, line in enumerate(walls):
		obstacle = Marker()
		obstacle.header.stamp = rospy.Time.now()
		
		dy = line[0][1] - line[1][1]
		dx = line[0][0] - line[1][0]
		#dy = line.coords[0][1] - line.coords[1][1]
		#dx = line.coords[0][0] - line.coords[1][0]
		wall_yaw = math.atan2(dy, dx)

		cy = line[0][1] + line[1][1]
		cx = line[0][0] + line[1][0]
		cz = line[0][2] + line[1][2]
		obstacle.header.frame_id = 'map'
		obstacle.id = 40+index
		obstacle.type = obstacle.CUBE
		obstacle.action = obstacle.ADD
		
		obstacle.pose.position.x = cx/2
		obstacle.pose.position.y = cy/2
		obstacle.pose.position.z = cz/2

		(obstacle.pose.orientation.x, obstacle.pose.orientation.y, obstacle.pose.orientation.z, obstacle.pose.orientation.w) = quaternion_from_euler(0, 0 , wall_yaw)
		obstacle.scale.x = math.hypot(dx, dy)
		obstacle.scale.y = 0.1
		obstacle.scale.z = line[1][2]

		obstacle.color=ColorRGBA(249, 105, 14, 1)

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