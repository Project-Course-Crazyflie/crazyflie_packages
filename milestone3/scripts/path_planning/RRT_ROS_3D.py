#!/usr/bin/env python

#Tools
import math
from random import uniform, randint
import numpy as np
import os

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

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
	print('path planner got request')
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


	path = rrt.rrt_planning(animation=True)
	if animation:
		rrt.draw_graph()
		plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')
		plt.show()

	if path == None:
		print('Cannot find path')
		return PlanPathResponse()
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
				 rho, inflation, path_resolution=0.1):

		self.start_node = self.RRTnode(start_point[0], start_point[1], start_point[2])
		self.start_node.x_path = [self.start_node.x]
		self.start_node.y_path = [self.start_node.y]
		self.start_node.z_path = [self.start_node.z]
		self.inflation = inflation
		print('start point RRT: ',start_point)
		if not self.safe(self.start_node, obstacles):
			raise Exception("Start node is not safe!")
		self.goal_node = self.RRTnode(goal_point[0], goal_point[1], goal_point[2])
		self.obstacles = obstacles

		self.min_rand_area = [xlb + self.inflation, ylb + self.inflation, zlb + self.inflation]
		self.max_rand_area = [xub - self.inflation, yub - self.inflation, zub - self.inflation]
		self.rho = rho
		self.inflation = inflation
		self.path_resolution = path_resolution
		self.sample_rate = sample_rate
		self.node_list = []

	def rrt_planning(self, animation=True):
		self.node_list = [self.start_node]
		is_planning = True
		i = 150 #,plot counter

		while is_planning is True:
			print('i',i)

			random_node = self.generate_random_node()

			#Get nearest node
			nearest_index = self.nearest_node_index(self.node_list, random_node)
			nearest_node = self.node_list[nearest_index]

			#Steer towards node c from b
			new_node = self.steer(nearest_node, random_node, self.rho)

			if animation and i%5 == 0:
				self.draw_graph(random_node)
				i -=1

			if self.safe(new_node, self.obstacles) == True:
				self.node_list.append(new_node)
			else:
				continue
			if self.calculate_distance_to_goal(self.node_list[-1].x, self.node_list[-1].y, self.node_list[-1].z) <= self.rho:
				final_node = self.steer(self.node_list[-1], self.goal_node, self.rho)
				if self.safe(final_node, self.obstacles) == True:
					is_planning = False
					return self.generate_final_course(len(self.node_list) - 1)

			if animation and i%5:
				self.draw_graph(random_node)
				i -= 1

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

		new_node.x_path = [new_node.x]
		new_node.y_path = [new_node.y]
		new_node.z_path = [new_node.z]
		new_node.yaw_path = [new_node.yaw]

		if expand_rho > to_distance:
			expand_rho = to_distance

		#floor: returns the floor of x as a float. Largest integer leq x
		node_expand = math.floor(expand_rho/self.path_resolution)

		for _ in range(int(node_expand)):
			new_node.x += ((to_node.x - from_node.x)*self.path_resolution)/to_distance
			new_node.y += ((to_node.y - from_node.y)*self.path_resolution)/to_distance
			new_node.z += ((to_node.z - from_node.z)*self.path_resolution)/to_distance

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

	def safe(self, node, obstacles):
		if node == None:
			return False
		if not node.parent: #assuming that the current pose is safe
			return True

		v_line = np.array([node.x - node.parent.x,node.y - node.parent.y,node.z - node.parent.z])
		p0 = np.array([node.parent.x, node.parent.y, node.parent.z])
		for o in obstacles:
			start = o[0]
			stop = o[1]
			corner = np.array(stop)
			corner[2] = start[2]

			u = start - corner
			v = stop - corner

			normal = np.cross(u,v)
			normal /= np.linalg.norm(normal)
			for i in range(-1,2,2):
				start1 = start + i*self.inflation*normal
				stop1 = stop + i*self.inflation*normal

				#t = -(normal[0]*(p0[0]-x0) +normal[1]*(p0[1]-y0) + normal[2]*(p0[2] -z0))/(Nx*v[0]+ Ny*v[1] +Nz*v[3]) + Nx(p0[0]-x0) +Ny(p0[1]-y0)
				direction = np.dot(normal,v_line)
				#print("direction", str(direction))
				if direction == 0:
					continue
				t = -np.dot(normal,p0-start1)/direction
				if t <= 1 and t >= 0:
					intersec = p0 + t*v_line
					#print("t", str(t))
					print("intersec", str(intersec))
					print("x" ,np.max([start1[0],stop1[0]]), np.min([start1[0],stop1[0]])) 
					print("y" ,np.max([start1[1],stop1[1]]), np.min([start1[1],stop1[1]]))
					print("z" ,np.max([start1[2],stop1[2]]), np.min([start1[2],stop1[2]]))  
					if intersec[0] <= np.max([start1[0],stop1[0]]) and intersec[0] >= np.min([start1[0],stop1[0]]) and \
						intersec[1] <= np.max([start1[1],stop1[1]]) and intersec[1] >= np.min([start1[1],stop1[1]]) and \
						intersec[2] <= np.max([start1[2],stop1[2]]) and intersec[2] >= np.min([start1[2],stop1[2]]):
						return False
		return True
			#

			#if intersec[0] < np.max(node.x,node.parent.x) and intersec[0] > np.min(node.x,node.parent.x) and \
			#	intersec[1] < np.max(node.y,node.parent.y) and intersec[1] > np.min(node.y,node.parent.y) and \
			#	intersec[2] < np.max(node.z,node.parent.z) and intersec[2] > np.min(node.z,node.parent.z)


		#convert node to vector


		'''
		x = p0[0] + v[0]*t
		y = p0[1] + v[1]*t
		z = p0[2] + v[2]*t

		Nx(x-x0) + Ny(y-y0) + Nz(z-z0) =0

		Nx( p0[0] + v[0]*t-x0) + Ny(p0[1] + v[1]*t-y0) + Nz(p0[2] + v[2]*t-z0) =0

		t*(Nx*v[0]+ Ny*v[1] +Nz*v[3]) + Nx(p0[0]-x0) +Ny(p0[1]-y0) + Nz(p0[2] -z0)=0

		t=...

		x = p0[0] + v[0]*t
		y = p0[1] + v[1]*t
		z = p0[2] + v[2]*t  --> find the point coordinates

		'''

	def safe_linda(self, node, obstacles):
		if node == None:
			return False
		for o in obstacles:
			start = o[0]
			stop = o[1]
			corner = np.array(stop)
			corner[2] = start[2]

			u = start - corner
			v = stop - corner
			#print(u, v)
			normal = np.cross(u,v)
			#print(normal)
			d = -corner.dot(normal)

			for xx, yy, zz in zip(node.x_path, node.y_path, node.z_path):
				point = np.array([xx, yy, zz])
				'''
				ax +by + cz +d =0
				ex + fy + g = 0

				ax + by + cz + d = ex + fy + g
				(a-e)*x + (b-f)*y + cz +(d-g) = 0
				'''
				check_plane = np.dot(point, normal) + d

				point_plane_distance = abs((np.dot(normal, point) + d)/math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2))
				#print("distance: " + str(point_plane_distance))
				if point_plane_distance < self.inflation:
					return False #point in plane, collision

		return True #safe

	def draw_graph(self, rnd=None):

		fig = plt.axes(projection="3d")

		if rnd is not None:
			fig.scatter(rnd.x, rnd.y, rnd.z, "^k")
		for node in self.node_list:
			if node.parent:
				fig.plot3D(node.x_path, node.y_path, node.z_path, "-g")

		for o in self.obstacles:
			start = o[0]
			stop = o[1]
			corner = np.array(stop)
			corner[2] = 0.0

			u = start - corner
			v = stop - corner

			normal = np.cross(u,v)

			d = -corner.dot(normal)

			if normal[0] == 0.0:
				xx, zz = np.meshgrid(np.linspace(start[0], stop[0], 2) , np.linspace(start[2], stop[2], 2))
				yy = (-normal[0]*xx - normal[2]*zz - d)*1./normal[1]

				fig.plot_surface(xx, yy, zz)
			elif normal[1] == 0.0:
				yy, zz = np.meshgrid(np.linspace(start[1], stop[1], 2) , np.linspace(start[2], stop[2], 2))
				xx = (-normal[1]*yy - normal[2]*zz - d)*1./normal[0]

				fig.plot_surface(xx, yy, zz)

		fig.scatter(self.start_node.x, self.start_node.y, self.start_node.z, "xr")
		fig.scatter(self.goal_node.x, self.goal_node.y, self.goal_node.z, "xr")
		fig.set_xlim3d(self.min_rand_area[0], self.max_rand_area[0])
		fig.set_ylim3d(self.min_rand_area[1], self.max_rand_area[1])
		fig.set_zlim3d(self.min_rand_area[2], self.max_rand_area[2])
		fig.set_aspect("equal")
		plt.pause(0.01)



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
