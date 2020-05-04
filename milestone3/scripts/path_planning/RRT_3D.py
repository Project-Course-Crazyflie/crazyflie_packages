#!/usr/bin/env python

#Tools
import math
from random import uniform, randint
import numpy as np
import os

#ROS tools
#import rospy
import json

"""
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
"""
#Plotting in Python
import ast
from shapely.geometry import LineString, box, Polygon, Point
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


#lower bounds
xlb = 0
ylb = 0
zlb = 0
#Upper bound
xub = 2.04
yub = 4
zub = 2


animation = True

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
				 rho=0.5, path_resolution=0.1):

		self.start_node = self.RRTnode(start_point[0], start_point[1], start_point[2])
		self.goal_node = self.RRTnode(goal_point[0], goal_point[1], goal_point[2])
		self.start_node.x_path = [self.start_node.x]
		self.start_node.y_path = [self.start_node.y]
		self.start_node.z_path = [self.start_node.z]
		self.obstacles = obstacles
		self.min_rand_area = [xlb, ylb, zlb]
		self.max_rand_area = [xub, yub, zub]
		self.rho = rho
		self.path_resolution = path_resolution
		self. sample_rate = sample_rate
		self.node_list = []
		self.inflation = path_resolution

	def rrt_planning(self, animation=True):
		self.node_list = [self.start_node]
		is_planning = True
		i = 150 #plot counter

		while is_planning is True:

			random_node = self.generate_random_node()
			#print(random_node.x, random_node.y, random_node.z)
			#print("generating new node")

			#Get nearest node
			nearest_index = self.nearest_node_index(self.node_list, random_node)
			nearest_node = self.node_list[nearest_index]

			#Steer towards node c from b
			new_node = self.steer(nearest_node, random_node, self.rho)
			#print("steering")
			if self.safe(new_node, self.obstacles) == True:
				self.node_list.append(new_node)


			if animation and i%5 == 0:
				self.draw_graph(random_node)
				i -=1


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
										uniform(self.min_rand_area[1], self.max_rand_area[1]),\
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

			#Append the new coordinates to the path list
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

		#Backtrack to generate final path
		while node.parent is not None:
			path.append([node.x, node.y, node.z])
			node = node.parent

		return path

	def draw_graph(self, rnd=None):
		"""

		----------------------------
				2D PLOTTING
		----------------------------

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
			plt.fill(xx, yy)


		plt.plot(self.start_node.x, self.start_node.y, self.start_node.z, "xr")
		plt.plot(self.goal_node.x, self.goal_node.y, self.goal_node.z, "xr")
		plt.axis("equal")
		plt.axis([self.min_rand_area[0], self.max_rand_area[0],
				  self.min_rand_area[1], self.max_rand_area[1],
				  self.min_rand_area[2], self.max_rand_area[2]])
		plt.grid(True)
		plt.pause(0.01)
		"""

		"""
		----------------------------
				3D PLOTTING
		----------------------------
		"""

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
			corner[2] = start[2]

			u = start - corner
			v = stop - corner

			normal = np.cross(u,v)
			normal /= np.linalg.norm(normal)
			#print(normal)

			d = -corner.dot(normal) 
			
			if normal[0] == 0.0:
				xx, zz = np.meshgrid(np.linspace(start[0], stop[0], 3) , np.linspace(start[2], stop[2], 3))
				yy = (-normal[0]*xx - normal[2]*zz - d)*1./normal[1]

				fig.plot_surface(xx, yy, zz)
			elif normal[1] == 0.0:
				yy, zz = np.meshgrid(np.linspace(start[1], stop[1], 3) , np.linspace(start[2], stop[2], 3))
				xx = (-normal[1]*yy - normal[2]*zz - d)*1./normal[0]

				fig.plot_surface(xx, yy, zz)
			elif normal[2] == 0.0:
				#print(o)
				xx, zz = np.meshgrid(np.linspace(start[0], stop[0], 3) , np.linspace(start[2], stop[2], 3))
				yy = (-normal[0]*xx - normal[2]*zz - d)*1./normal[1]

				#print(xx, yy, zz)

				fig.plot_surface(xx, yy, zz)

		fig.scatter(self.start_node.x, self.start_node.y, self.start_node.z, "xr")
		fig.scatter(self.goal_node.x, self.goal_node.y, self.goal_node.z, "xr")
		fig.set_xlim3d(-5, 5)
		fig.set_ylim3d(-5, 5)
		fig.set_zlim3d(0, 3)
		fig.set_xlabel('x')
		fig.set_ylabel('y')
		fig.set_zlabel('z')
		#fig.set_xlim3d(self.min_rand_area[0], self.max_rand_area[0])
		#fig.set_ylim3d(self.min_rand_area[1], self.max_rand_area[1])
		#fig.set_zlim3d(self.min_rand_area[2], self.max_rand_area[2])
		fig.set_aspect("equal")
		plt.pause(0.01)


	def calculate_distance_to_goal(self, x, y, z):

		distx_g = x - self.goal_node.x
		disty_g = y - self.goal_node.y
		distz_g = z - self.goal_node.z

		distance_g = math.sqrt(distx_g**2 + disty_g**2 + distz_g**2)

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
	def safe_joar(node, obstacles):
		if node == None:
			return False

		for o in obstacles:
			for xx, yy, zz in zip(node.x_path, node.y_path, node.z_path):
				dist = o.distance(Point(xx, yy, zz))
				if dist < 0.1:
					return False #collision

		return True #safe

	@staticmethod
	def safe_linda(node, obstacles):
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
					#print("intersec", str(intersec))
					#print("x" ,np.max([start1[0],stop1[0]]), np.min([start1[0],stop1[0]])) 
					#print("y" ,np.max([start1[1],stop1[1]]), np.min([start1[1],stop1[1]]))
					#print("z" ,np.max([start1[2],stop1[2]]), np.min([start1[2],stop1[2]]))  
					if intersec[0] <= np.max([start1[0],stop1[0]]) and intersec[0] >= np.min([start1[0],stop1[0]]) and \
						intersec[1] <= np.max([start1[1],stop1[1]]) and intersec[1] >= np.min([start1[1],stop1[1]]) and \
						intersec[2] <= np.max([start1[2],stop1[2]]) and intersec[2] >= np.min([start1[2],stop1[2]]):
						return False
		return True


	@staticmethod
	def distance_and_angle(from_node, to_node):
		distx = to_node.x - from_node.x
		disty = to_node.y - from_node.y
		distz = to_node.z - from_node.z
		dist = math.sqrt(distx**2 + disty**2 + distz**2)

		#from_vec = [from_node.x, from_node.y, from_node.z]
		#to_vec = [to_node.x, to_node.y, to_node.z]
		#print(from_vec)


		#from_vec_unit = from_vec/np.linalg.norm(from_vec)
		#to_vec_unit = to_vec/np.linalg.norm(to_vec)

		#dot = np.dot(from_vec_unit, to_vec_unit)
		#print(dot)

		#cos_yaw_3d = np.dot(from_vec, to_vec)/(np.linalg.norm(from_vec)*np.linalg.norm(to_vec))
		#yaw_3d = np.arccos(dot)

		yaw = math.atan2(disty, distx)

		return dist, yaw

	@staticmethod
	def nearest_node_index(node_list, random_node):
		#calculate nearest index

		distance_list = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2 + (node.z - random_node.z)**2 for node in node_list]
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
	mapFilePath = "../../maps/gandalfs_room.world.json"#"../../maps/tutorial_1.world.json"
	mapString = ""

	with open(os.path.join(os.path.dirname(__file__), mapFilePath), "r") as file:
		for line in file:  #for each row
			l = line.strip().replace(" ", "")  #remove all blankspace
			mapString += l


	world = ast.literal_eval(mapString)#convert string representation of read file into dictionary through some kind of black magic


	#plot_walls_list = []
	#plot_dilated_walls = []
	walls = []

	for i,o in enumerate(world['walls']):
		start_walls = np.array(o['plane']['start'])
		stop_walls = np.array(o['plane']['stop'])
		walls.append([start_walls, stop_walls])
		#plot_walls_list.append(LineString([tuple(start_walls), tuple(stop_walls)]))
		#plot_dilated_walls.append(plot_walls_list[i].buffer(0.1))
	#print(walls)

	rrt = RRT(start_point=[0.0, 1.0, 0.5], goal_point=[3.0, 1.0, 0.5], obstacles=walls, sample_rate=5)
	path = rrt.rrt_planning(animation)
	print(path)

	if path == None:
		print('Cannot find path')
	else:
		print('Found path')

	path.append([0.0, 1.0, 0.5])


	# Draw final path
	if animation:
		rrt.draw_graph()
		plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')
		plt.show()

if __name__ == '__main__':
	main()

"""
NOTES: The final path is not plotted and the walls are not plotted correctly. Also for the obstacle avoidance do a point/plane saftey distance. Change the path resolution and rho
"""
