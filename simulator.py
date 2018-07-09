import Tkinter as tk

# import sys
# import math
# import random

# from linalgebra import *
# from rrt import *

class Simulator(object):
	def __init__(self, root, obstacles, rrt):
		self.canvas = None
		self.root = root

		self.canvas_width = 400
		self.canvas_height = 400

		self.obstacles = obstacles
		self.rrt = rrt

		self.obstacle_pointers = {}
		self.centroid_pointers = {}
		self.rrt_node_pointers = {}
		self.rrt_connection_pointers = {}

		self.start_draw()

	def start_draw(self):
		self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height)
		self.canvas.pack()

		stop = tk.Button(self.root, text='Exit')
		stop.pack(side='bottom')
		stop.bind('<Button-1>', self.stop_prog)

		draw = tk.Button(self.root, text='Start', command= lambda: self.rrt.update(0))
		draw.pack(side='bottom')
		# draw.bind('<Button-1>', self.display_sim)

	def stop_prog(self, event=None):
		self.root.quit()

	def display_sim(self, t, event=None):
		self.draw_obstacles(t)
		self.draw_rrt()
		# self.draw_base()

	# draws a dot for the robot
	def draw_base(self):
		self.canvas.create_oval(self.draw_dot((
			self.rrt.base[0], 
			self.rrt.base[1]),
			self.rrt.size), 
			fill='green')

	# draws the rrt by looping over each node and each node's connections to other nodes
	def draw_rrt(self):
		for node, connections in self.rrt.data.items():

			color = 'dark green' if node.valid else 'red'

			if not node in self.rrt_node_pointers:
				self.rrt_node_pointers[node] = self.canvas.create_oval(self.draw_dot((
					node.loc[0],
					node.loc[1]),
					node.size),
					fill=color)
			else:
				self.canvas.itemconfig(self.rrt_node_pointers[node], fill=color)

			# draw the connections too
			for connection in connections:
				
				color = 'dark green' if connection.valid else 'red'

				invert = self.invert(connection)
				to_draw = connection # whichever connection to draw
				if connection.valid and not invert.valid:
					to_draw = invert # always draw an invalid connection if you can

				if not to_draw.start in self.rrt_connection_pointers:
					# get the actual node, not the name of it
					other_node = to_draw.end
					node = to_draw.start

					self.rrt_connection_pointers[node] = self.canvas.create_line(
						node.loc[0],
						node.loc[1],
						other_node.loc[0],
						other_node.loc[1],
						fill=color,
						width=4)
				else:
					self.canvas.itemconfig(self.rrt_node_pointers[to_draw.start], fill=color)

	# returns the connection which goes end -> start
	def invert(self, connection):
		name = self.rrt.connect_name(connection.end, connection.start)
		return self.rrt.connects[name]

	# loops over the obstacles and draws them in turn at time = t
	def draw_obstacles(self, t):
		for obstacle in self.obstacles:
			a = obstacle.velocity[2]
			absolute_obs = obstacle.absolute_pos(t)
			# print "obstacle: ", absolute_obs
			absolute_points = []

			for abs_point in absolute_obs.points:
				# we need to loop over this since we have vectors, not a list of points
				absolute_points.append(abs_point[0])
				absolute_points.append(abs_point[1])

			if not obstacle.t0 in self.obstacle_pointers:
				obstacle_pointer = self.canvas.create_polygon(absolute_points, fill='blue')
				# use t0 as a key, since we can assume no two shapes start atop each other
				self.obstacle_pointers[obstacle.t0] = obstacle_pointer
			else:
				# modify the existing obstacle
				obstacle_pointer = self.obstacle_pointers.get(obstacle.t0)
				self.canvas.coords(obstacle_pointer, tuple(absolute_points))
				# print "absolute points: ", absolute_points

			if not obstacle.t0 in self.centroid_pointers:
				# draw a dot at the centroid of the shape
				# need to add to the t0 point to get the absolute location
				absolute_centroid = self.draw_dot(obstacle.centroid(t).add(obstacle.t0), 3)
				self.centroid_pointers[obstacle.t0] = self.canvas.create_oval(absolute_centroid, 
					fill="cyan", outline="")
			else:
				centroid_pointer = self.centroid_pointers.get(obstacle.t0)
				absolute_centroid = self.draw_dot(obstacle.centroid(t).add(obstacle.t0), 3)
				# print "abs, ", absolute_centroid
				self.canvas.coords(centroid_pointer, absolute_centroid)

	# there is no built-in method for drawing a dot, so this implements one
	# returns the coordinates for a dot
	def draw_dot(self, coords, size):
		return (coords[0] - size, coords[1] - size, coords[0] + size, coords[1] + size)