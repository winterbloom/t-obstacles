import Tkinter as tk

import math

from linalgebra import *

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
		self.rrt_label_pointers = {}
		self.timestamp_pointer = None

		self.start_draw()

	def start_draw(self):
		self.root.geometry(str(self.canvas_width + 100) + "x" + str(self.canvas_height))

		self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height)
		self.canvas.pack(side='left')

		stop = tk.Button(self.root, text='Exit')
		stop.pack(side='bottom')
		stop.bind('<Button-1>', self.stop_prog)

		more = tk.Button(self.root, text='More Time')
		more.pack(side='bottom')
		more.bind('<Button-1>', self.more_time)
		self.more = more

		draw = tk.Button(self.root, text='Start') #, command= lambda: self.rrt.update(0))
		draw.pack(side='bottom')
		draw.bind('<Button-1>', self.start_prog)

		self.canvas.bind("<Button-1>", self.set_goal)

	def set_goal(self, event):
		self.rrt.goal = Vector((event.x, event.y))
		self.start_prog()

	def more_time(self, event=None):
		# old_time = self.rrt.max_time
		# self.rrt.max_time += 10
		# self.canvas.itemconfig(self.more, to=self.rrt.max_time)
		# self.rrt.create_rrt(old_time)
		pass

	def start_prog(self, event=None):
		visited = self.rrt.create_rrt()

		# self.rrt.max_time = (visited[0].t + visited[0].len) + 2

		# print "Done at ", self.rrt.max_time
		# print "Path:"
		# for item in visited:
		# 	print item

		self.rrt.max_time = 50

		self.draw_goal()

		time = tk.Scale(from_=0, to=self.rrt.max_time, length=(self.canvas_height - 25), resolution=.1, 
			activebackground='orchid3', troughcolor='orchid1', state=tk.DISABLED,
			command= lambda _: self.rrt.update(time.get()))
		time.pack(side='right')
		self.time = time
		self.time.config(state="normal")

	def stop_prog(self, event=None):
		self.root.quit()

	def display_sim(self, t, event=None):
		self.draw_rrt(t)
		self.draw_obstacles(t)
		self.draw_base()
		self.draw_timestamp(t)

		# self.root.after(int(self.rrt.time_step * 1000), self.display_sim, t + self.rrt.time_step)

	# displays a timestamp in the upper left corner
	def draw_timestamp(self, t):
		if not self.timestamp_pointer:
			self.timestamp_pointer = self.canvas.create_text(30, 10, text="t = " + str(t))
		else:
			self.canvas.itemconfig(self.timestamp_pointer, text="t = " + str(t))

	# draws a dot for the robot
	def draw_base(self):
		self.canvas.create_oval(self.draw_dot((
			self.rrt.base[0], 
			self.rrt.base[1]),
			self.rrt.size), 
			fill='green')

	# draws a dot for the goal node
	def draw_goal(self):
		self.canvas.create_oval(self.draw_dot((
			self.rrt.goal[0], 
			self.rrt.goal[1]),
			self.rrt.size), 
			fill='dodger blue')
		self.canvas.create_text(
			self.rrt.goal[0],
			self.rrt.goal[1] - 14,
			fill='black',
			text='Goal')

	# draws the rrt by looping over each node and each node's connections to other nodes
	def draw_rrt(self, t):
		for node, connections in self.rrt.data.items():

			color = 'PaleGreen1' if node.valid else 'salmon'

			# draw the connections too
			self.draw_connections(t, connections)

			if node and (node.t + node.len) <= t:
				if not node in self.rrt_node_pointers: # first time
					self.rrt_node_pointers[node] = self.canvas.create_oval(self.draw_dot((
						node.loc[0],
						node.loc[1]),
						node.size),
						fill=color)
					self.rrt_label_pointers[node] = self.canvas.create_text(
						node.loc[0], 
						node.loc[1] - 14, 
						fill='black',
						text=str(math.ceil((node.t + node.len)*10)/10)) # round to one decimal place
				else: # all later instances
					self.canvas.itemconfig(self.rrt_node_pointers[node], fill=color, outline=color)
					self.canvas.itemconfig(self.rrt_label_pointers[node], fill='black')
			elif self.rrt_node_pointers.get(node):
				self.canvas.itemconfig(self.rrt_node_pointers[node], fill='white', outline='white')
				self.canvas.itemconfig(self.rrt_label_pointers[node], fill='white')
				self.canvas.tag_lower(self.rrt_label_pointers[node])
				self.canvas.tag_lower(self.rrt_node_pointers[node])

	def draw_connections(self, t, connections):
		for connection in connections:
				
			color = 'PaleGreen1' if connection.valid else 'salmon'

			# description of the connection
			node = connection.start
			connect_name = self.rrt.connect_name(connection.start, connection.end)
			connect_pointer = self.rrt_connection_pointers.get(connect_name)

			if connection and node and (connection.t + connection.len) <= t:
				if not connect_pointer:
					# get the actual node, not the name of it
					other_node = connection.end

					connect_pointer = self.canvas.create_line(
						node.loc[0],
						node.loc[1],
						other_node.loc[0],
						other_node.loc[1],
						fill=color,
						width=4)
					self.rrt_connection_pointers[connect_name] = connect_pointer
					self.canvas.tag_lower(connect_pointer)
				else:
					self.canvas.itemconfig(connect_pointer, fill=color)
			elif self.rrt_connection_pointers.get(connect_name):
				self.canvas.itemconfig(connect_pointer, fill='white')
				self.canvas.tag_lower(connect_pointer)

	# returns the connection which goes end -> start
	def invert(self, connection):
		name = self.rrt.connect_name(connection.end, connection.start)
		return self.rrt.connects[name]

	# loops over the obstacles and draws them in turn at time = t
	def draw_obstacles(self, t):
		for obstacle in self.obstacles:
			a = obstacle.velocity[2]
			absolute_obs = obstacle.absolute_pos(t)
			absolute_points = []

			for abs_point in absolute_obs.points:
				# we need to loop over this since we have vectors, not a list of points
				absolute_points.append(abs_point[0])
				absolute_points.append(abs_point[1])

			if not obstacle.t0 in self.obstacle_pointers:
				obstacle_pointer = self.canvas.create_polygon(absolute_points, fill='light blue')
				# use t0 as a key, since we can assume no two shapes start atop each other
				self.obstacle_pointers[obstacle.t0] = obstacle_pointer
				self.canvas.tag_raise(obstacle_pointer)
			else:
				# modify the existing obstacle
				obstacle_pointer = self.obstacle_pointers.get(obstacle.t0)
				self.canvas.coords(obstacle_pointer, tuple(absolute_points))

			if not obstacle.t0 in self.centroid_pointers:
				# draw a dot at the centroid of the shape
				# need to add to the t0 point to get the absolute location
				absolute_centroid = self.draw_dot(obstacle.centroid(t).add(obstacle.t0), 3)
				self.centroid_pointers[obstacle.t0] = self.canvas.create_oval(absolute_centroid, 
					fill="steel blue", outline="")
			else:
				centroid_pointer = self.centroid_pointers.get(obstacle.t0)
				absolute_centroid = self.draw_dot(obstacle.centroid(t).add(obstacle.t0), 3)
				self.canvas.coords(centroid_pointer, absolute_centroid)

	# there is no built-in method for drawing a dot, so this implements one
	# returns the coordinates for a dot
	def draw_dot(self, coords, size):
		return (coords[0] - size, coords[1] - size, coords[0] + size, coords[1] + size)