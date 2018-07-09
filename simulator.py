import Tkinter as tk

import sys
import math
import random

from linalgebra import *
from rrt import *

class Simulator(object):
	def __init__(self, root, obstacles, robot):
		self.canvas = None
		self.root = root

		self.canvas_width = 400
		self.canvas_height = 400
		self.time_step = .5 # in seconds

		self.obstacles = obstacles
		self.robot = robot

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

		draw = tk.Button(self.root, text='Start', command= lambda: self.display_sim(0))
		draw.pack(side='bottom')
		# draw.bind('<Button-1>', self.display_sim)

	def stop_prog(self, event=None):
		self.root.quit()

	def display_sim(self, t, event=None):
		self.add_branches()

		self.draw_obstacles(t)
		self.draw_rrt()
		self.draw_robot()

		# wait one time_step, then redraw
		self.root.after(int(self.time_step * 1000), self.display_sim, t + self.time_step, False)

	# creates a series of random branches off of each existing node
	def add_branches(self):
		for key in self.robot.rrt.keys():
			add_branch = random.random() <= Tune.branch_creation
			if add_branch:
				self.robot.add_branch(key.name)

	# draws a dot for the robot
	def draw_robot(self):
		self.canvas.create_oval(self.draw_dot((
			self.robot.loc[0], 
			self.robot.loc[1]),
			self.robot.size), 
			fill='green')

	# draws the rrt by looping over each node and each node's connections to other nodes
	def draw_rrt(self):
		for node, connections in self.robot.rrt.items():
			if not node in self.rrt_node_pointers:
				color = 'dark green' if node.valid else 'red'

				self.rrt_node_pointers[node] = self.canvas.create_oval(self.draw_dot((
					node.loc[0],
					node.loc[1]),
					node.size),
					fill=color)

			# draw the connections too
			for connection in connections:
				if not node in self.rrt_connection_pointers:
					# get the actual node, not the name of it
					other_node = self.robot.name_to_node[connection.end]

					color = 'dark green' if connection.valid else 'red'

					self.rrt_connection_pointers[node] = self.canvas.create_line(
						node.loc[0],
						node.loc[1],
						other_node.loc[0],
						other_node.loc[1],
						fill=color,
						width=4)

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

def main():

	null = Vector((0, 0, 0))

	# one pentagon, one not square to the axes, and one with a velocity
	ob1 = Shape((
		Vector((-40, -40)), 
		Vector((40, -40)), 
		Vector((60, 0)), 
		Vector((40, 40)), 
		Vector((-40, 40))
		),
		Vector((100, 100, 0)), null)

	ob2 = Shape((
		Vector((0, -40)), 
		Vector((40, 0)), 
		Vector((0, 40)), 
		Vector((-40, 0))
		), 
		Vector((200, 240, 0)), null)

	ob3 = Shape((
		Vector((-50, -20)), 
		Vector((50, -20)), 
		Vector((50, 20)), 
		Vector((-50, 20))
		), 
		Vector((300, 300, 0)), Vector((6, 0, math.pi/10)))

	obstacles = (ob1, ob2, ob3)

	# print ob1.centroid(0)

	# print ob3.rotate(math.pi/2)

	# matrix_test = Matrix((Vector((2, 0)), Vector((0, 1))))
	# print matrix_test.mult(Vector((2, 5)))

	root = tk.Tk()
	robot = RRT()
	robot.add_node(robot.loc, [])
	sim = Simulator(root, obstacles, robot)

	root.mainloop()

if __name__ == "__main__":
	sys.exit(main())