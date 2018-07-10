import Tkinter as tk

import random
import sys

from linalgebra import *
from simulator import *

class RRT(object):

	# pixels traveled per second by simbot
	traversal_rate = 50

	# length of randomly created branches
	branch_len_min = 20.0
	branch_len_max = 180.0

	# the smaller, the fewer branches are created
	branch_weight = 10

	# update in seconds
	time_step = .5

	# the maximum time the simulator runs to
	max_time = 10

	def __init__(self, root):
		self.size = 7
		self.speed = 20
		self.base = Vector((200, 200))

		self.sim = None
		self.root = root

		self.rrt_index = 0

		self.data = {}
		self.connects = {}
		self.name_to_node = {} # converts a name to a node

		# probability of creating a new branch off of an existing one, checked each loop cycle
		self.branch_creation = self.update_branch_creation()

	def __str__(self):
		str_list = []
		str_list.append("RRT: [")

		for key, values in self.data.items():
			str_list.append(str(key) + " ")
			for value in values:
				str_list.append(str(value) + " ")

		str_list.append("]")
		return ''.join(str_list)

	def create_rrt(self):
		# need a second node to be able to run validity
		self.add_node(self.base, [], 0)
		self.add_branch(0, 0)

		for i in range(1, int(self.max_time / self.time_step)):
			self.add_branches(i * self.time_step)

		self.update(0)
						

	def update(self, t):

		# print self

		base = self.name_to_node.get(0)

		if self.sim and base:
			self.validity(self.add_connect(None, base, 0), t)
			self.sim.display_sim(t)

		# wait one time_step, then redraw
		# self.root.after(int(self.time_step * 1000), self.update, t + self.time_step)

	# creates a node which is at the given x, y; connected to connections; and has a name
	def add_node(self, loc, connection_nodes, time):
		new_node = Node(self.rrt_index, loc, time)

		# loop over names of connections and create a new one

		connections = []
		for connection_node in connection_nodes:
			connections.append(self.add_connect(new_node, connection_node, time))
		self.data[new_node] = connections

		self.name_to_node[self.rrt_index] = new_node

		# print "node t: ", time
		self.rrt_index += 1

		return new_node

	def add_connect(self, start, end, time):
		new_connect = Connection(start, end, time)
		
		# print "connect t: ", time

		new_connect_name = self.connect_name(start, end)
		self.connects[new_connect_name] = new_connect
		return new_connect

	def node_name(self, node):
		return str(node.name) if node else "None"

	def connect_name(self, start, end):
		return self.node_name(start) + ":" + self.node_name(end)

	# creates a series of random branches off of each existing node
	def add_branches(self, time):
		for key in self.data.keys():
			add_branch = random.random() <= self.update_branch_creation()
			if add_branch and key.valid:
				self.add_branch(key.name, time)

	# creates a branch in a random direction with given name off of given trunk
	def add_branch(self, trunk_name, time):
		trunk = self.name_to_node[trunk_name]

		# random number between [0, 2 pi), measured counterlockwise from the horizontal
		rand_a = random.random() * 2.0*math.pi

		while True:
			rand_dist = random.random() * self.branch_len_max + self.branch_len_min

			rand_x = math.cos(rand_a) * rand_dist
			rand_y = math.sin(rand_a) * rand_dist
			rand_loc = Vector((rand_x, rand_y)).add(trunk.loc)
			# print rand_loc
			if rand_loc[0] > 0 and rand_loc[0] < 400 and rand_loc[1] > 0 and rand_loc[1] < 400:
				# within the canvas
				break

			rand_a += math.pi/10

		new_branch = self.add_node(rand_loc, [trunk, ], time)
		self.data[trunk].append(self.add_connect(trunk, new_branch, time))

	# check validity of node paths, moves downards through connections to in_connect.end
	def validity(self, in_connect, time):
		for connection in self.data[in_connect.end]:
			# TODO: look if the connection intersects an obstacle



			# if this node isn't valid, nothing it connects to is
			# ignore the way you came from
			if not connection.valid and connection.end is not in_connect.start:
				connection.valid = False
				connection.end.valid = False
				self.validity(connection, time)

	# see https://www.desmos.com/calculator/zw6bjoeph2 for the probability outputted
	# see https://www.desmos.com/calculator/2iovtlu2fn for average nodes created each loop cycle
	# both are based on the number of existing nodes
	def update_branch_creation(self):
		self.branch_creation = 1 - math.tanh(len(self.data)/RRT.branch_weight)
		return self.branch_creation


# represents a single node in the rrt
class Node(object):
	def __init__(self, name, loc, time):
		self.name = name
		self.size = 7
		self.loc = loc
		self.time = time

		self.valid = True

	def __str__(self):
		return ("Node: [" + str(self.valid) + " " + str(self.name) + 
			": (" + str(self.loc[0]) + ", " + str(self.loc[1]) + ") ]")

# connects two nodes
class Connection(object):

	def __init__(self, start, end, time):
		self.start = start
		self.end = end
		traversal_time = (start.loc.subtract(end.loc).len() / RRT.traversal_rate 
			if (start and end) else 0)
		self.time = time + traversal_time
		self.end.time += traversal_time

		self.valid = end.name is not 1

	def __str__(self):
		return ("Connect: [" + str(self.valid) + " (" + 
			str(self.start) + " to " + str(self.end) + ") ]")

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

	root = tk.Tk()
	rrt = RRT(root)
	# print rrt
	sim = Simulator(root, obstacles, rrt)
	rrt.sim = sim

	root.mainloop()

if __name__ == "__main__":
	sys.exit(main())