import Tkinter as tk

import random
import sys

from linalgebra import *
from simulator import *

# stores variables to tune all random numbers
class Tune(object):
	# length of randomly created branches
	branch_len_min = 20.0
	branch_len_max = 180.0

	# probability of creating a new branch off of an existing one, checked each loop cycle
	branch_creation = .3

	# update in seconds
	time_step = .5

class RRT(object):
	def __init__(self, root):
		self.size = 7
		self.speed = 20
		self.base = Vector((200, 375))

		self.sim = None
		self.root = root

		self.rrt_index = 0

		self.data = {}
		self.connects = {}
		self.name_to_node = {} # converts a name to a node

	def __str__(self):
		str_list = []
		str_list.append("RRT: [")

		for key, values in self.data.items():
			str_list.append(str(key) + " ")
			for value in values:
				str_list.append(str(value) + " ")

		str_list.append("]")
		return ''.join(str_list)

	def update(self, t):

		# print self

		if self.sim:
			self.validity(self.add_connect(None, self.name_to_node[0]))

			self.add_branches()
			self.sim.display_sim(t)

		# wait one time_step, then redraw
		self.root.after(int(Tune.time_step * 1000), self.update, t + Tune.time_step)

	# creates a node which is at the given x, y; connected to connections; and has a name
	def add_node(self, loc, connection_nodes):
		new_node = Node(self.rrt_index, loc)

		# loop over names of connections and create a new one

		connections = []
		for connection_node in connection_nodes:
			connections.append(self.add_connect(new_node, connection_node))
		self.data[new_node] = connections

		self.name_to_node[self.rrt_index] = new_node

		self.rrt_index += 1

		return new_node

	def add_connect(self, start, end):
		new_connect = Connection(start, end)
		
		new_connect_name = self.connect_name(start, end)
		self.connects[new_connect_name] = new_connect
		return new_connect

	def node_name(self, node):
		return str(node.name) if node else "None"

	def connect_name(self, start, end):
		return self.node_name(start) + ":" + self.node_name(end)

	# creates a series of random branches off of each existing node
	def add_branches(self):
		for key in self.data.keys():
			add_branch = random.random() <= Tune.branch_creation
			if add_branch and key.valid:
				self.add_branch(key.name)

	# creates a branch in a random direction with given name off of given trunk
	def add_branch(self, trunk_name):
		rand_a = random.random() * 2.0*math.pi # random number between [0, 2 pi)
		rand_dist = random.random() * Tune.branch_len_max + Tune.branch_len_min # random distance

		rand_x = math.cos(rand_a) * rand_dist
		rand_y = math.sin(rand_a) * rand_dist

		rand_loc = Vector((rand_x, rand_y))

		trunk = self.name_to_node[trunk_name]

		new_branch = self.add_node(rand_loc.add(trunk.loc), [trunk, ])
		self.data[trunk].append(self.add_connect(trunk, new_branch))

	# check validity of node paths, moves downards through connections to in_connect.end
	def validity(self, in_connect):
		for connection in self.data[in_connect.end]:
			# if this node isn't valid, nothing it connects to is
			# ignore the way you came from
			if not connection.valid and connection.end is not in_connect.start:
				connection.valid = False
				print "invalid"
				connection.end.valid = False
				self.validity(connection)
			# look if the connection intersects an obstacle


# represents a single node in the rrt
class Node(object):
	def __init__(self, name, loc):
		self.name = name
		self.size = 7
		self.loc = loc

		self.valid = True

	def __str__(self):
		return ("Node: [" + str(self.valid) + " " + str(self.name) + 
			": (" + str(self.loc[0]) + ", " + str(self.loc[1]) + ") ]")

# connects two nodes
class Connection(object):

	def __init__(self, start, end):
		self.start = start
		self.end = end

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
	new_node = rrt.add_node(rrt.base, [])
	rrt.add_branch(0)
	# print rrt
	sim = Simulator(root, obstacles, rrt)
	rrt.sim = sim

	root.mainloop()

if __name__ == "__main__":
	sys.exit(main())