import random

from linalgebra import *

# stores variables to tune all random numbers
class Tune(object):
	# length of randomly created branches
	branch_len_min = 20.0
	branch_len_max = 180.0

	# probability of creating a new branch off of an existing one, checked each loop cycle
	branch_creation = .3

class RRT(object):
	def __init__(self):
		self.size = 7
		self.speed = 20
		self.loc = Vector((200, 375))

		self.rrt_index = 0

		self.rrt = {}
		self.name_to_node = {} # converts a name to a node

	# creates a node which is at the given x, y; connected to connections; and has a name
	def add_node(self, loc, connections_names):
		new_node = Node(self.rrt_index, loc)

		# loop over names of connections and create a new one

		connections = []
		for connection_name in connections_names:
			connections.append(Connection(self.rrt_index, connection_name))
		self.rrt[new_node] = connections

		self.name_to_node[self.rrt_index] = new_node

		self.rrt_index += 1

		return new_node

	# creates a branch in a random direction with given name off of given trunk
	def add_branch(self, trunk_name):
		rand_a = random.random() * 2.0*math.pi # random number between [0, 2 pi)
		rand_dist = random.random() * Tune.branch_len_max + Tune.branch_len_min # random distance

		rand_x = math.cos(rand_a) * rand_dist
		rand_y = math.sin(rand_a) * rand_dist

		rand_loc = Vector((rand_x, rand_y))

		trunk = self.name_to_node[trunk_name]

		new_branch = self.add_node(rand_loc.add(trunk.loc), [trunk_name, ])
		self.rrt[trunk].append(Connection(trunk_name, new_branch.name))

# represents a single node in the rrt
class Node(object):
	def __init__(self, name, loc):
		self.name = name
		self.size = 7
		self.loc = loc

		self.valid = True

	def __str__(self):
		return self.name + ": (" + str(self.loc[0]) + ", " + str(self.loc[1]) + ")"

# connects two nodes
class Connection(object):
	def __init__(self, start, end):
		self.start = start
		self.end = end

		self.valid = True