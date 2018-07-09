import Tkinter as tk
import sys
import math

class Shape(object):
	# Takes an tuple of vectors defining the corners of the shape relative to the center, 
	# clockwise from UL; where the center starts; and a velocity that the shape is moving at
	def __init__(self, points, t0, velocity):

		points3 = [] # points, but vectors of length 3

		# append a zero for rotation if needed
		for point in points:
			if len(point) is 2:
				points3.append(Vector((point[0], point[1], 0)))
			else:
				points3.append(point)

		self.points = tuple(points3)
		self.velocity = velocity
		self.t0 = t0 # position at t = 0

	def __str__(self):
		str_list = []
		for point in self.points:
			str_list.append(str(point))
		return ''.join(str_list)
	
	# returns the location of the shape at the given time t
	# as r(t) = integral(vdt, 0, t) = vt + initial position
	def location(self, t):
		return Vector((
			self.velocity[0] * t + self.t0[0], 
			self.velocity[1] * t + self.t0[1],
			self.velocity[2] * t + self.t0[2]
			))

	# returns the shape's points, relative to the canvas NOT the center at time = t
	# this is the very important function that returns coordinates for drawing the obstacle!
	def absolute_pos(self, t):
		abs_points = []

		rotated = self.rotate(self.velocity[2]*t + self.t0[2])

		curr_loc = rotated.location(t)

		for point in rotated.points:
			# adding combines the components of the center with those of each point
			abs_points.append(curr_loc.add(point))

		return Shape(abs_points, rotated.t0, rotated.velocity)

	# returns the shape, rotated by a (measured in radians)
	def rotate(self, a):
		rot_matrix = Matrix((
			Vector((math.cos(a), math.sin(a), 0)),
			Vector((-math.sin(a), math.cos(a), 0)),
			Vector((0, 0, 1))
			))

		new_points = []

		for point in self.points:
			new_points.append(rot_matrix.mult(point))

		# new_t0 = (self.t0[0], self.t0[1], self.t0[2] + a)
		return Shape(tuple(new_points), self.t0, self.velocity)

	# finds the centroid (center of mass) of the shape at time t
	# see: https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
	def centroid(self, t):
		# appends the first point to the end of the list, required for this
		looped = list(self.points)
		looped.append(self.points[0])

		area = self.area()
		c_x = 0
		c_y = 0
		for i in range(0, len(looped) - 1):
			c_x += ((looped[i][0] + looped[i+1][0]) * 
				(looped[i][0]*looped[i+1][1] - looped[i+1][0]*looped[i][1]))
			c_y += ((looped[i][1] + looped[i+1][1]) * 
				(looped[i][0]*looped[i+1][1] - looped[i+1][0]*looped[i][1]))

		c_x *= 1.0/(6.0*area)
		c_y *= 1.0/(6.0*area)
		return Vector((c_x + self.velocity[0] * t, c_y + self.velocity[1] * t))

	# Finds the area of the shape
	# This differs from the signed area, as area below the x axis is still positive here
	# see: https://en.wikipedia.org/wiki/Shoelace_formula
	def area(self):
		area = 0
		j = len(self.points) - 1
		for i in range(0, len(self.points)):
			area += (self.points[j][0] + self.points[i][0]) * (self.points[j][1] - self.points[i][1])
			j = i

		return abs(area / 2)

class Matrix(object):

	# values should be a tuple of tuples (list of columns (which are vectors))
	def __init__(self, values):
		self.values = values

	def __str__(self):
		str_list = []
		for col in self.values:
			str_list.append(str(col))
		return ''.join(str_list)

	# multiplies the matrix by the vector
	def mult(self, vector):
		result = Vector((0,) * len(vector)) # tuple of the length of the vector

		for i in range(0, len(vector)):
			# loop over the incoming vector's coordinates and the matrix's vectors
			col = self.values[i]
			vector_val = vector[i]
			result_col = col.scalar(vector_val)
			result = result.add(result_col)

		return result

# vector[0] = x; vector[1] = y; vector[2] = theta
class Vector(object):

	def __init__(self, coords):
		self.coords = coords

	def __str__(self):
		return str(self.coords)

	# gets the coordinate at the indicated index, called as vector[i]
	def __getitem__(self, index):
		return self.coords[index]

	# sets the coordinate at the indicated index, called as vector[i] = val
	def __setitem__(self, index, val):
		self.coords[index] = val
		return self

	# gives the number of coordinates in the vector, called as len(vector)
	def __len__(self):
		return len(self.coords)

	# gives the Euclidean length of vector, called as vector.len()
	def len(self):
		total = 0
		for coord in self.coords:
			total += coord**2

		return math.sqrt(total)

	# returns this vector multiplied by a scalar
	def scalar(self, scalar):
		new_vector = []

		for coord in self.coords:
			new_vector.append(coord * scalar)

		return Vector(tuple(new_vector))

	# adds this vector to other
	def add(self, other):
		new_vector = []

		for i in range(0, len(self)):
			new_vector.append(self[i] + other[i])

		return Vector(tuple(new_vector))

class SimRobot(object):
	def __init__(self):
		self.size = 7
		self.x = 200
		self.y = 375

		self.rrt = {}
		self.name_to_node = {} # converts a name to a node

	def add_node(self, name, x, y, connections):
		new_node = Node(name, x, y)
		self.rrt[new_node] = connections
		self.name_to_node[name] = new_node
		
# represents a single node in the rrt
class Node(object):
	def __init__(self, name, x, y):
		self.name = name
		self.size = 7
		self.x = x
		self.y = y

	def __str__(self):
		return self.name + ": (" + str(self.x) + ", " + str(self.y) + ")"

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
		self.draw_obstacles(t)
		self.draw_robot()
		self.draw_rrt()

		# wait one time_step, then redraw
		self.root.after(int(self.time_step * 1000), self.display_sim, t + self.time_step, False)

	# draws a dot for the robot
	def draw_robot(self):
		self.canvas.create_oval(self.draw_dot((
			self.robot.x, 
			self.robot.y),
			self.robot.size), 
			fill='green')

	# draws the rrt connected to the robot
	def draw_rrt(self):
		for key, value in self.robot.rrt.items():
			if not key in self.rrt_node_pointers:
				self.rrt_node_pointers[key] = self.canvas.create_oval(self.draw_dot((
					key.x,
					key.y),
					key.size),
					fill='dark green')

			# draw the connections too
			for connection in value:
				if not key in self.rrt_connection_pointers:
					# get the actual node, not the name of it
					other_node = self.robot.name_to_node[connection]

					self.rrt_connection_pointers[key] = self.canvas.create_line(
						key.x,
						key.y,
						other_node.x,
						other_node.y,
						fill='dark green',
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

	print ob1.centroid(0)

	# print ob3.rotate(math.pi/2)

	# matrix_test = Matrix((Vector((2, 0)), Vector((0, 1))))
	# print matrix_test.mult(Vector((2, 5)))

	root = tk.Tk()
	robot = SimRobot()
	robot.add_node("A", 200, 350, ("B"))
	robot.add_node("B", 225, 325, ("A"))
	sim = Simulator(root, obstacles, robot)

	root.mainloop()

if __name__ == "__main__":
	sys.exit(main())