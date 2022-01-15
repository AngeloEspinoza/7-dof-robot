import random
import math
import pygame

class Map():
	def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
		self.start = start
		self.goal = goal
		self.map_dimensions = map_dimensions
		self.map_height, self.map_width = self.map_dimensions

		# Window settings
		self.MapWindowName = "RRT"
		pygame.display.set_caption(self.MapWindowName)
		self.map = pygame.display.set_mode((self.map_width, self.map_height))
		
		self.font = pygame.font.SysFont('Arial', 25)

		self.map.fill((50, 50, 50))

		self.node_radius = 2
		self.node_thickness = 0
		self.edge_thickness = 1

		self.obstacles = []
		self.obs_dim = obs_dim
		self.obs_num = obs_num

		# Colors
		self.Gray = (70, 70, 70)
		self.Blue = (0, 0, 255)
		self.Green = (0, 255, 0)
		self.Red = (255, 0, 0)
		self.White = (255, 255, 255)
		self.Black = (20, 20, 20)
		self.Yellow = (255, 255, 0)

	def add_text(self, iterations):
		self.map.blit(self.font.render("Iterations:" + "  " + str(iterations), True, self.Black), (460, 0))

	def draw_map(self, obstacles):
		pygame.draw.circle(self.map, self.Green, self.start, self.node_radius + 5, 0)
		pygame.draw.circle(self.map, self.Red, self.goal, self.node_radius + 5, 0)

		self.draw_obstacle(obstacles)

	def draw_path(self, path):
		for node in path:
			pygame.draw.circle(self.map, self.Red, node, self.node_radius + 3, 0)

	def draw_obstacle(self, obstacles):
		obstacles_list = obstacles.copy()
		while len(obstacles_list) > 0:
		 	obstacle = obstacles_list.pop(0)
		 	pygame.draw.rect(self.map, self.White, obstacle)
		 	pygame.draw.rect(self.map, self.White, obstacle)

class Graph: 
	def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
		(x, y) = start
		self.start = start
		self.goal = goal
		self.goalFlag = False
		self.map_height, self.map_width = map_dimensions
		self.x = []
		self.y = []
		self.parent = []

		# Initialize the three
		self.x.append(x)
		self.y.append(y)
		self.parent.append(0)

		# The obstacles
		self.obstacles = []
		self.obs_dim = obs_dim
		self.obs_num = obs_num

		# Path
		self.goalState = None
		self.path = []

	def make_random_rect(self):
		upperCornerX = int(random.uniform(0, self.map_width-self.obs_dim))
		upperCornerY = int(random.uniform(0, self.map_height-self.obs_dim))

		return (upperCornerX, upperCornerY)

	def convert_to_pygame_coordinates(self, coordinates):
		return (coordinates + 3) * 100

	def make_obstacles(self):
		# Map 1
		# obs = [pygame.Rect((0 - 30, 215 - 30), (400 + 50 - 0, 240 + 30 - 215)), pygame.Rect((200 - 30, 425 - 30), (400 + 50, 455 + 30 - 425))]
		# Map 2
		obs = [pygame.Rect((self.convert_to_pygame_coordinates(-1) - 40, self.convert_to_pygame_coordinates(-1) - 40), (self.convert_to_pygame_coordinates(1) + 40 - self.convert_to_pygame_coordinates(-1), self.convert_to_pygame_coordinates(1) + 40 - self.convert_to_pygame_coordinates(-1))),
			   pygame.Rect((self.convert_to_pygame_coordinates(-0.25) - 40, self.convert_to_pygame_coordinates(1) - 40), (self.convert_to_pygame_coordinates(0.25) + 40 - self.convert_to_pygame_coordinates(-0.25), self.convert_to_pygame_coordinates(2) + 40 - self.convert_to_pygame_coordinates(1))),
			   pygame.Rect((self.convert_to_pygame_coordinates(-0.25) - 40, self.convert_to_pygame_coordinates(-2) - 40), (self.convert_to_pygame_coordinates(0.25) + 40 - self.convert_to_pygame_coordinates(-0.25), self.convert_to_pygame_coordinates(0.25) + 40 - self.convert_to_pygame_coordinates(-2))),
			   pygame.Rect((self.convert_to_pygame_coordinates(-2) - 40, self.convert_to_pygame_coordinates(-0.25) - 40), (self.convert_to_pygame_coordinates(-1) + 40 - self.convert_to_pygame_coordinates(-2), self.convert_to_pygame_coordinates(0.25) + 40 - self.convert_to_pygame_coordinates(-0.25))),
			   pygame.Rect((self.convert_to_pygame_coordinates(1) - 40, self.convert_to_pygame_coordinates(-0.25) - 40), (self.convert_to_pygame_coordinates(2) + 40 - self.convert_to_pygame_coordinates(1), self.convert_to_pygame_coordinates(0.25) + 40 - self.convert_to_pygame_coordinates(-0.25))),
			   ]

		self.obstacles = obs.copy()

		return obs

	def convert_to_webots_coordinates(self, coordinates):
		webots_coordinates = []

		for i in range(1, len(coordinates), 2):
			# Inverse coordinates to write in file later
			webots_coordinates.append(round((coordinates[i][1] / 100) - 3, 2))
			webots_coordinates.append(round((coordinates[i][0] / 100) - 3, 2))

		# First coordinates at the end
		webots_coordinates.append(round((coordinates[0][1] / 100) - 3, 2))
		webots_coordinates.append(round((coordinates[0][0] / 100) - 3, 2))

		return webots_coordinates

	def write_coordinates_to_file(self, coordinates):
		with open("/home/angelo/Desktop/test.txt", "w") as output_file:
			for i in range(len(coordinates) - 3, -2, -1):
				output_file.write(str(coordinates[i]) + "\n")
			output_file.write(str(coordinates[-2]))
		output_file.close()

	def add_node(self, n, x, y):
		self.x.insert(n, x)
		self.y.append(y)

	def remove_node(self, n):
		self.x.pop(n)
		self.y.pop(n)

	def add_edge(self, parent, child):
		self.parent.insert(child, parent)

	def remove_edge(self, n):
		self.parent.pop(n)

	def number_of_nodes(self):
		return len(self.x)
 
	def distance(self, n1, n2):
		(x1, y1) = (self.x[n1], self.y[n1])
		(x2, y2) = (self.x[n2], self.y[n2])

		# Euclidean distance
		px = (float(x1) - float(x2))**2
		py = (float(y1) - float(y2))**2
 
		return (px + py)**(0.5)

	def sample_environment(self):
		x = int(random.uniform(0, self.map_width))
		y = int(random.uniform(0, self.map_height))

		return x, y

	def nearest(self, n):
		dmin = self.distance(0, n)
		nnear = 0
		for i in range(0, n):
			if self.distance(i, n) < dmin:
				dmin = self.distance(i, n)
				nnear = i
		return nnear

	def isFree(self): # Check for collisions
		n = self.number_of_nodes() - 1
		(x, y) = (self.x[n], self.y[n])
		obs = self. obstacles.copy()

		while len(obs) > 0:
			rectangle = obs.pop(0)
			if rectangle.collidepoint(x, y):
				self.remove_node(n)

				return False
		return True

	def crossObstacle(self, x1, x2, y1, y2):
		obs = self.obstacles.copy()
		while len(obs) > 0:
			rectangle = obs.pop(0)
			# Interpolation
			for i in range(0, 101):
				u = i / 100
				x = x1 * u + x2 * (1 - u)
				y = y1 * u + y2 * (1 - u)
				if rectangle.collidepoint(x, y):
					return True
		return False

	def connect(self, n1, n2):
		(x1, y1) = (self.x[n1], self.y[n1])
		(x2, y2) = (self.x[n2], self.y[n2])
		if self.crossObstacle(x1, x2, y1, y2):
			self.remove_node(n2)
			return False
		else:
			self.add_edge(n1, n2)
			return True

	def step(self, nnear, nrand, dmax = 35):
		d = self.distance(nnear, nrand)
		if d > dmax:
			u = dmax / d
			(xnear, ynear) = (self.x[nnear], self.y[nnear])
			(xrand, yrand) = (self.x[nrand], self.y[nrand])
			(px, py) = (xrand - xnear, yrand - ynear)
			theta = math.atan2(py, px)
			(x, y) = (int(xnear + dmax * math.cos(theta)),
					  int(ynear + dmax * math.sin(theta)))
			self.remove_node(nrand)
			if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
				self.add_node(nrand, self.goal[0], self.goal[1])
				self.goalState = nrand
				self.goalFlag = True
			else:
				self.add_node(nrand, x, y)

	def path_to_goal(self):
		if self.goalFlag:
			self.path = []
			self.path.append(self.goalState)
			newpos = self.parent[self.goalState]
			while newpos != 0:
				self.path.append(newpos)
				newpos = self.parent[newpos]
				self.path.append(0)

		return self.goalFlag

	def get_path_coordinates(self):
		pathCoords = []
		for node in self.path:
			x, y = (self.x[node], self.y[node])
			pathCoords.append((x, y))
		return pathCoords

	def bias(self, ngoal):
		n = self.number_of_nodes()
		self.add_node(n, ngoal[0], ngoal[1])
		nnear = self.nearest(n)
		self.step(nnear, n)
		self.connect(nnear, n)

		return self.x, self.y, self.parent

	def expand(self):
		n = self.number_of_nodes()
		x, y = self.sample_environment()
		self.add_node(n, x, y)
		if self.isFree():
			xnearest = self.nearest(n)
			self.step(xnearest, n)
			self.connect(xnearest, n)

		return self.x, self.y, self.parent

	def cost():
		pass