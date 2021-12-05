import pygame
from RRTsrc import Graph
from RRTsrc import Map
from time import sleep
from random import randint

def main():
	dimensions = (600, 600)
	start = (50, 50)
	goal = (500, 530)
	obs_dim = 10
	obs_num = 50
	iteration = 0

	pygame.init()
	pygame.font.init() 

	map = Map(start, goal, dimensions, obs_dim, obs_num)
	graph = Graph(start, goal, dimensions, obs_dim, obs_num)

	obstacles = graph.make_obstacles()
	map.draw_map(obstacles)


	while (not graph.path_to_goal()):
		sleep(0.05)
		if iteration % 10 == 0:
			X, Y, Parent = graph.bias(goal)
			pygame.draw.circle(map.map, map.Yellow, (X[-1], Y[-1]), map.node_radius + 2, 0)
			pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
				map.edge_thickness)
		else:
			X, Y, Parent = graph.expand()
			pygame.draw.circle(map.map, map.Yellow, (X[-1], Y[-1]), map.node_radius + 2, 0)
			pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
				map.edge_thickness)

		if iteration % 5 == 0:
			pygame.display.update()
		iteration += 1

	coordinates = graph.convert_to_webots_coordinates(graph.get_path_coordinates())
	graph.write_coordinates_to_file(coordinates)
	print("Number of nodes to visit: ", (len(graph.get_path_coordinates()) + 1) // 2)
	map.draw_path(graph.get_path_coordinates())
	map.add_text(iteration)
	pygame.display.update()
	sleep(10)
	pygame.event.clear()
	pygame.event.wait(100)

if __name__ == '__main__':
	main()