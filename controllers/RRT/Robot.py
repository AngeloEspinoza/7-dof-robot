import pygame
import math

class Robot():
	def __init__(self, start, radius, position_x, position_y):
		self.start = start
		self.radius = radius
		self.position_x = position_x
		self.position_y = position_y


	def display_robot(self, position):
		pygame.draw.circle()