#!/usr/bin/env python

import numpy as np 
import matplotlib.pyplot as plt
import scipy.misc
from PIL import Image

"""
Adapted From:
Starter code for EECS C106B Spring 2020 Project 2.
Original Author: Tiffany Cappellari

"""
class Obstacle():
	def __init__(self, center_x, center_y, radius):
		self.center = (center_x, center_y)
		self.radius = radius


def create_grid(obstacle_list, width, height, resolution=0.01):
	"""Creates a grid representing the map with obstacles
	in obstacle_list.

	Args:
		obstacle_list: 'list' of Obstacle objects. Assumes center
		and radius are given in meters.
		width: width of the environment (expanse in x direction) in meters.
		height: height of the environment (expanse in y direction) in meters.
		resolution: meters per pixel.
	"""
	def m_to_pix(*nums):
		return [int(x / resolution) for x in nums]

	m = np.ones(m_to_pix(height, width))
	for obstacle in obstacle_list:
		x_c, y_c = m_to_pix(*obstacle.center)
		r, = m_to_pix(obstacle.radius)
		for x in range(x_c - r, x_c + r):
			for y in range(y_c - r, y_c + r):
				if ((x - x_c)**2 + (y - y_c)**2) <= r**2:
					m[y][x] = 0
	m = m[::-1]
	return m

def create_png(m, name):
	# scipy.misc.imsave(name + '.png', m)
	im = Image.fromarray((m * 255).astype(np.uint8))
	im.save(name + '.png')