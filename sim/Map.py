import numpy as np

class Map:


    def __init__(self, xmin,xmax,ymin,ymax, obstacles = []):
        self.xmin = xmin
        self.xmax =xmax
        self.ymin = ymin
        self.ymax=ymax
        self.obstacles = obstacles

