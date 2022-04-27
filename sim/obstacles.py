import numpy as np
"""
File to define obstacle objects to be used for path planning
"""
class Circular_Obstacle:
    """
    Generic  Obstacle
    """
    def __init__(self, radius, x,y):
        """
        Initialize an obstacle object.
        Inputs:
        center: (2, ) numpy array of xy center coordinates
        radius: radius of obstacle
        """
        self.radius = radius
        self.x = x
        self.y = y
        
    
    # def collision_check(self, points):
    #     """
    #     Function to check if set of points contains collisions with an obstacle
    #     Inputs:
    #     points: (2, N) list of N points to check collisions for 

    #     Returns:
    #     True if a point is inside the circle, False if no collisions
    #     """
    #     for i in range(points.shape[1]):
    #         point = points[:, i]
    #         x, y = point #unpack point into two coordinates
    #         x_c, y_c = self.center #unpack center coordinates
    #         if (x - x_c)**2 + (y-y_c)**2 <= self.radius**2:
    #             return True
    #     return False



