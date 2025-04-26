import numpy as np

class Block():
    """
    class for recording properties of blocks identified in a frame
    """
    def __init__(self, color:str, distance_from_robo:float, angle_to_robo:float, knocked_over=False, bounding_height=None):
        """
        @param color: The color of the block identified
        @param distance_from_robo: The distance the robot is from the block in meters
        @param anlge_to_robo: The angle in degrees of the block to the robot
        """
        self.color = color
        self.distance_from_robo = distance_from_robo
        self.angle_to_robo = angle_to_robo  # degrees
        self.bounding_height = bounding_height
        self.knocked_over = knocked_over
        
        x = self.distance_from_robo * np.cos(np.deg2rad(self.angle_to_robo))
        y = self.distance_from_robo * np.sin(np.deg2rad(self.angle_to_robo))
        self.location = (x, y)