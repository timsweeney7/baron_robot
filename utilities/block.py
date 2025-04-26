import numpy as np

class Block():
    """
    class for recording properties of blocks identified in a frame
    """
    def __init__(self, color:str, location, knocked_over=False, bounding_height=None):

        """
        @param color: The color of the block identified
        @param distance_from_robo: The distance the robot is from the block in meters
        @param anlge_to_robo: The angle in degrees of the block to the robot
        """
        self.color = color
        self.location = location
        self.bounding_height = bounding_height
        self.knocked_over = knocked_over        
