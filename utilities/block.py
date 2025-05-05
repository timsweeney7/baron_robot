import numpy as np


class Block:
    """
    class for recording properties of blocks identified in a frame
    """

    def __init__(
        self,
        color: str,
        location,
        knocked_over=False,
        bounding_height=None,
        angle_to_robo=None,
        bounding_origin=None,
    ):
        """
        @param color: The color of the block identified
        @param distance_from_robo: The distance the robot is from the block in meters
        @param anlge_to_robo: The angle in degrees of the block to the robot
        """
        self.color = color
        self.location = location
        self.bounding_height = bounding_height
        self.knocked_over = knocked_over
        self.angle_to_robo = angle_to_robo
        self.bounding_origin = bounding_origin
        
    def __eq__(self, other):
        if type(other) is not Block:
            return False
        if other.color != self.color:
            return False
        return self._close_together(other.location, self.location, 0.2)

    # We may need to update this for blocks that are far away from the robot
    def _close_together(self, b1, b2, threshold):
        """Check if the b1 position is close to the b2"""
        return (
            np.sqrt((b2[0] - b1[0]) ** 2 + (b2[1] - b1[1]) ** 2)
            < threshold
        )