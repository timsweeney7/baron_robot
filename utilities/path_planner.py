import numpy as np
from queue import Queue

from utilities.odometry import WorldMap
from utilities.block import Block


class PathPlanner():
    
    ORDER = ["RED", "GREEN", "BLUE"] * 3
    
    def __init__(self, world_map:WorldMap):
        "create an instance of the planner"
        self.wm = world_map
        
        
    def select_goal(self, blocks):
        "Given a list of blocks, select the correct block"
        
# class AStarSearch():


if __name__ == "__main__":
    
    block1 = Block(color="GREEN", 
                   location=(1.65, 0.495))
                   
    block2 = Block(color="CYAN", 
                   location=(0.9, 0.235))
    
    block3 = Block(color="RED", 
                   location=(1.77, 0.28))
    
    MAP_LENGTH = 2.54
    MAP_WIDTH = 1.829
    wm = WorldMap(MAP_LENGTH, MAP_WIDTH, None)
        
    # example camera data
    metadata = {
        "position": wm.get_robot_position()[0],
        "heading_deg": 0
    }
    
    
    wm.update_blocks([block1, block2, block3])
    
    wm.draw_map()