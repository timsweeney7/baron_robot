import numpy as np
from queue import Queue
import heapq

from utilities.odometry import WorldMap
from utilities.block import Block


ORDER = ["RED", "GREEN", "BLUE"] * 3


class Node:
    def __init__(self, position, cost_to_come, estimated_total_cost, parent=None):
        self.position = position
        self.cost_to_come = cost_to_come
        self.estimated_total_cost = estimated_total_cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.estimated_total_cost < other.estimated_total_cost
    
    
def _heuristic_calc(start, end):
    return np.sqrt((end[0]-start[0])^2 + (end[0]-start[0])^2)


class PathPlanner():
        
    def __init__(self, world_map:WorldMap):
        "create an instance of the planner"
        self.wm = world_map
        
        
    def select_goal(self, blocks):
        "Given a list of blocks, select the correct block"
        # find the next block that is closest to use
        return
    
    def reconstruct_path(node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # Reverse
    
    def _get_neighbors(self, node:Node, goal):
        """ Gets all of the neighboring nodes of a given node """
        grid_size = 0.1
        x, y  = node.position
        cost = node.cost_to_come
        
        # straight
        next_loc = (x+grid_size, y)
        n1 = Node(next_loc, cost+1, _heuristic_calc(next_loc, goal), node)
        next_loc = (x, y+grid_size)
        n2 = Node(next_loc, cost+1, _heuristic_calc(next_loc, goal), node)
        next_loc = (x-grid_size, y)
        n3 = Node(next_loc, cost+1, _heuristic_calc(next_loc, goal), node)
        next_loc = (x, y-grid_size)
        n4 = Node(next_loc, cost+1, _heuristic_calc(next_loc, goal), node)
        
        # angle
        next_loc = (x+grid_size, y+grid_size)
        n5 = Node(next_loc, cost+np.sqrt(2), _heuristic_calc(next_loc, goal), node)
        next_loc = (x-grid_size, y+grid_size)
        n6 = Node(next_loc, cost+np.sqrt(2), _heuristic_calc(next_loc, goal), node)
        next_loc = (x-grid_size, y-grid_size)
        n7 = Node(next_loc, cost+np.sqrt(2), _heuristic_calc(next_loc, goal), node)
        next_loc = (x+grid_size, y-grid_size)
        n8 = Node(next_loc, cost+np.sqrt(2), _heuristic_calc(next_loc, goal), node)
        
        return [n1, n2, n3, n4, n5, n6, n7, n8]
        
    
    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, Node(start, 0, _heuristic_calc(start), None))
        closed_set = set()

        while open_heap:
            current = heapq.heappop(open_heap)
            
            if current.position == goal:
                return self.reconstruct_path(current)
            
            closed_set.add(current.position)

            for neighbor_pos, move_cost in neighbors_fn(current.position):
                if neighbor_pos in closed_set:
                    continue

                new_cost_to_come = current.cost_to_come + move_cost
                neighbor_node = Node(
                    neighbor_pos,
                    new_cost_to_come,
                    new_cost_to_come + _heuristic_calc(neighbor_pos),
                    current
                )

                heapq.heappush(open_heap, neighbor_node)

        return None  # No path found

    
        


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
    
    
    wm.update_blocks([block1, block2, block3])
    blocks = wm.get_blocks()
    
    path_planner = PathPlanner(world_map=wm)
    
    print(path_planner.ORDER)