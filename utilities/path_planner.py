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
        
    
            
    def select_goal(self, blocks):
        "Given a list of blocks, select the correct block"
        # find the next block that is closest to use
        for block in blocks:
            if block.color == 'RED':
                goal = block.location
                return
        else:
            print("No RED block found")
            return None
        
    
    def reconstruct_path(node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # Reverse
    
    def _get_neighbors(self, node: Node):
        """ Gets all of the neighboring positions and their move costs """
        grid_size = 0.1
        x, y = node.position

        neighbors = [
            ((x + grid_size, y), 1),  # straight
            ((x, y + grid_size), 1),
            ((x - grid_size, y), 1),
            ((x, y - grid_size), 1),
            ((x + grid_size, y + grid_size), np.sqrt(2)),  # diagonal
            ((x - grid_size, y + grid_size), np.sqrt(2)),
            ((x - grid_size, y - grid_size), np.sqrt(2)),
            ((x + grid_size, y - grid_size), np.sqrt(2)),
        ]

        return neighbors
        
    
    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, Node(start, 0, _heuristic_calc(start), None))
        closed_set = set()

        while open_heap:
            current = heapq.heappop(open_heap)
            
            if current.position == goal:
                return self.reconstruct_path(current)
            
            closed_set.add(current.position)

            for neighbor_pos, move_cost in self._get_neighbors(current.position):
                if neighbor_pos in closed_set:
                    continue

                new_cost_to_come = current.cost_to_come + move_cost
                neighbor_node = Node(
                    position=neighbor_pos,
                    cost_to_come=new_cost_to_come,
                    estimated_total_cost=new_cost_to_come + _heuristic_calc(neighbor_pos),
                    parent=current
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
    
    path_planner = PathPlanner()
    goal = path_planner.select_goal(blocks)
    start = (0.25, 0.25)
    path = path_planner.astar(start, goal)
    if path:
        print("Path found:", path)
    else:
        print("No path found")
    
    print(path_planner.ORDER)