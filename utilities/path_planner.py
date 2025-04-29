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
    """ Start and end are tuples of (x, y) coordinates """
    return np.sqrt((end[0]-start[0])**2 + (end[0]-start[0])**2)


class PathPlanner():
    
    def __init__(self, world_map=None):
        self.MAX_WIDTH, self.MAX_HEIGHT = world_map.bounds
        self.blocks = world_map.get_blocks() if world_map else []
        self.target_block = None
        
        
    def select_goal(self):
        """ Selects the next destination for the robot to move towards """
        for block in self.blocks:
            if block.color == 'RED':
                print(f"[PLAN] Found RED block at {block.location}")
                self.target_block = block
                return True
        else:
            print("[PLAN] No RED block found")
            return False
        
    
    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # Reverse
    
    def _valid_move(self, position):
        """ Check if the move is valid within the world map """
        x, y = position
        # Return False if out of bounds
        if x < 0 or x > self.MAX_WIDTH or y < 0 or y > self.MAX_HEIGHT:
            return False
        # Check if the move is within the boundary of a block
        for block in self.blocks:
            if self._close_together(position, block.location) and block.color != self.target_block.color:
                return False
        return True  # Valid move
        
    
    def _get_neighbors(self, node: Node):
        """ Gets all of the neighboring positions and their move costs """
        grid_size = 0.25
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
        
        # Filter out neighbors that are not valid moves
        for neighbor in neighbors:
            if not self._valid_move(neighbor[0]):
                neighbors.remove(neighbor)

        return neighbors
    
    def _close_together(self, current, goal):
        """ Check if the current position is close to the goal """
        threshold = 0.2
        return np.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2) < threshold
    
    
    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, Node(start, 0, _heuristic_calc(start, goal), None))
        closed_set = set()

        counter = 0
        while open_heap:
            current = heapq.heappop(open_heap)
                        
            if self._close_together(current.position, goal):
                print("[PLAN] Found path to goal")
                return self.reconstruct_path(current)
            
            closed_set.add(current.position)

            for neighbor_pos, move_cost in self._get_neighbors(current):
                if neighbor_pos in closed_set:
                    continue

                new_cost_to_come = current.cost_to_come + move_cost
                neighbor_node = Node(
                    position=neighbor_pos,
                    cost_to_come=new_cost_to_come,
                    estimated_total_cost=new_cost_to_come + _heuristic_calc(neighbor_pos, goal),
                    parent=current
                )

                heapq.heappush(open_heap, neighbor_node)
            counter += 1
            if counter > 1_000_000:
                print("[PLAN] Too many iterations, stopping search")
                break
        return None  # No path found

    
        


if __name__ == "__main__":
    
    from utilities.RobotMotorControl import RobotMotorControl
    from utilities.imu import IMU
    from utilities.odometry import Odometer
    
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
    
    path_planner = PathPlanner(wm)
    path_planner.select_goal()

    start = (0.25, 0.25)
    path = path_planner.astar(start, path_planner.target_block.location)
    if path:
        print("Path found:")
        for i in path:
            print(i)
        print("Path length:", len(path))
    else:
        print("No path found")
    
    print()
    
    imu = IMU()
    rmc = RobotMotorControl(imu=imu, odom=Odometer)
    rmc.set_path(path)
    # for i in rmc.path:
    #     print(i)
    
    wm.draw_map()
    wm.draw_path_on_map(path)