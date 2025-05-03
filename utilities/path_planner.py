import numpy as np
import heapq
from typing import List
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from utilities.block import Block
from utilities.imu import IMU

MAP_EDGE_KEEPOUT = 0.3048/2  # 1 FOOT
BLOCK_KEEPOUT = 0.2

ORDER = ["RED", "GREEN", "BLUE"] * 3


class WorldMap:

    def __init__(self, length, width, imu: IMU):
        self.blocks = []  # list of x,y locations of objects in the world
        self.bounds = (
            length,
            width,
        )  # list of x,y locations for the bounds of the world
        self.robot_position = (0.4826, 0.25)
        self.imu = imu
        self.axis = None
        self.figure = None
        self.construction_area_dim = (1.2192, 1.2192)  # 4ft x 4ft
        
        x = (self.construction_area_dim[0] - MAP_EDGE_KEEPOUT) / 2 + MAP_EDGE_KEEPOUT
        y = -1 * x + self.bounds[1]
        self.construction_area_center = (x, y)
        
        self.previous_paths = []

    def get_robot_position(self):
        """Returns the position and orientation of the robot"""

        # For DEBUG ONLY
        if self.imu is None:
            print("[PLAN] WORLD MAP IS IN DEBUG MODE !!!")
            return self.robot_position, 0

        return self.robot_position, self.imu.get_heading()

    def update_blocks(self, blocks: List[Block]):
        """
        Updates the location of blocks in the world map based on the metadata provided
        @param blocks: List of block positions from the robot frame of reference
        """
        for block in blocks:
            self.blocks.append(block)

    def get_blocks(self):
        return self.blocks

    def draw_map(self):
                
        fig, ax = plt.subplots()  # << New figure and axes each time
        self.axis = ax
        self.figure = fig
        ax.set_aspect("equal")

        # Draw the bounds of the map
        width, height = self.bounds
        arena = patches.Rectangle(
            (0, 0), width=width, height=height, facecolor="none", edgecolor="black"
        )
        ax.add_patch(arena)
        keepout = patches.Rectangle(
            (MAP_EDGE_KEEPOUT, MAP_EDGE_KEEPOUT), width=width-MAP_EDGE_KEEPOUT*2, height=height-MAP_EDGE_KEEPOUT*2, facecolor="none", edgecolor="red"
        )
        ax.add_patch(keepout)
        
        # Draw the construction area
        construction_area = patches.Rectangle(
            (0, height),
            width=self.construction_area_dim[0],
            height=-1*self.construction_area_dim[1],
            facecolor="none",
            edgecolor="black"
        )
        ax.add_patch(construction_area)
        ax.scatter(*self.construction_area_center, c="black")

        # Draw the robot
        location, heading_deg = self.get_robot_position()
        robo_x, robo_y = location
        triangle = np.array([[0.0762, 0, 1], [-0.254, 0.1016, 1], [-0.254, -0.1016, 1]])
        theta = np.deg2rad(heading_deg)
        c, s = np.cos(theta), np.sin(theta)
        T = np.array([[c, -s, robo_x], [s, c, robo_y], [0, 0, 1]])
        triangle = (T @ triangle.T).T[:, :2]
        robot_patch = patches.Polygon(triangle, closed=True, color="black")
        ax.add_patch(robot_patch)

        # Draw the blocks
        for block in self.blocks:
            x, y = block.location
            ax.scatter(x, y, c=block.color)
            circle = plt.Circle(
                (x, y), BLOCK_KEEPOUT, color=block.color, fill=False, clip_on=True
            )
            ax.add_patch(circle)
            
        ax.set_title("World Map")
        ax.set_xlim(-1 * width * 0.02, width * 1.02)
        ax.set_ylim(-1 * height * 0.02, height * 1.02)
        ax.set_xlabel("Meters")

        ax.set_axisbelow(True)
        ax.grid(True, which="both", linestyle="--", linewidth=0.5)
        
        
        # Draw the previously driven paths
        for path in self.previous_paths:
            self.draw_path_on_map(path, "blue", save_fig=False)

        
        plt.savefig("world_map.jpg")
        # plt.show()
        # plt.close(fig)  # << clean up the figure from memory if you're doing many plots
        

    def draw_path_on_map(self, path: List[tuple], color:str, save_fig=False):
        """Draws the path on the map"""

        if self.axis is None or self.figure is None:
            print("[PLAN] No map to draw on")
            return
        
        if color.lower() == "blue":
            self.robot_position = path[-1]
            print(f"[PLAN] Robot position updated to {self.robot_position}")

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            self.axis.plot([x1, x2], [y1, y2], color=color, linewidth=2)

        if save_fig:
            plt.savefig("world_map.jpg")

            
    def remove_block(self, block: Block):
        """Removes a block from the world map"""
        self.blocks.remove(block)
        self.draw_map()
        print(f"[PLAN] Removed {block.color} block at {block.location}")
        input("[PLAN] 101")
        return True


class Node:
    def __init__(self, position, cost_to_come=None, estimated_total_cost=None, heading = None, parent=None):
        self.position = position
        self.cost_to_come = cost_to_come
        self.estimated_total_cost = estimated_total_cost
        self.heading = heading
        self.parent = parent

    def __lt__(self, other):
        return self.estimated_total_cost < other.estimated_total_cost
    
    def __hash__(self):
        return hash((self.position, self.heading))
    
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.position == other.position and self.heading == other.heading
        return False

def angle_between_points(p1, p2):
    """
    Returns the angle between two points in degrees
    from [-180 to 180]
        """
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    return np.rad2deg(np.arctan2(dy, dx))


def _calculate_cost_to_go(start_heading, start, end):
    """Start and end are tuples of (x, y) coordinates"""
    straight_line_cost = np.sqrt((end[0] - start[0]) ** 2 + (end[0] - start[0]) ** 2)
    angle = (angle_between_points(start, end) + 360) % 360 # convert to [0, 360]
    angle_diff = abs(angle - start_heading)
    if angle_diff > 45:
        cost_to_turn = 1.1
    else:
        cost_to_turn = 1.0
    return straight_line_cost + cost_to_turn


class PathPlanner:

    def __init__(self, world_map: WorldMap = None):
        self.MAX_WIDTH, self.MAX_HEIGHT = world_map.bounds
        self.blocks = world_map.get_blocks() if world_map else []
        self.target_block = None
        self.GRID_SIZE = 0.1
        self.wm = world_map

    def select_goal(self):
        """Selects the next destination for the robot to move towards"""
        for block in self.blocks:
            if block.color == "RED":
                print(f"[PLAN] Found RED block at {block.location}")
                self.target_block = block
                return self.target_block
        else:
            print("[PLAN] No RED block found")
            return False

    def reconstruct_path(self, node: Node):
        """
        Returns the path from Node to planning start point in x,y steps
        """
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # Reverse

    def _valid_move(self, position):
        """Check if the move is valid within the world map"""
        x, y = position
        # Return False if out of bounds
        if (
            x < 0 + MAP_EDGE_KEEPOUT
            or x > self.MAX_WIDTH - MAP_EDGE_KEEPOUT
            or y < 0 + MAP_EDGE_KEEPOUT
            or y > self.MAX_HEIGHT - MAP_EDGE_KEEPOUT
        ):
            return False
        # Check if the move is within the boundary of a block
        for block in self.blocks:
            if (
                self._close_together(position, block.location, threshold=BLOCK_KEEPOUT)
                and block.color != self.target_block.color
            ):
                return False
        return True  # Valid move

    def _get_neighbors(self, node: Node, goal):
        """
        Gets all of the neighboring positions and their move costs
        Returns a list of Node objects
        """
        
        def _calculate_cost_to_come(child:tuple):
            """Calculate the cost of moving to a neighbor"""
            current_heading = node.heading
            child_heading = (angle_between_points(node.position, child) + 360) % 360 # convert to [0, 360]
            # Calculate the cost based on the angle difference
            angle_diff = abs(current_heading - child_heading)
            if angle_diff > 1:
                turn_cost = 1
            else:
                turn_cost = 0
            # Calculate the distance to the child node
            distance = np.sqrt((child[0] - node.position[0]) ** 2 + (child[1] - node.position[1]) ** 2)
            return turn_cost + distance
        
        x, y = node.position
       
        # fmt: off
        neighbor_positions = [
            (round(x + self.GRID_SIZE, 5), round(y                 , 5)),  # straight
            (round(x                 , 5), round(y + self.GRID_SIZE, 5)),
            (round(x - self.GRID_SIZE, 5), round(y                 , 5)),
            (round(x                 , 5), round(y - self.GRID_SIZE, 5)),
            (round(x + self.GRID_SIZE, 5), round(y + self.GRID_SIZE, 5)),  # diagonal
            (round(x - self.GRID_SIZE, 5), round(y + self.GRID_SIZE, 5)),
            (round(x - self.GRID_SIZE, 5), round(y - self.GRID_SIZE, 5)),
            (round(x + self.GRID_SIZE, 5), round(y - self.GRID_SIZE, 5)),
        ]
        # fmt: on

        # Filter out neighbors that are not valid moves
        valid_positions = [
            pos for pos in neighbor_positions
            if self._valid_move(pos)
        ]
        
        # Convert positions and costs to Node objects
        valid_neighbor_nodes = []
        for pos in valid_positions:
            cost_to_come = node.cost_to_come + _calculate_cost_to_come(pos)
            estimated_total_cost = cost_to_come + _calculate_cost_to_go(node.heading, pos, goal)
            neighbor_node = Node(
                position=pos,
                cost_to_come=cost_to_come,
                estimated_total_cost=estimated_total_cost,
                heading = (angle_between_points(node.position, pos) + 360) % 360, # convert to [0, 360]
                parent=node
            )
            valid_neighbor_nodes.append(neighbor_node)

        return valid_neighbor_nodes

    def _close_together(self, current, goal, threshold):
        """Check if the current position is close to the goal"""
        return (
            np.sqrt((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2)
            < threshold
        )

    def astar(self, start, goal, debug=0):
        # setup
        open_heap = []
        start_heading = self.wm.get_robot_position()[1]
        heapq.heappush(
            open_heap, 
            Node(
                position=start, 
                cost_to_come=0, 
                estimated_total_cost=_calculate_cost_to_go(start_heading, start, goal), 
                heading=start_heading, 
                parent=None
            )
        )
        closed_set = {}
        open_set = {}
        
        # run
        counter = 0
        while open_heap:
            current = heapq.heappop(open_heap)

            if self._close_together(
                current.position, goal, threshold=BLOCK_KEEPOUT
            ):
                print("[PLAN] Found path to goal")
                break
                

            closed_set[Node(current.position)] = current

            if debug > 0 and (current.parent is not None):
                closed_path = [current.parent.position, current.position]
                self.wm.draw_path_on_map(closed_path, "pink", False)
                if debug > 1:
                    self.wm.draw_path_on_map(closed_path, "pink", True)
                    input("...")  # give time to review map

            for neighbor in self._get_neighbors(current, goal):
                
                if neighbor in closed_set:
                    continue
                if neighbor in open_set:
                    # update the node if the new node has a lower cost to come
                    old_cost_to_come = open_set[neighbor].cost_to_come
                    if neighbor.cost_to_come < old_cost_to_come:
                        open_set[neighbor] = neighbor
                        # push the new node to the heap
                        heapq.heappush(open_heap, neighbor)
                    continue
                else:
                    # Add the node to the open_set
                    open_set[neighbor] = neighbor
                    heapq.heappush(open_heap, neighbor)
                    if debug > 0 :
                        closed_path = [neighbor.parent.position, neighbor.position]
                        self.wm.draw_path_on_map(closed_path, "gray", False)
                        if debug > 1:
                            self.wm.draw_path_on_map(closed_path, "gray", True)
                            input("...")

            counter += 1
            if counter > 1_000_000:
                print("[PLAN] Too many iterations, stopping search")
                return None  # No path found

        # if the heap is empty then no path was found
        if len(open_heap) == 0:
            print("[PLAN] No path found")
            return None
        
        # orient robot towards the goal
        angle_to_goal = angle_between_points(current.position, goal)
        # print(f"Angle to goal: {angle_to_goal}")
        last_position = (
            current.position[0]+np.cos(np.deg2rad(angle_to_goal))*0.01 , 
            current.position[1]+np.sin(np.deg2rad(angle_to_goal))*0.01
        )
        last_node = Node(
                position=last_position, 
                cost_to_come=None, 
                estimated_total_cost=None, 
                heading=None,
                parent=current
            )
        return self.reconstruct_path(last_node)



if __name__ == "__main__":

    import time
    from utilities.RobotMotorControl import RobotMotorControl
    from utilities.imu import IMU
    from utilities.odometry import Odometer

    block1 = Block(color="GREEN", location=(1.65, 0.495))

    block2 = Block(color="CYAN", location=(1.790680797371233, 0.02305584746844269))

    block3 = Block(color="RED", location=(2.0, 1.25))

    # MAP_LENGTH = 3.048
    # MAP_WIDTH = 3.048
    MAP_LENGTH = 2.54
    MAP_WIDTH = 1.829   
    
    wm = WorldMap(MAP_LENGTH, MAP_WIDTH, None)
    wm.update_blocks([block1, block2, block3])
    wm.draw_map()

    path_planner = PathPlanner(wm)
    
    # get the first block 
    target_block = path_planner.select_goal()
    start = (0.25, 0.25)
    path = path_planner.astar(start, path_planner.target_block.location)
    wm.draw_path_on_map(path, "red", save_fig=True)
    wm.previous_paths.append(path)
    
    # remove the block from the map
    wm.remove_block(target_block)
    
    # drop it off at the goal location
    start = target_block.location
    goal = wm.construction_area_center
    path = path_planner.astar(start, goal)
    wm.draw_path_on_map(path, "red", save_fig=True)
    wm.previous_paths.append(path)
    input("'3'")
    
    

    # imu = IMU()
    # odom = Odometer()
    # time.sleep(0.25)
    # rmc = RobotMotorControl(imu=imu, odom=odom)
    # rmc.set_path(path)
    # for i in rmc.path:
    #     print(i)

    
    print("game over")
    # rmc.game_over()

    
