import numpy as np
import heapq
from typing import List
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as ticker
import copy

from utilities.block import Block
from utilities.imu import IMU

MAP_EDGE_KEEPOUT = 0.3048/2  # 1 FOOT
BLOCK_KEEPOUT = 0.3

ORDER = ["RED", "GREEN", "BLUE"] * 3


class WorldMap:

    def __init__(self, length, width, imu: IMU, _debug_heading = None):
        self.blocks = []  # list of x,y locations of objects in the world
        self.bounds = (
            length,
            width,
        )  # list of x,y locations for the bounds of the world
        self.robot_position = (0.3048, 0.3048) # 1 ft, 1 ft
        self.imu = imu
        self.axis = None
        self.figure = None
        self.construction_area_dim = (1.2192, 1.2192)  # 4ft x 4ft
        self.start_area_dim = (0.6096, 0.6096)
        self.map_center = (length / 2, width / 2)
        
        x = (self.construction_area_dim[0] - MAP_EDGE_KEEPOUT) / 2 + MAP_EDGE_KEEPOUT
        y = -1 * x + self.bounds[1]
        self.construction_area_center = (x, y)
        
        self.previous_paths = []
        self.previous_planned_paths = []
        
        self._debug_heading = _debug_heading

    def get_robot_position(self):
        """Returns the position and orientation of the robot"""

        # For DEBUG ONLY
        if self.imu is None:
            print("[PLAN] WORLD MAP IS IN DEBUG MODE !!!")
            return self.robot_position, self._debug_heading

        return self.robot_position, self.imu.get_heading()

    def update_blocks(self, new_blocks: List[Block], metadata):
        """
        Updates the location of blocks in the world map based on the metadata provided
        @param blocks: List of block positions from the robot frame of reference
        """
        
        # add blocks to map
        for new_block in new_blocks:
            found_match = False
            for block in self.blocks:
                if new_block == block:
                    print("hehehhehehe")
                    # Average positions
                    updated_x = (block.location[0] + new_block.location[0]) / 2
                    updated_y = (block.location[1] + new_block.location[1]) / 2
                    block.location = (updated_x, updated_y)
                    found_match = True
                    break
            if not found_match:
                self.blocks.append(new_block)
            
        # remove blocks in map that aren't present
        robo_position = metadata['position']
        robo_heading = metadata['heading_deg']
        rotated_old_blocks = self.translate_to_robo_position(
            robo_position, robo_heading, self.blocks
        )
        rotated_new_blocks = self.translate_to_robo_position(
            robo_position, robo_heading, new_blocks
        )
        remove_idxs = []
        for i, block in enumerate(rotated_old_blocks):  
            x, y = block.location
            if (y < x * np.sqrt(3)/3  and
                y > -1 * x * np.sqrt(3)/3 and
                block not in rotated_new_blocks):
                    remove_idxs.append(i)
        self.blocks = [
            block for i, block in enumerate(self.blocks) if i not in remove_idxs
            ]
                
                
        
    def translate_to_robo_position(self, robo_position, theta_d, blocks):
        theta_r = np.deg2rad(theta_d)
        cos_t = np.cos(theta_r)
        sin_t = np.sin(theta_r)
        x_r, y_r = robo_position

        # Construct the inverse transformation matrix
        T_inv = np.array([
            [ cos_t,  sin_t, -x_r * cos_t - y_r * sin_t],
            [-sin_t,  cos_t,  x_r * sin_t - y_r * cos_t],
            [ 0.0,    0.0,    1.0]
        ])
        
        blocks_rf = copy.deepcopy(blocks)
        for block in blocks_rf:
            block.location = np.array(block.location)
            block.location = T_inv @ np.array([block.location[0], block.location[1], 1])
            block.location = (block.location[0], block.location[1])

        return blocks_rf
        
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
        
        # Draw the starting area
        start_area = patches.Rectangle(
            (0, 0),
            width=self.start_area_dim[0],
            height=self.start_area_dim[1],
            facecolor="none",
            edgecolor="black"
        )
        ax.add_patch(start_area)

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
            
        # Draw the previous paths
        for path in self.previous_paths:
            self.draw_path_on_map(path, "blue", save_fig=False)
        for path in self.previous_planned_paths:
            self.draw_path_on_map(path, "gray", save_fig=False)
            
            
        ax.set_title("World Map")
        ax.set_xlim(-1 * width * 0.02, width * 1.02)
        ax.set_ylim(-1 * height * 0.02, height * 1.02)
       
        meters_to_feet = 3.28084
        feet_interval = 2
        meters_interval = feet_interval / meters_to_feet  # 0.6096 meters
        # Set fixed ticks in meters, spaced to represent 2 feet
        x_start, x_end = ax.get_xlim()
        y_start, y_end = ax.get_ylim()
        ax.set_xticks(np.arange(0, x_end, meters_interval))
        ax.set_yticks(np.arange(0, y_end, meters_interval))
        # Set major tick formatter to convert from meters to feet
        ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: f"{x * meters_to_feet:.1f}"))
        ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda y, pos: f"{y * meters_to_feet:.1f}"))

        # Label axes accordingly
        ax.set_xlabel("Distance (ft)")

        ax.set_axisbelow(True)
        ax.grid(True, which="both", linestyle="--", linewidth=0.5)
    
        

        plt.savefig("world_map.jpg")
        # plt.show()
        # plt.close(fig)  # << clean up the figure from memory if you're doing many plots
        

    def draw_path_on_map(self, path: List[tuple], color:str, save_fig=False):
        """Given a list of x,y points, draws the path on the map"""

        if self.axis is None or self.figure is None:
            print("[PLAN] No map to draw on")
            return

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            self.axis.plot([x1, x2], [y1, y2], color=color, linewidth=2)

        if save_fig:
            plt.savefig("world_map.jpg")
            
    
    def add_to_driven_path(self, path):
        """ 
        Given a path of x, y points, add it to the running list of movements
        Update the robot location to the last point in the given path
        """
        self.previous_paths.append(path)
        self.robot_position = path[-1]
        
    def add_to_planned_paths(self, path):
        self.previous_planned_paths.append(path)

            
    def remove_block(self, block: Block):
        """Removes a block from the world map"""
        self.blocks.remove(block)
        self.draw_map()
        print(f"[PLAN] Removed {block.color} block at {block.location}")
        return True
    
    
    def convert_points_to_movements(self, path):
        """
        Given a list of points, convert them to a list of angles and distances

        Returns a list of angles and disstances in alternating order, starting with the angle.
        For example: [angle1, distance1, angle2, distance2, ...]
        """
        movements = []

        if len(path) < 2:
            print("[WM] Path is too short to convert to movements")
            return False
        base = path[0]
        inc = path[1]

        # heading = self.imu.get_heading()
        angle = np.degrees(np.arctan2(inc[1] - base[1], inc[0] - base[0]))
        movements.append(("Angle", angle))
        distance = np.sqrt((inc[0] - base[0]) ** 2 + (inc[1] - base[1]) ** 2)
        # loop over the path converting to angles and distances
        # if the next point has the same heading as the previous point,
        # then we can just add the distance
        for i in range(2, len(path)):
            check = path[i]
            angle_check = np.degrees(np.arctan2(check[1] - inc[1], check[0] - inc[0]))
            if abs(angle_check - angle) < 0.01:
                distance += np.sqrt((check[0] - inc[0]) ** 2 + (check[1] - inc[1]) ** 2)
            else:
                movements.append(("Distance", distance))
                movements.append(("Angle", angle_check))
                distance = np.sqrt((check[0] - inc[0]) ** 2 + (check[1] - inc[1]) ** 2)
            angle = angle_check
            inc = check
        movements.append(("Distance", distance))
        return movements
    
    
    def convert_movements_to_points(self, start_point, movements):
        """
        Given a list of movements the robot took, return the path in the world frame.
        """
        angle = self.get_robot_position()[1]
        path = [start_point]
        base = start_point
        for i in movements:
            if i[0] == "Angle":
                angle = i[1]
                # print(f"[RMC][path_from_move]: Angle: {angle}")
            elif i[0] == "Distance":
                distance = i[1]
                # print(f"[RMC][path_from_move]: Distance: {distance}")
                base = (
                    base[0] + distance * np.cos(np.radians(angle)),
                    base[1] + distance * np.sin(np.radians(angle)),
                )
                path.append(base)
        return path


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
        self.target_block = None
        self.GRID_SIZE = 0.1
        self.wm = world_map
        

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
        blocks = self.wm.get_blocks()
        for block in blocks:
            if (
                self._close_together(position, block.location, threshold=BLOCK_KEEPOUT)
                and block.color != self.target_color
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

    def astar(self, start, goal, target_color=None, debug=0):
        # setup
        self.target_color = target_color
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
                        open_path = [neighbor.parent.position, neighbor.position]
                        self.wm.draw_path_on_map(open_path, "gray", False)
                        if debug > 1:
                            self.wm.draw_path_on_map(open_path, "gray", True)
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
        path = self.reconstruct_path(last_node)
        # self.wm.draw_path_on_map(path, "gray", True)
        if debug > 0:
            self.wm.draw_path_on_map(path, "red", True)
        return path



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

    
