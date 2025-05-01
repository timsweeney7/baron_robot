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

    def get_robot_position(self):
        """Returns the position and orientation of the robot"""

        # For DEBUG ONLY
        if self.imu is None:
            print("[ODOMETER] WORLD MAP IS IN DEBUG MODE !!!")
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

        plt.savefig("world_map.jpg")
        # plt.show()

        # plt.close(fig)  # << clean up the figure from memory if you're doing many plots
        self.axis = ax
        self.figure = fig

    def draw_path_on_map(self, path: List[tuple], color, save_fig=False):
        """Draws the path on the map"""

        if self.axis is None or self.figure is None:
            print("[ODOMETER] No map to draw on")
            return

        print(f"[ODOMETER] Drawing path on map: {path}")

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            self.axis.plot([x1, x2], [y1, y2], color=color, linewidth=2)

        if save_fig:
            plt.savefig("planned_path.jpg")


class Node:
    def __init__(self, position, cost_to_come, estimated_total_cost, parent=None):
        self.position = position
        self.cost_to_come = cost_to_come
        self.estimated_total_cost = estimated_total_cost
        self.parent = parent

    def __lt__(self, other):
        return self.estimated_total_cost < other.estimated_total_cost


def _heuristic_calc(start, end):
    """Start and end are tuples of (x, y) coordinates"""
    return np.sqrt((end[0] - start[0]) ** 2 + (end[0] - start[0]) ** 2)


class PathPlanner:

    def __init__(self, world_map: WorldMap = None):
        self.MAX_WIDTH, self.MAX_HEIGHT = world_map.bounds
        self.blocks = world_map.get_blocks() if world_map else []
        self.target_block = None
        self.GRID_SIZE = 0.15
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
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # Reverse

    def _valid_move(self, position):
        """Check if the move is valid within the world map"""
        print(f"Valid move? {position}", end="  ")
        x, y = position
        # Return False if out of bounds
        if (
            x < 0 + MAP_EDGE_KEEPOUT
            or x > self.MAX_WIDTH - MAP_EDGE_KEEPOUT
            or y < 0 + MAP_EDGE_KEEPOUT
            or y > self.MAX_HEIGHT - MAP_EDGE_KEEPOUT
        ):
            print("no")
            return False
        # Check if the move is within the boundary of a block
        for block in self.blocks:
            if (
                self._close_together(position, block.location, threshold=BLOCK_KEEPOUT)
                and block.color != self.target_block.color
            ):
                print("no")
                return False
        print('yes')
        return True  # Valid move

    def _get_neighbors(self, node: Node):
        """Gets all of the neighboring positions and their move costs"""

        x, y = node.position

        # fmt: off
        neighbors = [
            ((round(x + self.GRID_SIZE, 5), round(y                 , 5)), 1),  # straight
            ((round(x                 , 5), round(y + self.GRID_SIZE, 5)), 1),
            ((round(x - self.GRID_SIZE, 5), round(y                 , 5)), 1),
            ((round(x                 , 5), round(y - self.GRID_SIZE, 5)), 1),
            ((round(x + self.GRID_SIZE, 5), round(y + self.GRID_SIZE, 5)), np.sqrt(2)+1),  # diagonal
            ((round(x - self.GRID_SIZE, 5), round(y + self.GRID_SIZE, 5)), np.sqrt(2)+1),
            ((round(x - self.GRID_SIZE, 5), round(y - self.GRID_SIZE, 5)), np.sqrt(2)+1),
            ((round(x + self.GRID_SIZE, 5), round(y - self.GRID_SIZE, 5)), np.sqrt(2)+1),
        ]
        # fmt: on

        # Filter out neighbors that are not valid moves
        valid_neighbors = [
            neighbor for neighbor in neighbors
            if self._valid_move(neighbor[0])
        ]

        return valid_neighbors

    def _close_together(self, current, goal, threshold):
        """Check if the current position is close to the goal"""
        return (
            np.sqrt((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2)
            < threshold
        )

    def astar(self, start, goal, debug=False):
        open_heap = []
        heapq.heappush(open_heap, Node(start, 0, _heuristic_calc(start, goal), None))
        closed_set = set()

        counter = 0
        while open_heap:
            print(len(open_heap))
            current = heapq.heappop(open_heap)

            if self._close_together(
                current.position, goal, threshold=BLOCK_KEEPOUT + 0.2
            ):
                print("[PLAN] Found path to goal")
                return self.reconstruct_path(current)

            closed_set.add(current.position)

            if debug and (current.parent is not None):
                closed_path = [current.parent.position, current.position]
                self.wm.draw_path_on_map(closed_path, "blue", True)
                input("...")  # give time to review map

            for neighbor_pos, move_cost in self._get_neighbors(current):
                if debug:
                    open_path = [current.position, neighbor_pos]
                    self.wm.draw_path_on_map(open_path, "gray")
                if neighbor_pos in closed_set:
                    continue

                new_cost_to_come = current.cost_to_come + move_cost
                neighbor_node = Node(
                    position=neighbor_pos,
                    cost_to_come=new_cost_to_come,
                    estimated_total_cost=new_cost_to_come
                    + _heuristic_calc(neighbor_pos, goal),
                    parent=current,
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

    block1 = Block(color="GREEN", location=(1.65, 0.495))

    block2 = Block(color="CYAN", location=(0.9, 0.235))

    block3 = Block(color="RED", location=(2.0, 1.25))

    MAP_LENGTH = 2.54
    MAP_WIDTH = 1.829
    wm = WorldMap(MAP_LENGTH, MAP_WIDTH, None)
    wm.draw_map()

    wm.update_blocks([block1, block2, block3])

    path_planner = PathPlanner(wm)
    path_planner.select_goal()

    start = (0.25, 0.25)
    path = path_planner.astar(start, path_planner.target_block.location, debug=True)
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

    wm.draw_path_on_map(path)
