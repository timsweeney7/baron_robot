from typing import List
import numpy as np
import threading
import queue
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import pigpio

from utilities.block import Block
from utilities.imu import IMU

# for Odometer
RIGHT_ENCODER = 18 # right
LEFT_ENCODER = 4 # left 

# for mapping
KEEPOUT = 0.3048/1.5

class Odometer():
    
    def __init__(self):
        
        self.pi = pigpio.pi()
        self.pi.set_mode(RIGHT_ENCODER, pigpio.INPUT)
        self.pi.set_mode(LEFT_ENCODER, pigpio.INPUT)
        self.right_ticks = 0
        self.left_ticks = 0
        
        self._running = False
        self.lock = threading.Lock()
        self._T1 = threading.Thread(target=self.start)
        self._T1.start()
        
    def reset(self):
        with self.lock:
            self.right_ticks = 0
            self.left_ticks = 0
            
    def kill(self):
        self._running = False

    def start(self):
        
        print("[Odometer] Started thread")
        self._running = True
        buttonBR = np.int(0)
        buttonFL = np.int(0)
        
        while self._running:
            
            newBR = int(self.pi.read(RIGHT_ENCODER))
            newFL = int(self.pi.read(LEFT_ENCODER))
            
            with self.lock:
                if newBR != buttonBR:
                    buttonBR = newBR
                    self.right_ticks += 1
                if newFL != buttonFL:
                    buttonFL = newFL
                    self.left_ticks += 1
    
    def get_ticks(self):
        with self.lock:
            return self.left_ticks, self.right_ticks
    
    def get_distance(self):
        with self.lock:
            return self.ticks_to_distance(self.left_ticks), self.ticks_to_distance(self.right_ticks)
        
    
    @staticmethod
    def distance_to_ticks(dist:float) -> int:
        """ 
        Returns the number of encoder ticks that corresponds to an input distance
        Input is in meters
        Output is encoder ticks
        """
        rotations = (dist) * (1/(2*np.pi*0.0325))
        ticks = rotations * 20
        print("wheel rotations = ", rotations)
        print("Estimated Ticks = ", ticks)
        return  ticks 

    @staticmethod
    def ticks_to_distance(ticks:int) -> float:
        """ 
        Returns the distance that corresponds to number of encoder ticks
        Input is in encoder ticks
        Output is in distance
        """
        rotations = ticks/20
        distance = rotations  * (2*np.pi*0.0325)
        return distance
    
    
class WorldMap():
    
    def __init__(self, length, width, imu:IMU):
        self.objects = []   # list of x,y locations of objects in the world
        self.bounds = (length, width)   # list of x,y locations for the bounds of the world
        self.robot_position = (0.4826, 0.25)
        self.imu = imu
        
    def get_robot_position(self):
        """ Returns the position and orientation of the robot """
        return self.robot_position, self.imu.get_heading()
    
    def update_blocks(self, blocks:List[Block], metadata):
        """ 
        Updates the location of blocks in the world map based on the metadata provided
        @param blocks: List of block positions from the robot frame of reference
        """
        for block in blocks:
            block_global = self.translate_to_world_frame(point= block.location,
                                                         angle= metadata['heading_deg'],
                                                         robo_pos= metadata['position'])
            block_global_x = block_global[0,0]
            block_global_y = block_global[1,0]
            self.objects.append((block_global_x, block_global_y, block.color))
            
    def translate_to_world_frame(self, point, angle, robo_pos):
        px, py = point
        rx, ry = robo_pos
        P_robo = np.array([[rx],
                           [ry],
                           [1]])
        T = np.array([[np.cos(angle), -1*np.sin(angle), px],
                      [np.sin(angle), np.cos(angle), py],
                      [0,  0,   1]])
        return T @ P_robo
    
    def draw_map(self):
        
        ax = plt.gca()
        ax.set_aspect('equal')
        
        # Draw the bounds of the map
        width, height = self.bounds
        arena = patches.Rectangle((0, 0), width=width, height=height, facecolor='none', edgecolor='black')
        ax.add_patch(arena)
        
        # Draw the robot
        location, heading_deg = self.get_robot_position()
        robo_x, robo_y = location
        triangle = np.array([   # homogeneous coordinates
            [0.0762, 0, 1],     # Robot origin is at the base of the claw
            [-0.254, 0.1016, 1],
            [-0.254, -0.1016, 1]
        ])
        theta = np.deg2rad(heading_deg)
        c, s = np.cos(theta), np.sin(theta)
        T = np.array([
            [c, -s, robo_x],
            [s,  c, robo_y],
            [0,  0,       1]
        ])
        triangle = (T @ triangle.T).T[:, :2]  # Drop the homogeneous coordinate
        robot_patch = patches.Polygon(triangle, closed=True, color='black')
        ax.add_patch(robot_patch)
        
        # Draw the blocks
        # Unpack into separate x and y lists
        
        for block in self.objects:
            x = block[0]
            y = block[1]
            color = block[2]
            plt.scatter(x, y, c=color)
            circle = plt.Circle((x, y), KEEPOUT, color=color, fill=False, clip_on=True)
            ax.add_patch(circle)
            
        plt.title("World Map")
        plt.xlim(-1*width*0.02, width*1.02)
        plt.ylim(-1*height*0.02, height*1.02)
        plt.xlabel("Meters")
        plt.savefig("world_map.jpg")
        plt.grid(True)
        pass