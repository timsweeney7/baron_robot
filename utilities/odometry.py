from typing import List
import numpy as np
import threading
import queue
import matplotlib.pyplot as plt

import pigpio

from utilities.vision import Block
from utilities.imu import IMU


RIGHT_ENCODER = 18 # right
LEFT_ENCODER = 4 # left 

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
        self.bounds = []    # list of x,y locations for the bounds of the world
        self.robot_position = (0, 0)
        self.imu = imu
        
    def get_robot_position(self):
        """ Returns the position and orientation of the robot """
        return self.robot_position, self.imu.get_heading()
    
    def update_blocks(self, blocks:List[Block], metadata):
        """ 
        Updates the location of blocks in the world map
        @param blocks: List of block positions from the robot frame of reference
        """
        robo_x, robo_y  = metadata['position']
        robo_heading = metadata['heading']
        for block in blocks:
            block_local_x, block_local_y = block.location
            block_global_x = robo_x + block_local_x
            block_global_y = robo_y + block_local_y
            self.objects.append((block_global_x, block_global_y))
    
    def draw_map(self):
        # Unpack into separate x and y lists
        x_vals, y_vals = zip(*self.objects)
        plt.scatter(x_vals, y=y_vals)
        plt.title("World Map")
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        plt.savefig("world_map.jpg")
        plt.grid()
        pass