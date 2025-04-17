import numpy as np
import threading
import queue
import pigpio

RIGHT_ENCODER = 18 # right
LEFT_ENCODER = 4 # left 

class odometer():
    
    def __init__(self):
        
        self.pi = pigpio.pi()
        self.pi.set_mode(RIGHT_ENCODER, pigpio.INPUT)
        self.pi.set_mode(LEFT_ENCODER, pigpio.INPUT)
        self.right_ticks = 0
        self.left_ticks = 0
        
        self.lock = threading.Lock()
        self._T1 = threading.Thread(target=self.start)
        self._T1.start()
        self._running = False
        
        
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