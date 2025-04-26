import serial
import re
import threading


class IMU():
    
    def __init__(self):
        
        self.heading = 0
        
        self._running = False
        self.lock = threading.Lock()
        self._T1 = threading.Thread(target=self.start)
        self._T1.start()
        
        # connect to the arduino nano
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
    
        # ignore the first 10 prints from the IMU
        count = 0
        while True:
            if(self.ser.in_waiting > 0):
                count += 1
                line = self.ser.readline()
                if count <= 10:
                    continue
                else:
                    break
        
    def kill(self):
        self._running = False
        
    def start(self):
        
        print("[IMU] Started thread")
        self._running = True
        
        while self._running:
            self.ser.reset_input_buffer()
            while(self.ser.in_waiting<1):
                continue
            line = str(self.ser.readline())
            data = re.findall("\d+\.\d+", line)
            _heading = float(data[0])

            with self.lock:
                self.heading = _heading
        
    def get_heading(self):
        with self.lock:
            return self.heading
        
        
        
if __name__ == "__main__":
    
    imu = IMU()    
    while True:
        print(imu.get_heading())
        