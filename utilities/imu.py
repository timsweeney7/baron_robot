import serial
import re
import time

# connect to the arduino nano
ser = serial.Serial('/dev/ttyUSB0', 115200)

class IMU():
    
    def __init__(self):
    
        # ignore the first 10 prints from the IMU
        count = 0
        while True:
            if(ser.in_waiting > 0):
                count += 1
                line = ser.readline()
                
                if count <= 10:
                    continue
                else:
                    break
                
    def get_heading(self):
        ser.reset_input_buffer()
        while(ser.in_waiting<1):
            continue
        line = str(ser.readline())
        data = re.findall("\d+\.\d+", line)
        x = float(data[0])
        # print(f"Heading = {x}")
        return x
        
        
        
if __name__ == "__main__":
    
    imu = IMU()    
    while True:
        print(imu.get_heading())
        