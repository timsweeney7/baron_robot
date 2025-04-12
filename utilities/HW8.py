from RobotMotorControl import roboMotorControl
import matplotlib.pyplot as plt
import numpy as np

rmc = roboMotorControl()


LONG = 1.4224
SHORT = 1.0668

heading = rmc.imu.get_heading()

pos_list = np.array([(0, 0)])

for _ in range(2):
    
    distance = rmc.forward(LONG)
    pos = (distance*np.cos(heading), distance*np.sin(heading))
    pos_list = np.vstack(( pos_list, pos))
    
    heading = rmc.orient_to(270)
    distance = rmc.forward(SHORT)
    pos = (distance*np.cos(heading), distance*np.sin(heading))
    pos_list = np.vstack(( pos_list, pos))
    
    heading = rmc.orient_to(180)
    distance = rmc.forward(LONG)
    pos = (distance*np.cos(heading), distance*np.sin(heading))
    pos_list = np.vstack(( pos_list, pos))
    
    heading = rmc.orient_to(90)
    distance = rmc.forward(SHORT)
    pos = (distance*np.cos(heading), distance*np.sin(heading))
    pos_list = np.vstack(( pos_list, pos))
    
    heading = rmc.orient_to(0)
    
print(np.shape(pos_list))

plt.title("Robot Position in World Frame")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.savefig("rectangle_output.png")