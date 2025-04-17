from utilities.RobotMotorControl import roboMotorControl
import matplotlib.pyplot as plt
import numpy as np

rmc = roboMotorControl()

input("Hit any key to start!")

LONG = 0.5
SHORT = 0.5

heading = rmc.imu.get_heading()
pos_list = np.array([(0, 0)])
print("Heading: ", heading)

# for _ in range(2):
for _ in range(1):
    
    distance = rmc.forward(LONG)[0]
    pos = (distance*np.cos(heading), distance*np.sin(heading))
    pos_list = np.vstack(( pos_list, pos))
    print("x: ", pos[0], "   y: ", pos[1])
    print()
    
    heading = rmc.orient_to(270)
    print("Heading: ", heading)
    distance = rmc.forward(SHORT)[0]
    pos = (distance*np.cos(heading), distance*np.sin(heading))
    pos_list = np.vstack(( pos_list, pos))
    print("x: ", pos[0], "   y: ", pos[1])
    print()
    
    # heading = rmc.orient_to(180)
    # print("Heading: ", heading)
    # distance = rmc.forward(LONG)[0]
    # pos = (distance*np.cos(heading), distance*np.sin(heading))
    # pos_list = np.vstack(( pos_list, pos))
    # print("x: ", pos[0], "   y: ", pos[1])
    # print()
    
    # heading = rmc.orient_to(90)
    # print("Heading: ", heading)
    # distance = rmc.forward(SHORT)[0]
    # pos = (distance*np.cos(heading), distance*np.sin(heading))
    # pos_list = np.vstack(( pos_list, pos))
    # print("x: ", pos[0], "   y: ", pos[1])
    # print()
    
    # heading = rmc.orient_to(0)
    # print("Heading: ", heading)
    
print("Course Complete!")
rmc.game_over()
    
print(pos_list)



plt.plot(pos_list[:,0], pos_list[:,1])
plt.grid()
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Robot Position in World Frame")
plt.savefig("rectangle_output.png")