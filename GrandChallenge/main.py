from utilities.RobotMotorControl import RobotMotorControl
from utilities.vision import Camera
from utilities.imu import IMU
from utilities.odometry import Odometer, WorldMap


# set up state machine
START = 0
COLLECT_BLOCK = 1
DELIVER_BLOCK = 2
SEARCH = 3
END = 4

CURRENT_STATE = START

cam = Camera()
imu = IMU()
odom = Odometer()
world_map = WorldMap()
rmc = RobotMotorControl(imu=imu, odom=odom)

# Begin state machine
while CURRENT_STATE != END:
    print("here")
    CURRENT_STATE = END


# State is now End
