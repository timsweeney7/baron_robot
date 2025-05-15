from utilities.RobotMotorControl import RobotMotorControl
from utilities.imu import IMU
from utilities.path_planner import WorldMap
from utilities.odometry import Odometer
import time

imu = IMU()
wm = WorldMap(10, 10, None)
odom = Odometer()
rmc = RobotMotorControl(imu=imu, odom=odom)

time.sleep(3)
rmc.rotate_by(45)
rmc.close_gripper()
time.sleep(0.5)
rmc.open_gripper()
time.sleep(0.5)
