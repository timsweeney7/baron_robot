import cv2 as cv
import numpy as np

from utilities.RobotMotorControl import RobotMotorControl
from utilities.vision import Camera
from utilities.imu import IMU
from utilities.odometry import Odometer, WorldMap


# World Parameters
# HOME ----------
MAP_LENGTH = 2.54
MAP_WIDTH = 1.829

# set up state machine
START = 0
COLLECT_BLOCK = 1
DELIVER_BLOCK = 2
SEARCH = 3
END = 4

CURRENT_STATE = START

imu = IMU()
world_map = WorldMap(length=MAP_LENGTH, width=MAP_WIDTH, imu=imu)
cam = Camera(world_map=world_map)
odom = Odometer()
rmc = RobotMotorControl(imu=imu, odom=odom)

print("[Main] Setup complete")

# Start state machine
frame, metadata = cam.capture_image()
cv.imwrite("rgb_image.jpg", cv.cvtColor(frame, cv.COLOR_RGB2BGR))
frame, blocks = cam.find_blocks(frame=frame, metadata=metadata)

cv.imwrite("block_image.jpg", cv.cvtColor(frame, cv.COLOR_RGB2BGR))
if len(blocks) == 0:
    print("[Main] No blocks found")
    CURRENT_STATE = SEARCH
else:
    world_map.update_blocks(blocks)

# Enter loop
while CURRENT_STATE != END:
    print("[Main] Entered main loop")
    CURRENT_STATE = END

# plot the output data
world_map.draw_map()

# State is now END
odom.kill()
imu.kill()
print("[Main] Course complete")

