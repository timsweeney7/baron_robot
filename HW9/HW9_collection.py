from utilities.RobotMotorControl import roboMotorControl
from utilities.vision import Camera
import time

block_collected = False
cam  = Camera()
rmc = roboMotorControl()

rmc.open_gripper()

input("Press enter to start.")

while not block_collected:
    rgb_image = cam.capture_image()
    blocked_image, blocks  = cam.find_blocks(rgb_image)
    if blocks == None:
        print("NO BLOCK DETECTED! - EXITING")
        exit()
    if blocks[0].knocked_over is True:
        rmc.close_gripper()
        print("Block collected!! - Exiting")
        time.sleep(10)
        rmc.open_gripper()
        rmc.game_over()
        exit()
    
    # Get block heading and orient to it
    rotation_angle = blocks[0].angle_to_robo
    rmc.rotate_by(rotation_angle)
    
    # Roll Based on area
    if blocks[0].bounding_area > 10_000:
        rmc.forward(0.3048)
    else:
        rmc.forward(0.3048/2)