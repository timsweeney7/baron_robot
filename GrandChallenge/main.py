import cv2 as cv
import numpy as np

from utilities.RobotMotorControl import RobotMotorControl
from utilities.vision import Camera
from utilities.imu import IMU
from utilities.odometry import Odometer, WorldMap
from utilities.path_planner import PathPlanner


# World Parameters
# HOME ----------
MAP_LENGTH = 2.54
MAP_WIDTH = 1.829

# set up state machine
START = 0
PLAN_PATH = 1
COLLECT_BLOCK = 2
DELIVER_BLOCK = 3
SEARCH = 4
DRIVE_PATH = 5
END = 6


imu = IMU()
world_map = WorldMap(length=MAP_LENGTH, width=MAP_WIDTH, imu=imu)
cam = Camera(world_map=world_map)
odom = Odometer()
rmc = RobotMotorControl(imu=imu, odom=odom)
planner = PathPlanner(world_map=world_map)

print("[Main] Setup complete")

# Start state machine
CURRENT_STATE = START

# Enter loop
while CURRENT_STATE != END:
    
    if CURRENT_STATE == START:
        print("[Main] Starting collection program")
        frame, metadata = cam.capture_image()
        cv.imwrite("rgb_image.jpg", cv.cvtColor(frame, cv.COLOR_RGB2BGR))
        frame, blocks = cam.find_blocks(frame=frame, metadata=metadata)
        cv.imwrite("block_image.jpg", cv.cvtColor(frame, cv.COLOR_RGB2BGR))

        if len(blocks) == 0:
            CURRENT_STATE = SEARCH
            print("[Main] No blocks found")
        else:
            world_map.update_blocks(blocks)
            world_map.draw_map()
            CURRENT_STATE = PLAN_PATH
    
    elif CURRENT_STATE == PLAN_PATH:
        print("[Main] Planning path")
        planner.select_goal()
        path = planner.astar(start=world_map.get_robot_position()[0], goal=planner.target_block.location)
        if path is None:
            print("[Main] No path found")
        world_map.draw_path_on_map(path)
        rmc.set_path(path)
        CURRENT_STATE = DRIVE_PATH
    
    elif CURRENT_STATE == DRIVE_PATH:
        print("[Main] Driving path")
        rmc.drive_path()
        CURRENT_STATE = COLLECT_BLOCK
        print("[Main] Path complete")

    elif CURRENT_STATE == COLLECT_BLOCK:
        print("[Main] Collecting block")
        
        block_collected = False
        while not block_collected:
            rgb_image = cam.capture_image()
            blocked_image, blocks  = cam.find_blocks(rgb_image)
            if blocks == None:
                print("[MAIN] NO BLOCK DETECTED!")
                rmc.rotate_by(15)
            if blocks[0].knocked_over is True:
                rmc.close_gripper()
                print("[MAIN] Block collected!")
                CURRENT_STATE = DELIVER_BLOCK
            # Get block heading and orient to it
            rotation_angle = blocks[0].angle_to_robo
            rmc.rotate_by(rotation_angle)
            # Roll Based on area
            rmc.forward(0.1)
                
    elif CURRENT_STATE == DELIVER_BLOCK:
        print("[MAIN] Delivering block")
        CURRENT_STATE == END
        
        
    elif CURRENT_STATE == SEARCH:
        print("[MAIN] Entered Search Mode")
        CURRENT_STATE == END
        
        
# plot the output data
world_map.draw_map()

# State is now END
odom.kill()
imu.kill()
print("[Main] Course complete")

