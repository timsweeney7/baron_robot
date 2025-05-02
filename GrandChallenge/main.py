import cv2 as cv
import numpy as np

from utilities.RobotMotorControl import RobotMotorControl
from utilities.vision import Camera
from utilities.imu import IMU
from utilities.odometry import Odometer
from utilities.path_planner import PathPlanner, WorldMap


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


# Value for determining if a block can be picked up
# Higher values require the block to be closer to the robot
BLOCK_IN_GRIP_RANGE = 270


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
        rmc.open_gripper()

    elif CURRENT_STATE == PLAN_PATH:
        print("[Main] Planning path")
        goal = planner.select_goal()
        path = planner.astar(
            start=world_map.get_robot_position()[0], goal=planner.target_block.location
        )
        if path is None:
            print("[Main] No path found")
        
        print()     # debug
        for _ in path:
            print(f"{_}")
        print()
        
        world_map.draw_path_on_map(path, "blue", save_fig=True)
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
            rgb_image, metadata = cam.capture_image()
            cv.imwrite("rgb_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
            blocked_image, blocks = cam.find_blocks(rgb_image, metadata)
            cv.imwrite("block_image.jpg", cv.cvtColor(blocked_image, cv.COLOR_RGB2BGR))
            if len(blocks) == 0:
                print("[MAIN] NO BLOCK DETECTED!")
                rmc.rotate_by(15)
            for block in blocks:
                if block.color == goal.color:
                    target_block = block
            if target_block.bounding_origin[1] >= BLOCK_IN_GRIP_RANGE:
                rmc.close_gripper()
                block_collected = True
                print("[MAIN] Block collected!")
                CURRENT_STATE = DELIVER_BLOCK
                continue
            # Get block heading and orient to it
            rotation_angle = block.angle_to_robo
            rmc.rotate_by(rotation_angle)
            rmc.forward(0.07)

    elif CURRENT_STATE == DELIVER_BLOCK:
        print("[MAIN] Delivering block")
        CURRENT_STATE = END

    elif CURRENT_STATE == SEARCH:
        print("[MAIN] Entered Search Mode")
        CURRENT_STATE = END


# plot the output data
world_map.draw_map()

# State is now END
odom.kill()
imu.kill()
print("[Main] Course complete")
