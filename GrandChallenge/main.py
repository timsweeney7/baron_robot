import cv2 as cv
import numpy as np
from enum import IntEnum

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
PLAN_PATH_TO_BLOCK = 1
DRIVE_PATH_TO_BLOCK = 2
COLLECT_BLOCK = 3
PLAN_PATH_TO_CONSTRUCTION = 4
DRIVE_PATH_TO_CONSTRUCTION = 5
RELEASE_BLOCK = 6
SEARCH = 7
END = 8


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
            CURRENT_STATE = PLAN_PATH_TO_BLOCK
        rmc.open_gripper()

    elif CURRENT_STATE == PLAN_PATH_TO_BLOCK:
        print("[Main] Planning path")
        goal = planner.select_goal()
        path = planner.astar(
            start=world_map.get_robot_position()[0], goal=planner.target_block.location
        )
        if path is None:
            print("[Main] No path found")
            exit()
            #TODO: change to SEARCH state
        
        world_map.draw_path_on_map(path, "gray", save_fig=True)
        rmc.set_path_from_points(path)
        CURRENT_STATE = DRIVE_PATH_TO_BLOCK
        input('1')

    elif CURRENT_STATE == DRIVE_PATH_TO_BLOCK:
        print("[Main] Driving path")
        driven_path = rmc.drive_path()
        world_map.draw_path_on_map(path, "blue", save_fig=True)
        CURRENT_STATE = COLLECT_BLOCK
        print("[Main] Path complete")
        input('2')

    elif CURRENT_STATE == COLLECT_BLOCK:
        print("[Main] Collecting block")
        collection_movements = []

        block_collected = False
        while not block_collected:
            rgb_image, metadata = cam.capture_image()
            cv.imwrite("rgb_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
            blocked_image, blocks = cam.find_blocks(rgb_image, metadata)
            cv.imwrite("block_image.jpg", cv.cvtColor(blocked_image, cv.COLOR_RGB2BGR))
            correct_color_in_frame = False
            if len(blocks) == 0:
                print("[MAIN] NO BLOCK DETECTED!")
                rmc.rotate_by(30)
            for block in blocks:
                if block.color == goal.color:
                    target_block = block
                    correct_color_in_frame = True
            if not correct_color_in_frame:
                print("[MAIN] Correct color block not found in frame")
                rmc.rotate_by(30)
                continue
            if target_block.bounding_origin[1] >= BLOCK_IN_GRIP_RANGE: # if bounding box is low in frame
                rmc.close_gripper()
                block_collected = True
                print(f"[MAIN] {target_block.color} block collected!")
                world_map.remove_block(goal)
                CURRENT_STATE = PLAN_PATH_TO_CONSTRUCTION
                continue
            # Get block heading and orient to it
            goal_angle = target_block.angle_to_robo
            rotation_angle = rmc.rotate_by(goal_angle)
            forward_distance = rmc.forward(0.07)
            collection_movements.append(
                (
                    ("Angle", rotation_angle),
                    ("Distance", forward_distance)
                )
            )
        # Add collection movements to world map, update robot position
        collection_path = rmc.get_path_from_movements(collection_movements)
        print(collection_path)
        world_map.draw_path_on_map(collection_path, "blue", save_fig=True)
        input('3')
        world_map.draw_map()
        input('4')
        
    elif CURRENT_STATE == PLAN_PATH_TO_CONSTRUCTION:
        print("[MAIN] Delivering block")
        start = world_map.get_robot_position()[0]
        goal = world_map.construction_area_center
        path = planner.astar(start=start, goal=goal, debug=1)
        world_map.draw_path_on_map(path, "gray", save_fig=True)
        CURRENT_STATE = DRIVE_PATH_TO_CONSTRUCTION
        input('4')
        
    elif CURRENT_STATE == DRIVE_PATH_TO_CONSTRUCTION:
        print("[Main] Driving path to construction")
        driven_path = rmc.drive_path()
        world_map.draw_path_on_map(path, "blue", save_fig=True)
        CURRENT_STATE = RELEASE_BLOCK
        print("[Main] Path complete")
        input('5')
        
    elif CURRENT_STATE == RELEASE_BLOCK:
        print("[MAIN] Releasing block")
        rmc.open_gripper()
        # TODO: add code to back up?
        CURRENT_STATE = PLAN_PATH_TO_BLOCK

    elif CURRENT_STATE == SEARCH:
        print("[MAIN] Entered Search Mode")
        CURRENT_STATE = END


# plot the output data
# world_map.draw_map()

# State is now END
odom.kill()
imu.kill()
print("[Main] Course complete")
