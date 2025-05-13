"""
each run is 10 minutes max
"""

import cv2 as cv
import numpy as np
import time

from utilities.RobotMotorControl import RobotMotorControl
from utilities.vision import Camera
from utilities.imu import IMU
from utilities.odometry import Odometer
from utilities.path_planner import PathPlanner, WorldMap
from utilities.PingSensor import pingSensor


# World Parameters
# HOME ----------
MAP_LENGTH = 3.048 # 10ft
MAP_WIDTH = 2.7432 # 9ft

# ARENA ----------
# MAP_LENGTH = 3.048 # 10ft
# MAP_WIDTH = 3.048 # 10ft

# set up state machine
GET_NEXT_GOAL = 0
PLAN_PATH_TO_BLOCK = 1
DRIVE_PATH_TO_BLOCK = 2
COLLECT_BLOCK = 3
PLAN_PATH_TO_CONSTRUCTION = 4
DRIVE_PATH_TO_CONSTRUCTION = 5
RELEASE_BLOCK = 6
SEARCH = 7
SCAN_AREA = 9
END = 8
START = 10
REORIENT = 11
CLEANUP = 12

# Value for determining if a block can be picked up
# Higher values require the block to be closer to the robot
BLOCK_IN_GRIP_RANGE = 300

# after 3 blocks, reorient the robot
reorient_counter = 0



fourcc = cv.VideoWriter_fourcc(*'mp4v')
out = cv.VideoWriter('drive_video.mp4', fourcc, 1.0, (800, 456))


def capture_and_process_image(cam:Camera, wm:WorldMap, got_that_thang_on_me:bool=False):
    """
    Takes an image and processes it.  Returns True if a block is identified.
    Returns False if no blocks are identified.
    """
    frame, metadata = cam.capture_image()
    # print("metadata: ", metadata)
    cv.imwrite("rgb_image.jpg", cv.cvtColor(frame, cv.COLOR_RGB2BGR))
    frame, blocks = cam.find_blocks(frame=frame, metadata=metadata)
    bgr_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
    cv.imwrite("block_image.jpg", bgr_frame)
    
    out.write(bgr_frame)
    
    if got_that_thang_on_me:
        blocks = [block for block in blocks if block.color != target_color]

    wm.update_blocks(blocks, metadata)
    wm.draw_map()
    
    return blocks

def take_step_update_map():
    position, heading = world_map.get_robot_position()
    realized_movement = rmc.drive_next_path_step()
    realized_path = world_map.convert_movements_to_points(position, [realized_movement])
    world_map.add_to_driven_path(realized_path)
    

def do_arena_scan(got_that_thang_on_me:bool=False):
    print("[MAIN] Doing arena scan")
    for angle in range(0, 360, 45):
        rmc.orient_to(angle)
        capture_and_process_image(cam=cam, wm=world_map, got_that_thang_on_me=got_that_thang_on_me)
        
def do_pickup_area_scan():
    print("[MAIN] Scanning pickup area")
    for angle in [0, 45, -45]:
        rmc.orient_to(angle)
        capture_and_process_image(cam=cam, wm=world_map, got_that_thang_on_me=False)



if __name__ == "__main__":
    
    block_order = ["RED", "GREEN", "CYAN", "RED", "GREEN", "CYAN", "RED", "GREEN", "CYAN"]

    imu = IMU()
    world_map = WorldMap(length=MAP_LENGTH, width=MAP_WIDTH, imu=imu)
    cam = Camera(world_map=world_map)
    odom = Odometer()
    rmc = RobotMotorControl(imu=imu, odom=odom)
    planner = PathPlanner(world_map=world_map)
    ping = pingSensor()

    print("[Main] Setup complete")
    print("[Main] Starting collection program")
    rmc.open_gripper()
    world_map.draw_map()
    position, heading = world_map.get_robot_position()

    # Start state machine
    CURRENT_STATE = START

    # Enter loop
    while CURRENT_STATE != END:
        
        if CURRENT_STATE == START:
            print("[Main] Starting program")
            capture_and_process_image(cam=cam, wm=world_map)
            rmc.orient_to(45)
            capture_and_process_image(cam=cam, wm=world_map)
            CURRENT_STATE = GET_NEXT_GOAL
            # input("1")

        if CURRENT_STATE == GET_NEXT_GOAL:
                        
            if len(block_order) == 0:
                CURRENT_STATE = CLEANUP
                continue
            # get the next target then remove from list
            target_color = block_order[0]
            print(f"[MAIN] Goal block: {target_color}")
            block_order.remove(block_order[0])
            
            blocks = world_map.get_blocks()
            for block in blocks:
                if block.color == target_color:
                    print(f"[MAIN] Found {target_color} block at {block.location}")
                    goal = block
                    CURRENT_STATE = PLAN_PATH_TO_BLOCK
                    break
            else:
                print(f"[MAIN] No {target_color} block found!")
                CURRENT_STATE = SEARCH        
                

        elif CURRENT_STATE == PLAN_PATH_TO_BLOCK:
            print("[MAIN] Planning path")
        
            path = planner.astar(
                start=world_map.get_robot_position()[0], 
                goal=goal.location,
                target_color=target_color,
                debug=0
            )
            world_map.add_to_planned_paths(path)
            if path is None:
                print("[Main] Goal found but no path found!")
                do_arena_scan()
                continue

            movements = world_map.convert_points_to_movements(path)
            rmc.set_path(movements)
            CURRENT_STATE = DRIVE_PATH_TO_BLOCK


        elif CURRENT_STATE == DRIVE_PATH_TO_BLOCK:
            print("[Main] Driving path")
            for step in movements:
                take_step_update_map()
                blocks = capture_and_process_image(cam=cam, wm=world_map)
            world_map.draw_map()
            CURRENT_STATE = COLLECT_BLOCK
            print("[Main] Path complete")


        elif CURRENT_STATE == COLLECT_BLOCK:
            print("[Main] Collecting block")
            collection_movements = []
            start_point = world_map.get_robot_position()[0]
            
            block_collected = False
            while not block_collected:
                blocks = capture_and_process_image(cam=cam, wm=world_map)
                correct_color_in_frame = False
                if len(blocks) == 0:
                    print("[MAIN] NO BLOCK DETECTED!")
                    rmc.rotate_by(30)
                for block in blocks:
                    if block.color == target_color:
                        target_block = block
                        correct_color_in_frame = True
                if not correct_color_in_frame:
                    print("[MAIN] Correct color block not found in frame")
                    rmc.rotate_by(30)
                    continue
                
                if target_block.bounding_origin[1] >= BLOCK_IN_GRIP_RANGE: # if bounding box is low in frame
                    if target_block.angle_to_robo < 10:
                        rmc.close_gripper()
                        block_collected = True
                        print(f"[MAIN] {target_block.color} block collected!")
                        try:
                            world_map.remove_block(goal)
                        except:
                            ValueError("[MAIN] Target block already removed.")
                        CURRENT_STATE = PLAN_PATH_TO_CONSTRUCTION
                        continue
                    else:
                        # Rotate to Block
                        goal_angle = target_block.angle_to_robo
                        rotation_angle = rmc.rotate_by(goal_angle)
                        collection_movements.append(
                            (
                                ("Angle", rotation_angle)
                            )
                        )
                        continue
                
                # Get block heading and orient to it
                goal_angle = target_block.angle_to_robo
                rotation_angle = rmc.rotate_by(goal_angle)
                forward_distance = rmc.forward(0.02)
                collection_movements.extend(
                    (
                        ("Angle", rotation_angle),
                        ("Distance", max(forward_distance))
                    )
                )
            # Add collection movements to world map, update robot position
            collection_path = world_map.convert_movements_to_points(start_point, collection_movements)
            world_map.add_to_driven_path(collection_path)
            world_map.draw_map()
            
            
        elif CURRENT_STATE == PLAN_PATH_TO_CONSTRUCTION:
            print("[MAIN] Delivering block")
            start = world_map.get_robot_position()[0]
            goal = world_map.construction_area_center
            # Remove any block from the world map that the robot is already in the keepout of
            
            world_map.remove_blocks_already_in_keepout(start)

            path = planner.astar(
                start=start, 
                goal=goal, 
                debug=0
            )
            if path is None:
                do_arena_scan(True)
            else:
                world_map.add_to_planned_paths(path)
                movements = world_map.convert_points_to_movements(path)
                rmc.set_path(movements)
                CURRENT_STATE = DRIVE_PATH_TO_CONSTRUCTION
            
            
        elif CURRENT_STATE == DRIVE_PATH_TO_CONSTRUCTION:
            print("[Main] Driving path to construction")
            for step in movements:
                take_step_update_map()
                capture_and_process_image(cam=cam, wm=world_map, got_that_thang_on_me=True)
            CURRENT_STATE = RELEASE_BLOCK
            
            
        elif CURRENT_STATE == RELEASE_BLOCK:
            print("[MAIN] Releasing block")
            rmc.open_gripper()
            time.sleep(0.5)
            
            start_point, heading = world_map.get_robot_position()
            realized_distance =  rmc.backward(0.1)
            realized_movements = [
                ("Angle", heading),
                ("Distance", realized_distance)
            ]
            realized_path = world_map.convert_movements_to_points(start_point, realized_movements)
            world_map.add_to_driven_path(realized_path)
            
            reorient_counter += 1
            print("[MAIN] Reorient counter: ", reorient_counter)
            if reorient_counter >= 3:
                CURRENT_STATE = REORIENT
                reorient_counter = 0
                continue
            
            CURRENT_STATE = GET_NEXT_GOAL


        elif CURRENT_STATE == SEARCH:
            print("[MAIN] Entered Search Mode")
            # rotate to the center of the map
            path = planner.astar(
                start=world_map.get_robot_position()[0], 
                goal=world_map.map_center,
                debug=0
            )
            if path is None:
                do_arena_scan()
                continue
            movements = world_map.convert_points_to_movements(path)
            rmc.set_path(movements)
            # make a move to the center of the map.  With every move, look for the goal block
            for _ in movements:
                take_step_update_map()
                blocks = capture_and_process_image(cam=cam, wm=world_map)
                target_block_found = False
                if len(blocks) != 0:
                    # loop through updated world map blocks looking for goal
                    for block in world_map.blocks:
                        if block.color == target_color:
                            goal = block
                            CURRENT_STATE = PLAN_PATH_TO_BLOCK
                            target_block_found = True
                            break  
                    if target_block_found:
                        break
            else:             
                # Now in the middle of the map, begin scanning
                for angle in range(0, 360, 45):
                    rmc.orient_to(angle)
                    blocks = capture_and_process_image(cam=cam, wm=world_map)
                    target_block_found = False
                    if len(blocks) != 0:
                        # loop through updated world map blocks looking for goal
                        for block in world_map.blocks:
                            if block.color == target_color:
                                # Plan a path to the goal
                                goal = block
                                CURRENT_STATE = PLAN_PATH_TO_BLOCK
                                target_block_found = True
                                break  
                        if target_block_found:
                            break

        elif CURRENT_STATE == REORIENT:
            print("[MAIN] Reorienting robot in world")
            rmc.orient_to(90)
            distance_y = ping.get_distance()/100 # convert cm to m
            print("[MAIN] Distance Y: ", distance_y)
            rmc.orient_to(180)
            distance_x = ping.get_distance()/100 # convert cm to m
            
            print("[MAIN debug] Heading: ", world_map.get_robot_position()[1])
            assumed_position = world_map.get_robot_position()[0]
            actual_position = (distance_x, world_map.bounds[1]-distance_y)
            difference = (
                actual_position[0] - assumed_position[0] ,
                actual_position[1] - assumed_position[1] 
            )
            
            world_map.robot_position = actual_position
            for block in world_map.get_blocks():
                block.location = (
                    block.location[0] + difference[0],
                    block.location[1] + difference[1]
                )
                
            print("[MAIN debug] Heading: ", world_map.get_robot_position()[1])
            
            world_map.draw_map()
            
            CURRENT_STATE = GET_NEXT_GOAL
            
            
        elif CURRENT_STATE == CLEANUP:
            
            blocks = world_map.get_blocks()
            
            if len(blocks) == 0:
                do_pickup_area_scan()
                blocks = world_map.get_blocks()
                if len(blocks) == 0:
                    CURRENT_STATE == END
                else:
                    continue
                
            else:
                goal = blocks[0]
                target_color = blocks[0].color
                CURRENT_STATE == PLAN_PATH_TO_BLOCK
                continue



    # State is now END
    odom.kill()
    imu.kill()
    world_map.kill()
    print("[Main] Course complete")
