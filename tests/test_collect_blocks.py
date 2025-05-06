import cv2 as cv

from utilities.path_planner import WorldMap
from utilities.imu import IMU
from utilities.vision import Camera
from utilities.RobotMotorControl import RobotMotorControl
from utilities.odometry import Odometer



COLLECT_BLOCK = 0
CURRENT_STATE = COLLECT_BLOCK

odom = Odometer()
imu = IMU()
world_map = WorldMap()
cam = Camera(world_map)
rmc = RobotMotorControl(imu=imu, odometer=odom)

target_color = "red"

if CURRENT_STATE == COLLECT_BLOCK:
    print("[Main] Collecting block")
    collection_movements = []
    start_point = world_map.get_robot_position()[0]
    
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
            if block.color == target_color:
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
            try:
                world_map.remove_block(goal)
            except:
                ValueError("[MAIN] Target block already removed.")
            CURRENT_STATE = PLAN_PATH_TO_CONSTRUCTION
            continue
        # Get block heading and orient to it
        goal_angle = target_block.angle_to_robo
        rotation_angle = rmc.rotate_by(goal_angle)
        forward_distance = rmc.forward(0.02)
        collection_movements.append(
            (
                ("Angle", rotation_angle),
                ("Distance", forward_distance)
            )
        )
    # Add collection movements to world map, update robot position
    collection_path = world_map.convert_movements_to_points(start_point, collection_movements)
    world_map.add_to_driven_path(collection_path)
    # world_map.draw_path_on_map(collection_path, "blue", save_fig=True)
    world_map.draw_map()