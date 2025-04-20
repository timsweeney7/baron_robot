import time

from utilities.RobotMotorControl import roboMotorControl
from utilities.vision import DEG_PER_PIXEL, IMAGE_WIDTH
from utilities.vision import Camera

import cv2 as cv


if __name__ == "__main__":

    rmc = roboMotorControl()
    cam = Camera()
    
    while True:
        rgb_image = cam.capture_image()
        boxed_image, blocks = cam.find_blocks(frame=rgb_image)
        cv.imwrite("boxedImageStart.jpg", cv.cvtColor(boxed_image, cv.COLOR_RGB2BGR))
        
        if blocks != None:
            rotation_angle = blocks[0].angle_to_robo
            print(f"[HW9] Rotation Angle: {rotation_angle}")
            rmc.rotate_by(rotation_angle)
            
            rgb_image = cam.capture_image()
            boxed_image, center = cam.find_blocks(frame=rgb_image)
            cv.imwrite("boxedImageEnd.jpg", cv.cvtColor(boxed_image, cv.COLOR_RGB2BGR))
        
            print()
        time.sleep(1)