from picamera2 import Picamera2
import libcamera
import cv2 as cv
import time
import numpy as np
from typing import Tuple, List


# ARENA -------------
RED_LB = np.array([0, 80, 0])
RED_UB = np.array([10, 255, 255])

GREEN_LB = np.array([25, 80, 0])
GREEN_UB = np.array([75, 255, 255])

CYAN_LB = np.array([80, 80, 0])
CYAN_UB = np.array([105, 255, 255])

# HOME ----------------
RED_LB = np.array([0, 200, 105])
RED_UB = np.array([13, 255, 255])

GREEN_LB = np.array([60, 50, 50])
GREEN_UB = np.array([91, 255, 255])


IMAGE_WIDTH = 800
IMAGE_HEIGHT = 606
DEG_PER_PIXEL = 0.0775


class Block():
    """
    class for recording properties of blocks identified in a frame
    """
    def __init__(self, color:str, distance_from_robo:float, angle_to_robo:float, bounding_area=None):
        """
        @param color: The color of the block identified
        @param distance_from_robo: The distance the robot is from the block in meters
        @param anlge_to_robo: The angle in degrees of the block to the robot
        """
        self.color = color
        self.distance_from_robo = distance_from_robo
        self.angle_to_robo = angle_to_robo
        self.bounding_area = bounding_area


class Camera():

    def __init__(self):
        # configure the camera
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size":(808,606)},
                                                transform=libcamera.Transform(hflip=1, vflip=1))
        self.picam2.align_configuration(config)
        self.picam2.configure(camera_config=config)

        self.picam2.start()
        time.sleep(1)  # Allow camera to warm u
    
    
    def capture_image(self):
        """
        Captures an image in RGB format and returns it as a numpy array
        """
        rgb_image = self.picam2.capture_array()
        return rgb_image
    
    
    def calculate_block_distance(self, block: Block):
        """
        Calculates the distance a block is away from the robot based on the area of its bouding box
        @param block: instance of Block class to find distance of
        """
        ALPHA = 2
        distance = bounding_area * ALPHA
        return distance
    
    
    def find_blocks(self, frame) -> Tuple[np.array, List[block]]:
        """
        Searches the image for red, green, and cyan blocks.  Adds a bounding box around each one
        and marks the center
        
        @param frame: image to be searched in RGB format
        
        @return boxed_frame: rgb image with boxes added
        @return blocks: list of blocks identified
        """
        hsv_frame = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
        # create mask 
        mask = cv.inRange(hsv_frame, GREEN_LB, GREEN_UB)
        # find contours
        contours, _ = cv.findContours(mask, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_NONE)
        # Sort contours by area in descending order
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        if len(contours) == 0:
            return frame, None
        coords = cv.findNonZero(mask)
        # Place a bounding box around the largest contour
        x, y, w, h = cv.boundingRect(contours[0])
        # Calculate the center of the bounding box
        center = (int((x + x + w)/2), int((y + y + h)/2))
        # Draw the bounding box and center in the image.  Draw a line down the middle of the image
        boxed_frame = cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        boxed_frame = cv.circle(boxed_frame, center=center, radius=3, color=(0,255,0), thickness=-1)
        boxed_frame = cv.line(boxed_frame, pt1=(int(IMAGE_WIDTH/2), 0), pt2=(int(IMAGE_WIDTH/2), IMAGE_HEIGHT), color=(0,0,0))
        # Find the angle of the robot to the block
        angle = (center[0] - IMAGE_WIDTH/2) * DEG_PER_PIXEL
        # Find the area of the bounding box
        area = (h) * (w) 
        # dist = calculate_block_distance(area)
        # Create a block object to return
        out_block = block(color='GREEN', distance_from_robo=0, angle_to_robo=angle, bounding_area=area)
        return boxed_frame, [out_block]

        
        
        
        
if __name__ == "__main__":
    
    cam = camera()
    rgb_image = cam.capture_image()
    boxed_image, center = cam.find_blocks(rgb_image)
    
    cv.imwrite("boxedImage.jpg", cv.cvtColor(boxed_image, cv.COLOR_RGB2BGR))
    print(f"Image shape: {np.shape(boxed_image)}")
    print(f"Distance from center line: {center[0] - IMAGE_WIDTH/2}")
    
    # cv.imwrite("rgb_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
    
