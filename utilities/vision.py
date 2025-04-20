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


# MOM AND DADS ---------------
GREEN_LB = np.array([40, 50, 50])
GREEN_UB = np.array([75, 255, 255])


# ON THE PORCH ----------------
RED_LS_LB = np.array([0, 195, 160])
RED_LS_UB = np.array([25, 255, 255])
RED_HS_LB = np.array([165, 195, 160])
RED_HS_UB = np.array([180, 255, 255])

GREEN_LB = np.array([40, 100, 0])
GREEN_UB = np.array([80, 255, 255])

CYAN_LB = np.array([100, 140, 100])
CYAN_UB = np.array([110, 255, 255])


# ------------------------------
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 606
DEG_PER_PIXEL = 0.0775
RGB_BLACK = (0, 0, 0)
RGB_GREEN = (0, 255, 0)
RGB_CYAN = (0, 255, 255)
RGB_RED = (255, 0, 0)


class Block():
    """
    class for recording properties of blocks identified in a frame
    """
    def __init__(self, color:str, distance_from_robo:float, angle_to_robo:float, knocked_over=False, bounding_height=None):
        """
        @param color: The color of the block identified
        @param distance_from_robo: The distance the robot is from the block in meters
        @param anlge_to_robo: The angle in degrees of the block to the robot
        """
        self.color = color
        self.distance_from_robo = distance_from_robo
        self.angle_to_robo = angle_to_robo
        self.bounding_height = bounding_height
        self.knocked_over = knocked_over


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
        distance = block.bounding_area * ALPHA
        return distance
    
    
    def find_blocks(self, frame) -> Tuple[np.array, List[Block]]:
        """
        Searches the image for red, green, and cyan blocks.  Adds a bounding box around each one
        and marks the center
        
        @param frame: image to be searched in RGB format
        
        @return boxed_frame: rgb image with boxes added
        @return blocks: list of blocks identified
        """
        found_blocks = []
        hsv_frame = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
        # Draw a line down the middle of the image
        output_frame = cv.line(frame, pt1=(int(IMAGE_WIDTH/2), 0), pt2=(int(IMAGE_WIDTH/2), IMAGE_HEIGHT), color=(0,0,0))
        
        # Find CYAN and GREEN blocks
        for color in [(GREEN_LB, GREEN_UB, RGB_GREEN), (CYAN_LB, CYAN_UB, RGB_CYAN)]:
            # create mask 
            mask = cv.inRange(hsv_frame, color[0], color[1])
            # find contours
            contours, _ = cv.findContours(mask, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_NONE)
            # non_zero_coords = cv2.findNonZero(mask)
            # Sort contours by area in descending order
            contours = sorted(contours, key=cv.contourArea, reverse=True)
            if len(contours) == 0:
                continue
            # Place a bounding box around the largest contour
            x, y, w, h = cv.boundingRect(contours[0])
            # Calculate the center of the bounding box
            center = (int((x + x + w)/2), int((y + y + h)/2))
            # Draw the bounding box and center in the image.  
            output_frame = cv.rectangle(output_frame, (x, y), (x + w, y + h), color[2], 2)
            output_frame = cv.circle(output_frame, center=center, radius=3, color=color[2], thickness=-1)
            # Find the angle of the robot to the block
            angle = (center[0] - IMAGE_WIDTH/2) * DEG_PER_PIXEL
            # dist = calculate_block_distance(area)
            # Determine if the block is knocked over based on aspect ratio
            if w > h:
                knocked_over = True
            else:
                knocked_over = False
            # Create a block object to return
            out_block = Block(color='GREEN', distance_from_robo=0, angle_to_robo=angle, knocked_over=knocked_over, bounding_height=h)
            found_blocks.append(out_block)
            
        # Repeat the process for RED blocks.  
        # RED wraps from 0 to 180 in HSV colorspace so we need two loops
        mask = np.zeros(np.shape(frame)[:2]).astype(np.uint8)  # 2d shape
        for color in [(RED_LS_LB, RED_LS_UB),(RED_HS_LB, RED_HS_UB)]:
            # create mask
            partial_mask = cv.inRange(hsv_frame, color[0], color[1])
            mask = cv.bitwise_or(mask, partial_mask)
        
        # find contours
        contours, _ = cv.findContours(mask, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_NONE)
        # non_zero_coords = cv2.findNonZero(mask)
        # Sort contours by area in descending order
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        if len(contours) != 0:
            # Place a bounding box around the largest contour
            x, y, w, h = cv.boundingRect(contours[0])
            # Calculate the center of the bounding box
            center = (int((x + x + w)/2), int((y + y + h)/2))
            # Draw the bounding box and center in the image.  
            output_frame = cv.rectangle(output_frame, (x, y), (x + w, y + h), RGB_RED, 2)
            output_frame = cv.circle(output_frame, center=center, radius=3, color=RGB_RED, thickness=-1)
            # Find the angle of the robot to the block
            angle = (center[0] - IMAGE_WIDTH/2) * DEG_PER_PIXEL
            # dist = calculate_block_distance(area)
            # Determine if the block is knocked over based on aspect ratio
            if w > h:
                knocked_over = True
            else:
                knocked_over = False
            # Create a block object to return
            out_block = Block(color='GREEN', distance_from_robo=0, angle_to_robo=angle, knocked_over=knocked_over, bounding_height=h)
            found_blocks.append(out_block)
        
        return output_frame, found_blocks

        
        
        
        
if __name__ == "__main__":
    
    cam = Camera()
    rgb_image = cam.capture_image()
    cv.imwrite("rgb_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
    boxed_image, blocks = cam.find_blocks(rgb_image)
    
    cv.imwrite("boxedImage.jpg", cv.cvtColor(boxed_image, cv.COLOR_RGB2BGR))
    print(f"Image shape: {np.shape(boxed_image)}")
    
