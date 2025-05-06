from picamera2 import Picamera2
import libcamera
import cv2 as cv
import time
import numpy as np
from typing import Tuple, List
import time

from utilities.path_planner import WorldMap
from utilities.block import Block


# ARENA -------------
# RED_LB = np.array([0, 80, 0])
# RED_UB = np.array([10, 255, 255])

# GREEN_LB = np.array([25, 80, 0])
# GREEN_UB = np.array([75, 255, 255])

# CYAN_LB = np.array([80, 80, 0])
# CYAN_UB = np.array([105, 255, 255])


# HOME ----------------
RED_LS_LB = np.array([0, 220, 100])
RED_LS_UB = np.array([15, 255, 255])
RED_HS_LB = np.array([165, 150, 105])
RED_HS_UB = np.array([180, 255, 255])

GREEN_LB = np.array([40, 60, 110])
GREEN_UB = np.array([80, 255, 255])

CYAN_LB = np.array([100, 140, 100])
CYAN_UB = np.array([110, 255, 255])


# ON THE PORCH ----------------
# RED_LS_LB = np.array([0, 180, 100])
# RED_LS_UB = np.array([15, 255, 255])
# RED_HS_LB = np.array([165, 180, 150])
# RED_HS_UB = np.array([180, 255, 255])

# GREEN_LB = np.array([40, 100, 0])
# GREEN_UB = np.array([80, 255, 255])

# CYAN_LB = np.array([100, 140, 100])
# CYAN_UB = np.array([110, 255, 255])


# ------------------------------
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 606
DEG_PER_PIXEL = 0.0775
RGB_BLACK = (0, 0, 0)
RGB_GREEN = (0, 255, 0)
RGB_CYAN = (0, 255, 255)
RGB_RED = (255, 0, 0)
GRAND_CHALLENGE_ROW_CROP = 150
# For distance calculation
A = 30.2322
B = 5.2283


class Camera:

    def __init__(self, world_map: WorldMap):
        """
        Configures the camera
        @param world_map: the world map the robot is operating in
        """
        self.world_map = world_map

        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(
            main={"size": (808, 606)}, transform=libcamera.Transform(hflip=1, vflip=1)
        )
        self.picam2.align_configuration(config)
        self.picam2.configure(camera_config=config)

        self.picam2.start()
        time.sleep(1)  # Allow camera to warm u

    def capture_image(self) -> Tuple[np.array, dict]:
        """
        Captures an image in RGB format and returns it as a numpy array
        @return rgb_image: image captured
        @return metadata: data associated with the image ['world_pos', 'orientation']
        """
        rgb_image = self.picam2.capture_array()
        world_pos, heading = self.world_map.get_robot_position()
        current_time = time.time()
        metadata = {"position": world_pos, "heading_deg": heading, "time": current_time}
        return rgb_image, metadata

    def crop_image(self, frame, row):
        """
        Crops the image to keep FOV within the grand challenge arena
        @param frame: (np.array) the frame to crop
        @param row: the row of the image to begin the crop.  All row LESS THAN the input row will be removed
        """
        output_frame = frame[row:]
        return output_frame

    def calculate_block_distance(self, bounding_height):
        """
        Calculates the distance a block is away from the robot based on the height of its bouding box
        @param block: instance of Block class to find distance of
        @return: Distance of block in meters
        """
        distance = A / (bounding_height - B)
        return distance

    def find_blocks(self, frame, metadata, strict=True) -> Tuple[np.array, List[Block]]:
        """
        Searches the image for red, green, and cyan blocks.  Adds a bounding box around each one
        and marks the center.  Determines block position in world frame

        @param frame: image to be searched in RGB format

        @return boxed_frame: rgb image with boxes added
        @return blocks: list of blocks identified
        """
        # frame = cv.blur(frame, (5, 5))
        found_blocks = []
        frame = self.crop_image(frame, GRAND_CHALLENGE_ROW_CROP)
        hsv_frame = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
        # Draw a line down the middle of the image
        output_frame = cv.line(
            frame,
            pt1=(int(IMAGE_WIDTH / 2), 0),
            pt2=(int(IMAGE_WIDTH / 2), IMAGE_HEIGHT),
            color=(0, 0, 0),
        )

        for i, color in enumerate([
            (GREEN_LB, GREEN_UB, RGB_GREEN, "GREEN"),
            (CYAN_LB, CYAN_UB, RGB_CYAN, "CYAN"),
            (RED_LS_LB, RED_LS_UB, RGB_RED, "RED")
        ]):
            # handle special case for red blocks
            if i == 2:
                mask = np.zeros(np.shape(frame)[:2]).astype(np.uint8)  # 2d shape
                for red_hsv in [(RED_LS_LB, RED_LS_UB), (RED_HS_LB, RED_HS_UB)]:
                    partial_mask = cv.inRange(hsv_frame, red_hsv[0], red_hsv[1])
                    mask = cv.bitwise_or(mask, partial_mask)
                cv.imwrite("mask_debug.jpg", mask)
            else:
                # create mask
                mask = cv.inRange(hsv_frame, color[0], color[1])
                cv.imwrite("mask_debug.jpg", mask)
                
            # find contours
            contours, _ = cv.findContours(
                mask, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_NONE
            )
            # non_zero_coords = cv2.findNonZero(mask)
            # Sort contours by area in descending order
            contours = sorted(contours, key=cv.contourArea, reverse=True)
            if len(contours) == 0:
                continue
            # Place a bounding box around the largest contour
            x, y, w, h = cv.boundingRect(contours[0])
            
            

            if h < 12:
                continue
            # if strict:
            #     if  h/w < 1.1 or h/w > 1.6:  # vertical block 
            #         continue
            
            
            
            # Calculate the center of the bounding box
            center = (int((x + x + w) / 2), int((y + y + h) / 2))
            # Draw the bounding box and center in the image.
            output_frame = cv.rectangle(
                output_frame, (x, y), (x + w, y + h), color[2], 2
            )
            output_frame = cv.circle(
                output_frame, center=center, radius=3, color=color[2], thickness=-1
            )
            # Find the angle of the robot to the block
            angle = -1 * (center[0] - IMAGE_WIDTH / 2) * DEG_PER_PIXEL
            dist = self.calculate_block_distance(h)
            # Determine location locally
            x_block = dist * np.cos(np.deg2rad(angle))
            y_block = dist * np.sin(np.deg2rad(angle))
            # Determine location in world frame
            location = translate_to_world_frame(
                (x_block, y_block), metadata["heading_deg"], metadata["position"]
            )
            # Determine if the block is knocked over based on aspect ratio
            if w > h:
                knocked_over = True
            else:
                knocked_over = False
            # Create a block object to return
            out_block = Block(
                color=color[3],
                location=location,
                knocked_over=knocked_over,
                bounding_height=h,
                angle_to_robo=angle,
                bounding_origin=(x, y),
            )
            found_blocks.append(out_block)

        return output_frame, found_blocks


def translate_to_world_frame(point, robo_heading, robo_pos):
    
    px, py = point
    rx, ry = robo_pos
    theta = np.deg2rad(robo_heading)
    
    T = np.array(
        [
            [np.cos(theta), -1 * np.sin(theta), rx],
            [np.sin(theta), np.cos(theta), ry],
            [0, 0, 1],
        ]
    )
    point_in_robo_frame = np.array([[px], [py], [1]])
    vector = T @ point_in_robo_frame
    return vector[0, 0], vector[1, 0]


if __name__ == "__main__":
    from utilities.path_planner import WorldMap
    from utilities.imu import IMU
    imu = IMU()
    wm = WorldMap(1, 1, imu)
    cam = Camera(wm)
    
    metadata = {"position": (0,0), "heading_deg": (45.0), "time": time.time()}
    rgb_image, metadata = cam.capture_image()
    # rgb_image = cv.imread("rgb_image.jpg", cv.IMREAD_COLOR)
    # rgb_image = cv.cvtColor(rgb_image, cv.COLOR_BGR2RGB)
    cv.imwrite("rgb_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
    boxed_image, blocks = cam.find_blocks(rgb_image, metadata)
    cv.imwrite("boxed_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
    # mask = np.zeros(np.shape(rgb_image)[:2]).astype(np.uint8)  # 2d shape
    # for red_hsv in [(RED_LS_LB, RED_LS_UB), (RED_HS_LB, RED_HS_UB)]:
    #     partial_mask = cv.inRange(rgb_image, red_hsv[0], red_hsv[1])
    #     mask = cv.bitwise_or(mask, partial_mask)
    # cv.imwrite("mask.jpg", mask)
    # corners = cv.goodFeaturesToTrack(image=mask, maxCorners=7, qualityLevel=0.01, minDistance=25)
    # corners = np.int0(corners)

    # for i in corners:
    #     x, y = i.ravel()
    #     cv.circle(img=rgb_image, center=(x,y), radius=4, color=(0,0,255), thickness=-1)

    #     cv.imwrite("corner_Image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
