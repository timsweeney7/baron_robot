from picamera2 import Picamera2
import libcamera
import cv2 as cv
import time

RED_LB = [0, 80, 0]
RED_UB = [10, 255, 255]

GREEN_LB = [25, 80, 0]
GREEN_UB = [75, 255, 255]

CYAN_LB = [80, 80, 0]
CYAN_UB = [105, 255, 255]


class camera():

    def __init__(self):
        # configure the camera
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size":(800,606)},
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



if __name__ == "__main__":
    
    cam = camera()
    rgb_image = cam.capture_image()
    
    # mask the image
    mask = cv.inRange(hsv_image, self.lower_bound, self.upper_bound)
    contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
    
    # contour
    # bounding box