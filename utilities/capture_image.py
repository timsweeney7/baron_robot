#!/usr/bin/python3
import time
import cv2
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import numpy as np
import copy
import libcamera

BGR_RED = (0,0,255)
RGB_RED = (255,0,0)

# configure the camera
picam2 = Picamera2()
# config = picam2.create_still_configuration(main={"size":(808,606)},
#                                            transform=libcamera.Transform(hflip=1, vflip=1))
config = picam2.create_still_configuration(main={"size":(800,606)},
                                           transform=libcamera.Transform(hflip=1, vflip=1))
picam2.align_configuration(config)
picam2.configure(camera_config=config)

picam2.start()
time.sleep(2)  # Allow camera to warm up

rgb_image = picam2.capture_array()
bgr_image = cv2.cvtColor(rgb_image,cv2.COLOR_RGB2BGR)
hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

# output the image
cv2.imwrite(filename="rgb_image.jpg", img=bgr_image)
cv2.imwrite(filename="hsv_image.jpg", img=hsv_image)

picam2.stop()