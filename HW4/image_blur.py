import time
import cv2
from picamera2 import Picamera2
import libcamera
import numpy as np
import RPi.GPIO as gpio
import time


# configure the camera
picam2 = Picamera2()
config = picam2.create_still_configuration(transform=libcamera.Transform(hflip=1, vflip=1))
picam2.configure(camera_config=config)

# start the camera
picam2.start()
time.sleep(2)  # Allow camera to warm up

# capture an image
rgb_image = picam2.capture_array()
bgr_image = cv2.cvtColor(rgb_image,cv2.COLOR_RGB2BGR)
hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

# mask the image
lower_bound = np.array([71, 80, 97])  # Lower bound (Hue=70, min Sat & Val)
upper_bound = np.array([100, 255, 255])  # Upper bound (Hue=80, max Sat & Val)
mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

# blur the image
blur = cv2.blur(mask, (50, 50))

# write out clear and blurred image
cv2.imwrite(filename="clear_image.jpg", img=mask)
cv2.imwrite(filename="blur_image.jpg", img=blur)