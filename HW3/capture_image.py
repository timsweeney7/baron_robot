#!/usr/bin/python3
import time
import cv2
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import numpy as np
import copy

BGR_RED = (0,0,255)
RGB_RED = (255,0,0)

picam2 = Picamera2()
picam2.start()
time.sleep(2)  # Allow camera to warm up
rgb_image = picam2.capture_array()
bgr_image = cv2.cvtColor(rgb_image,cv2.COLOR_RGB2BGR)
hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

# output the image
cv2.imwrite(filename="rgb_image.jpg", img=bgr_image)
cv2.imwrite(filename="hsv_image.jpg", img=hsv_image)

# Define lower and upper bounds for Hue (70-80), with full Saturation and Value ranges
lower_bound = np.array([60, 50, 50])  # Lower bound (Hue=70, min Sat & Val)
upper_bound = np.array([80, 255, 255])  # Upper bound (Hue=80, max Sat & Val)

# Create the mask
mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
# Output the mask image
cv2.imwrite(filename="mask.jpg", img=mask)
# Find the contours
contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)

# Choose the contour with the largest internal area
selected_contour = max(contours, key=cv2.contourArea)
(x_axis, y_axis), radius = cv2.minEnclosingCircle(points=selected_contour)
radius = int(radius)
circle_center = (int(x_axis), int(y_axis))

track_image = copy.deepcopy(bgr_image)
# draw the min enclosing circle
cv2.circle(track_image, center=circle_center, radius=radius, color=BGR_RED, thickness=2)


cv2.imwrite(filename="trackFrame.jpg", img=track_image)

picam2.stop()