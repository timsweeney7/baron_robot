#!/usr/bin/python3

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import time
from datetime import datetime
import matplotlib.pyplot as plt
import cv2
import numpy as np
from pprint import *


# Define lower and upper bounds for Hue (70-80), with full Saturation and Value ranges
lower_bound = np.array([60, 50, 50])  # Lower bound (Hue=70, min Sat & Val)
upper_bound = np.array([80, 255, 255])  # Upper bound (Hue=80, max Sat & Val)

BGR_RED = (0,0,255)
RGB_RED = (255,0,0)
        
picam2 = Picamera2()

writer = cv2.VideoWriter(filename='output.h264',
                        fourcc=cv2.VideoWriter.fourcc(*'h264'),
                        fps=100/30,
                        frameSize=(640, 480))
video_array = np.zeros((480, 640, 3, 100))
f = open('hw3data.txt','w')

picam2.start()
time.sleep(1)  # Allow camera to warm up


for i in range(100):
    # start the timer
    start = time.perf_counter()
    
    # capture the image, convert to HSV color space
    rgb_image_array = picam2.capture_array()[:,:,0:3]
    bgr_image_array = cv2.cvtColor(rgb_image_array, cv2.COLOR_RGB2BGR)
    hsv_image = cv2.cvtColor(bgr_image_array, cv2.COLOR_BGR2HSV)

    try:
        # find the contour within the color bounds with the largest area
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        selected_contour =  max(contours, key=cv2.contourArea)
        (x_axis, y_axis), radius = cv2.minEnclosingCircle(points=selected_contour)
        radius = int(radius)
        circle_center = (int(x_axis), int(y_axis))
        cv2.circle(bgr_image_array, center=circle_center, radius=radius, color=BGR_RED, thickness=2)
        video_array[:,:,:,i] = bgr_image_array
    except:
        continue

    # write the processed image to video
    writer.write(bgr_image_array)
    
    # stop the timer and record processing time
    end = time.perf_counter()
    time_diff = end - start
    outstring = str(str(time_diff) + '\n')
    f.write(outstring)
    

f.close()
picam2.stop()

