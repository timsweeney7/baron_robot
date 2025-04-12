import time
import cv2
from picamera2 import Picamera2
import libcamera
import numpy as np
from numpy import pi
import RPi.GPIO as gpio
import time


def determine_orientation(img, corners):
    """ 
    Determines the orientation of the green arrow
    img:  input image to draw on
    corners:  (5, 1, 2) shaped np array or corner points
    """

    # determine the line of best fit
    x = corners[:, 0, 0]
    y = corners[:, 0, 1]
    m,b = np.polyfit(x, y, 1)

    # draw the line
    x1 = -1
    y1 = int(m * x1 + b)
    x2 = 5000
    y2 = int(m * x2 + b)
    cv2.line(img=img, pt1=(x1, y1), pt2=(x2, y2), color=(0, 0, 255), thickness=3)

    # find the centroid of the points
    center = np.int0(np.mean(corners, axis=0))
    print(center)
    cv2.circle(img=img, center=center[0], radius=10, color=(255,0,0), thickness=-1)

    # Determine the angle of the line of best fit with the x axis
    x_crossing = (b*-1)/m
    shifted_x = center[0, 0] - x_crossing
    angle = np.arctan2(center[0,1], shifted_x)  # angle = atan2(y/x)
    print(angle)

    # Find the line orthogonal to the line of best fit that passes through the centroid
    _m = -1/m
    _b = (-1*_m*center[0,0] + center[0,1])
    x1 = -1
    y1 = int(_m * x1 + _b)
    x2 = 5000
    y2 = int(_m * x2 + _b)
    cv2.line(img=img, pt1=(x1, y1), pt2=(x2, y2), color=(255, 0, 0), thickness=3)

    # Determine which side of the orthogonal line has more points to determine direction
    below = 0
    above = 0
    for corner in corners:
        # determine if the corner point is above or below the orthogal line
        x = corner[0, 0]
        y = corner[0, 1]
        calculated_y =  _m * x + _b
        if y > calculated_y:
            above+=1
        else:
            below+=1

    if above>below:
        above = True
    else:
        above = False
    
    # determine orientation, while accounting for the coordinate frame of the camera
    if angle >= 0 and angle < pi/4:
        if above:
            orientation = "left"
        else:
            orientation = "right" 
    if angle >= pi/4 and angle < 3*pi/4:
        if above:
            orientation = "down"
        else:
            orientation = "up"
    else:
        if above:
            orientation = "right"
        else:
            orientation = "left"
    
    return orientation


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
# cv2.imwrite(filename="rgb_image.jpg", img=bgr_image)

# mask the image
lower_bound = np.array([71, 80, 97])  # Lower bound (Hue=70, min Sat & Val)
upper_bound = np.array([92, 255, 255])  # Upper bound (Hue=80, max Sat & Val)
mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

# blur the image
blur = cv2.blur(mask, (8, 8))

# write out the clear and blurred images
# cv2.imwrite(filename="clear_image.jpg", img=mask)
# cv2.imwrite(filename="blur_image.jpg", img=blur)

# detect the corners of the image and draw them
corners = cv2.goodFeaturesToTrack(image=blur, maxCorners=7, qualityLevel=0.05, minDistance=25)
corners = np.int0(corners)

for i in corners:
    x, y = i.ravel()
    cv2.circle(img=bgr_image, center=(x,y), radius=10, color=(0,0,255), thickness=-1)

orientation = determine_orientation(bgr_image, corners)
print(orientation)

# write out the image with the corners detected
cv2.imwrite(filename="corner_detect.jpg", img=bgr_image)