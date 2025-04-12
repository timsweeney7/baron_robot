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

    # find the centroid of the points
    center = np.int0(np.mean(corners, axis=0))[0]
    print(center)
    cv2.circle(img=img, center=center, radius=10, color=(255,0,0), thickness=-1)

    # find the 3 furthest corners from the center.  The closest of these is the point
    distances = np.linalg.norm(corners[:,0,:]-center, axis=1)
    top3_distances = np.argsort(distances)[-3:]
    point_idx = top3_distances[0]
    point = corners[point_idx][0]
    # get the vector from center to the point
    vector = (point[0]-center[0], point[1]-center[1])
    # get the angle of the vector
    angle = np.arctan2(vector[1], vector[0]) # angle = arctan(y/x)
    print(angle)

    # convert from angle to direction
    if angle >= -1*pi and angle < -3*pi/4:
        orientation = "left"
    elif angle >= -3*pi/4 and angle < -1*pi/4:
        orientation = "up"
    elif angle >= -1*pi/4 and angle < pi/4:
        orientation = "right"
    elif angle >= pi/4 and angle < 3*pi/4:
        orientation = "down"
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

# write the orientation of the arrow on the image
org = (150,300)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 6
color = (0, 0, 255)
thickness = 15

# Add text to the image
cv2.putText(bgr_image, orientation, org, font, fontScale, color, thickness, cv2.LINE_AA)

# write out the image with the corners detected
cv2.imwrite(filename="corner_detect.jpg", img=bgr_image)