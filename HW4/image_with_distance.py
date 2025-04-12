import time
import cv2
from picamera2 import Picamera2
import libcamera
import numpy as np
import RPi.GPIO as gpio
import time

# Define pin allocations
trig = 16
echo = 18


def distance():
    
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)

    # ensure output has no value
    gpio.output(trig, False)
    time.sleep(0.5)

    # Generate trigger pulse
    gpio.output(trig, True)
    time.sleep(0.000_01)
    gpio.output(trig, False)

    # Generate echo time signal
    pulse_start = time.time()
    while gpio.input(echo) == 0:
        pulse_start = time.time()

    while gpio.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    print(pulse_duration, end="\t")

    # Convert time to distance
    distance = pulse_duration * 17150
    distance = round(distance,2)
    print(distance)
    
    # Cleanup gpio pins & return distance estimate
    gpio.cleanup()
    return distance


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

# measure the distance to the object
ranges = [distance() for i in range(10)]
avg_range = np.mean(ranges)
print(avg_range)

# write the distance on the image
text = f"Distance: {avg_range:.3f}"
org = (150,150)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 2
color = (0, 0, 255)
thickness = 5

# Add text to the image
cv2.putText(bgr_image, text, org, font, fontScale, color, thickness, cv2.LINE_AA)

# write out image
cv2.imwrite(filename="rgb_image.jpg", img=bgr_image)