import time
from utilities.RobotMotorControl import roboMotorControl
from utilities.PingSensor import pingSensor
import cv2
from picamera2 import Picamera2
import libcamera


def key_input(event, rmc:roboMotorControl):
    
    print("Key: ", event)
    key_press = event
    tf = 1
    
    if key_press.lower()=='w':
        rmc.forward(tf)
    elif key_press.lower()=='s':
        rmc.backward(tf)
    elif key_press.lower()=='a':
        rmc.pivot_left(tf)
    elif key_press.lower()=='d':
        rmc.pivot_right(tf)
    elif key_press.lower()=='g':
        rmc.open_gripper()
    elif key_press.lower()=='h':
        rmc.close_gripper()
    elif key_press.lower()=='x':
        rmc.set_gripper_pwm(2)
        rmc.set_gripper_pwm(6.9)
    else:
        print("Invalid Keypress")
    

# create a motor control object
rmc = roboMotorControl()

# create a distance sensor object
ping = pingSensor()

# configure the camera
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size":(808,606)},
                                           transform=libcamera.Transform(hflip=1, vflip=1))
picam2.configure(camera_config=config)

# create a video writer object
writer = cv2.VideoWriter(filename='output2.avi',
                        fourcc=cv2.VideoWriter.fourcc(*'MJPG'),
                        fps=1,
                        frameSize=(808, 606))

# start the camera
picam2.start()
time.sleep(1)  # Allow camera to warm up

# create parameters for text writing
org1 = (30,40)
org2 = (60,40)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (0, 0, 255)
thickness = 2

count = 0

while True:
    
    if count >= 30:
        break
    else:
        count += 1
        
    # make next move
    key_press = input("Select driving mode: ")
    if key_press == 'p':
        break
    key_input(key_press, rmc)
    time.sleep(0.5)
    
    # determine distance from ping sensor
    dist = ping.get_distance()
    dist = round(dist, 2)
    
    # capture images for video 
    rgb_image = picam2.capture_array()
    bgr_image = cv2.cvtColor(rgb_image,cv2.COLOR_RGB2BGR)

    # Add text to the image
    cv2.putText(bgr_image, f"Distance: {dist} cm", org1, font, fontScale, color, thickness, cv2.LINE_AA)

    # write the processed image to video
    writer.write(bgr_image)
    

rmc.game_over()
ping.game_over()