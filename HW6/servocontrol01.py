import time
import cv2
from picamera2 import Picamera2
import libcamera
from numpy import pi
import RPi.GPIO as gpio
import time


class roboMotorControl():
    
    def __init__(self):
        gpio.setmode(gpio.BOARD)
        # wheels DC motor
        gpio.setup(31, gpio.OUT)
        gpio.setup(33, gpio.OUT)
        gpio.setup(35, gpio.OUT)
        gpio.setup(37, gpio.OUT)
        # gripper servo
        gpio.setup(36, gpio.OUT)
        self.pwm = gpio.PWM(36, 50)
        self.pwm.start(3.0)

    def set_gripper_pwm(self, pwm_speed):
        # bound the input so that we don't break the motor
        if pwm_speed <= 2.8:
            pwm_speed = 2.8
        if pwm_speed >= 6.5:
            pwm_speed = 6.5
            
        self.pwm.ChangeDutyCycle(pwm_speed)
        return pwm_speed
    
    def game_over(self):
        self.pwm.stop()
        gpio.cleanup()  
    

# configure the camera
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size":(808,606)},
                                           transform=libcamera.Transform(hflip=1, vflip=1))
picam2.configure(camera_config=config)

# create a video writer object
writer = cv2.VideoWriter(filename='output.avi',
                        fourcc=cv2.VideoWriter.fourcc(*'MJPG'),
                        fps=1,
                        frameSize=(808, 606))

# start the camera
picam2.start()
time.sleep(1)  # Allow camera to warm up

# create parameters for text writing
org = (25,40)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (0, 0, 255)
thickness = 2

# create a motor control object
rmc = roboMotorControl()

step_size = (6.5-2.8)/10
pwm = rmc.set_gripper_pwm(2.8)


for i in range(10):
    pwm = round(pwm, 2)
    # wait for the gripper to get in position
    time.sleep(0.6)
    print(f"Step {i}.  PWM = {pwm}", end=" ")

    # capture an image
    # rgb_image = picam2.capture_array()[:,:,0:3]
    rgb_image = picam2.capture_array()
    
    bgr_image = cv2.cvtColor(rgb_image,cv2.COLOR_RGB2BGR)

    # Add text to the image
    cv2.putText(bgr_image, str(pwm)+"% Duty Cycle", org, font, fontScale, color, thickness, cv2.LINE_AA)
    # cv2.imwrite(filename=f"rgb_image_{i}.jpg", img=bgr_image)

    # write the processed image to video
    writer.write(bgr_image)
    
    # set the new gripper position
    pwm = rmc.set_gripper_pwm(pwm+step_size)

picam2.stop()
rmc.game_over()