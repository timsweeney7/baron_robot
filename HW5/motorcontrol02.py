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
        
    def gameover(self):
        # set all pins low
        gpio.output(31, False)
        gpio.output(33, False)
        gpio.output(35, False)
        gpio.output(37, False)
        
    def forward(self, tf):
        # Left wheels
        gpio.output(31, True)
        gpio.output(33, False)
        # right wheels
        gpio.output(35, False)
        gpio.output(37, True)
        # Wait
        time.sleep(tf)
        # Send all pins 
        self.gameover()
        
    def backward(self, tf):
        # Left wheels
        gpio.output(31, False)
        gpio.output(33, True)
        # right wheels
        gpio.output(35, True)
        gpio.output(37, False)
        # Wait
        time.sleep(tf)
        # Send all pins 
        self.gameover()   
        
    def pivot_left(self, tf):
        # Left wheels
        gpio.output(31, True)
        gpio.output(33, False)
        # right wheels
        gpio.output(35, True)
        gpio.output(37, False)
        # Wait
        time.sleep(tf)
        # Send all pins 
        self.gameover()  
        
    def pivot_right(self, tf):
        # Left wheels
        gpio.output(31, False)
        gpio.output(33, True)
        # right wheels
        gpio.output(35, False)
        gpio.output(37, True)
        # Wait
        time.sleep(tf)
        # Send all pins 
        self.gameover() 
        
    def open_gripper(self):
        self.pwm.ChangeDutyCycle(6.5)
        
    def close_gripper(self):
        self.pwm.ChangeDutyCycle(2.8)


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
    else:
        print("Invalid Keypress")
    
rmc = roboMotorControl()
while True:
    key_press = input("Select driving mode: ")
    if key_press == 'p':
        break
    key_input(key_press, rmc)

rmc.pwm.stop()
gpio.cleanup()