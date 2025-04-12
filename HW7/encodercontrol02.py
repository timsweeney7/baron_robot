import RPi.GPIO as gpio
import time
import numpy as np

#### Initialize GPIO pins  #####

encoder_pin = 12 # right
wheel_motor_pin = 31  # right

# encoder_pin = 7 # left 
# wheel_motor_pin = 37  # left

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    
    gpio.setup(encoder_pin, gpio.IN, pull_up_down=gpio.PUD_UP)
    
def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    
    gpio.cleanup()

### Main code ####

init()

counter = np.uint64(0)
button = int(gpio.input(encoder_pin))

# Initialize pwm signal to control motor
pwm = gpio.PWM(wheel_motor_pin, 50)
val = 15
pwm.start(val)
time.sleep(0.1)

for i in range(0, 100_000):
    
    if int(gpio.input(encoder_pin)) != int(button):
        button = int(gpio.input(encoder_pin))
        counter += 1
        print("counter = ", counter, "GPIO state: ", gpio.input(encoder_pin))
        time.sleep(0.003)
        
    if counter >= 40:
        pwm.stop()
        gameover()
        print("Thanks for playing!")
        break
    
    time.sleep(0.01)
