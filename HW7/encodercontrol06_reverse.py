import RPi.GPIO as gpio
import time
import numpy as np
import matplotlib.pyplot as plt

### Initialize GPIO pins ####

right_encoder = 12 # right
right_motor = 35  # right

left_encoder = 7 # left 
left_motor = 33  # left

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    
    gpio.setup(left_encoder, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(right_encoder, gpio.IN, pull_up_down=gpio.PUD_UP)
    

def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    gpio.cleanup()

def distance_to_ticks(dist):
    """ 
    Returns the number of encoder ticks that corresponds to an input distance
    Input is in meters
    Output is encoder ticks
    """
    rotations = (dist) * (1/(2*np.pi*0.0325))
    ticks = rotations * 20
    print("wheel rotations = ", rotations)
    print("Estimated Ticks = ", ticks)
    return  ticks

class PID_Control():
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.integral = 0
        
    def update(self, setpoint, measurement, mv):

        error = setpoint - measurement
        P = error * self.Kp
        self.integral = self.integral + error
        I  = self.integral * self.Ki
        mv = mv + P + I
        print("error: ", error, end="   ")
        print("mv: ", mv)
        return mv
        
#### Main code ####

init()
distance_counterBR = np.uint64(0)
control_counterBR = np.uint64(0)
distance_counterFL = np.uint64(0)
control_counterFL = np.uint64(0)

buttonBR = np.int(0)
buttonFL = np.int(0)

# initialize pwm signal to control motor
pwm1 = gpio.PWM(right_motor, 50) # RIGHT, 50Hz
pwm2 = gpio.PWM(left_motor, 50) # LEFT, 50Hz

val = 25 # val = duty cycle
pwm1.start(val)
pwm2.start(val)
time.sleep(0.1)

stateListFL = []
stateListBR = []


distance_in_ticks = distance_to_ticks(1.8288)

pid = PID_Control(Kp=2.5, Ki=0.3, Kd=0)

for i in range(0, 100_000):
    # print("counterBR: ", counterBR, "counterFL: ", counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
    stateListFL.append(gpio.input(left_encoder))
    stateListBR.append(gpio.input(right_encoder))
    
    if int(gpio.input(right_encoder)) != int(buttonBR):
        buttonBR = int(gpio.input(right_encoder))
        distance_counterBR += 1
        control_counterBR += 1
        print("counterFL: (", distance_counterFL, ", ", control_counterFL, ")", end="   ")
        print("counterBR: (", distance_counterBR, ", ", control_counterBR, ")")
        
    if int(gpio.input(left_encoder)) != int(buttonFL):
        buttonFL = int(gpio.input(left_encoder))
        distance_counterFL += 1
        control_counterFL += 1
        print("counterFL: (", distance_counterFL, ", ", control_counterFL, ")", end="   ")
        print("counterBR: (", distance_counterBR, ", ", control_counterBR, ")")
    
    # PID control loop
    if control_counterFL >= 10 or control_counterBR >= 10:
        val = pid.update(setpoint=control_counterFL, measurement=control_counterBR, mv=val)
        control_counterBR = 0
        control_counterFL = 0
        
    pwm1.ChangeDutyCycle(val)
    
    
    if distance_counterBR >= distance_in_ticks and distance_counterFL >= distance_in_ticks:
        print("here")
        gameover()
        break
    
    time.sleep(0.001)
    
    
fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)

fig.suptitle("Motor Encoder Analysis")
ax1.plot(stateListFL)
ax1.set_ylabel("Front Left Encoder")
ax2.plot(stateListBR)
ax2.set_ylabel("Back Right Encoder")

fig.savefig("MotorEncoderOutput.png")
