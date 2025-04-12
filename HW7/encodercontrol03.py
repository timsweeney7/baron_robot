import RPi.GPIO as gpio
import time
import numpy as np
import matplotlib.pyplot as plt

#### Initialize GPIO pins ####

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
    
def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    
    gpio.cleanup()
    
    
##### Main code #####

init()
counterBR = np.uint64(0)
counterFL = np.uint64(0)

buttonBR = np.int(0)
buttonFL = np.int(0)

# initialize pwm signal to control motor
pwm = gpio.PWM(37, 50) # left
val = 16
pwm.start(val)
time.sleep(0.1)

stateListFL = []
stateListBR = []

for i in range(0, 200_000):
    print("counterBR: ", counterBR, "counterFL: ", counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
    stateListFL.append(gpio.input(7))
    stateListBR.append(gpio.input(12))
    
    if int(gpio.input(12)) != int(buttonBR):
        buttonBR = int(gpio.input(12))
        counterBR += 1 
        
    if int(gpio.input(7) != int(buttonFL)):
        buttonFL = int(gpio.input(7))
        counterFL += 1
        
    if counterFL >= 40:
        pwm.stop()
        gameover()
        print("Thanks for playing!")
        break
    
    time.sleep(0.01)
    
    
fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
ax1.plot(stateListBR)
ax1.set_ylabel('Back Right State')
ax2.plot(stateListFL)
ax2.set_ylabel('Front Left State')
fig.suptitle("Motor Encoder Analysis")
fig.savefig('MotorEncoderOutput.png')
