import RPi.GPIO as gpio
import numpy as np

##### Initialize GPIO pins #####

# pin_number = 7   # left
pin_number = 12  # right

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(pin_number, gpio.IN, pull_up_down=gpio.PUD_UP)
    
def gameover():
    gpio.cleanup()
    

##### Main Code #####

init()

counter = np.uint64(0)
button = int(0)

while True:
    if int(gpio.input(pin_number)) != int(button):
        button = int(gpio.input(pin_number))
        counter += 1
        print(counter)
        
    if counter>= 100:
        gameover()
        print("Thanks for playing!")
        break