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
        print("waiting low")
        pulse_start = time.time()

    while gpio.input(echo) == 1:
        print("waiting high")
        pulse_end = time.time()

    print(f"pulse start: {pulse_start}")
    print(f"pulse end: {pulse_end}")
    pulse_duration = pulse_end - pulse_start
    print(pulse_duration)

    # Convert time to distance
    distance = pulse_duration * 17150
    distance = round(distance,2)
    
    # Cleanup gpio pins & return distance estimate
    gpio.cleanup()
    return distance

print("Distance: ", distance(), " cm")


