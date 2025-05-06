import RPi.GPIO as gpio
import time


class pingSensor:

    def __init__(self):
        # Define pin allocations
        self.trig = 16
        self.echo = 18

        gpio.setmode(gpio.BOARD)
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)

        # ensure output has no value
        gpio.output(self.trig, False)
        time.sleep(0.5)

    def get_distance(self):
        # Generate trigger pulse
        gpio.output(self.trig, True)
        time.sleep(0.000_01)
        gpio.output(self.trig, False)

        # Generate echo time signal
        pulse_start = time.time()
        while gpio.input(self.echo) == 0:
            pulse_start = time.time()

        while gpio.input(self.echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        # print(pulse_duration, end="\t")

        # Convert time to distance
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        # print(distance)
        return distance

    def game_over(self):
        gpio.cleanup()


if __name__ == "__main__":
    ping = pingSensor()
    
    while True:
        distance = ping.get_distance()
        print(f"Distance: {distance} cm")
        input('...')