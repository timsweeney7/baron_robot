import time
from imu import IMU
import pigpio
import numpy as np
from odometry import odometer


SERVO_GPIO = 16

FORWARD_LEFT = 6
BACKWARD_LEFT = 13
BACKWARD_RIGHT = 19
FORWARD_RIGHT = 26

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
        print("error: ", error)
        # print("mv: ", mv)
        return mv


class roboMotorControl():
    
    def __init__(self):
        self.pi = pigpio.pi()

        # wheels DC motor
        self.pi.set_mode(FORWARD_LEFT, pigpio.OUTPUT)
        self.pi.set_mode(BACKWARD_LEFT, pigpio.OUTPUT)
        self.pi.set_mode(BACKWARD_RIGHT, pigpio.OUTPUT)
        self.pi.set_mode(BACKWARD_LEFT, pigpio.OUTPUT)
        self.stop_motion()
        self.pi.set_PWM_range(FORWARD_RIGHT, 100)
        self.pi.set_PWM_range(BACKWARD_RIGHT, 100)
        self.pi.set_PWM_range(BACKWARD_LEFT, 100)
        self.pi.set_PWM_range(FORWARD_LEFT, 100)
        self.pi.set_PWM_frequency(FORWARD_RIGHT, frequency=50)
        self.pi.set_PWM_frequency(BACKWARD_LEFT, frequency=50)
        self.pi.set_PWM_frequency(BACKWARD_RIGHT, frequency=50)
        self.pi.set_PWM_frequency(FORWARD_LEFT, frequency=50)
       
        # gripper servo 
        self.pi.set_mode(SERVO_GPIO, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(SERVO_GPIO, 1000)
 
        # IMU for rotation calculations
        self.imu = IMU()
        
        # Create an odometer
        self.odom = odometer()
        
    def stop_motion(self):
        # set all pins low
        self.pi.write(FORWARD_LEFT, False)
        self.pi.write(BACKWARD_LEFT, False)
        self.pi.write(BACKWARD_RIGHT, False)
        self.pi.write(FORWARD_RIGHT, False)
        
    def forward(self, distance):
        """
        distance = meters
        """
        pid = PID_Control(Kp= -1.75, Ki= 0, Kd=0)
        
        distance_in_ticks = self.odom.distance_to_ticks(distance)
        DUTY_CYCLE = 59
        self.pi.set_PWM_dutycycle(FORWARD_RIGHT, DUTY_CYCLE)
        self.pi.set_PWM_dutycycle(FORWARD_LEFT, DUTY_CYCLE)
        new_duty_cycle = DUTY_CYCLE
        
        start_heading = self.imu.get_heading()
        print(f"Start heading: {start_heading}")
        print() 
        
        self.odom.reset()
        
        while True:

            current_heading = self.imu.get_heading()
            
            # use offsets to avoid wrap-around 0 to 360 problem
            if start_heading > 270:
                # add 360 to low readings
                if current_heading < 90:
                    current_heading += 360
            elif start_heading < 90:
                # sub 360 from high readings
                if current_heading > 270:
                    current_heading -= 360
                    
            print(f"Current heading: {current_heading}")
            
            left_ticks, right_ticks = self.odom.get_ticks()
            print(f"Left ticks: {left_ticks} \t Right ticks: {right_ticks} ")
            if left_ticks > distance_in_ticks or right_ticks > distance_in_ticks:
                print("here 1")
                break
            
            new_duty_cycle = pid.update(setpoint=start_heading, 
                                        measurement=current_heading, 
                                        mv=new_duty_cycle)
            
            print(f"New duty cycle: {new_duty_cycle}")
            self.pi.set_PWM_dutycycle(FORWARD_RIGHT, new_duty_cycle)
            delay_counter = 0
            print()
        
            time.sleep(0.2)

        self.stop_motion()
        time.sleep(0.4)
        return self.odom.get_distance()
        
        
    def backward(self, tf):
        # Left wheels
        self.pi.write(FORWARD_LEFT, False)
        self.pi.write(BACKWARD_LEFT, True)
        # right wheels
        self.pi.write(FORWARD_RIGHT, False)
        self.pi.write(BACKWARD_RIGHT, True)
        # Wait
        time.sleep(tf)
        # Send all pins 
        self.stop_motion()   
        
        
    def orient_to(self, angle_deg):
        """
        Rotates the robot to the given angle. 
        Automatically determines if it would be faster to rotate cw or ccw
        """
        heading = self.imu.get_heading()
        option_1 = angle_deg - heading
        if option_1 >= 0:
            option_2 = option_1 % -360
        else:
            option_2 = option_1 % 360
        
        if abs(option_1) <= abs(option_2):
            self.rotate_by(option_1)
        else:
            self.rotate_by(option_2)
        return self.imu.get_heading()
        
        
    def rotate_by(self, angle_deg):
        """
        Rotates robot by the given angle (degrees).
        Positive: clockwise
        Negative: counterclockwise
        """
        
        # if angle_deg > 0:
        #     angle_deg -= 4
        
        start_heading = self.imu.get_heading()  # 0–360
        target_heading = (start_heading + angle_deg) % 360
        
        FAST_DUTY_CYCLE = 80
        SLOW_DUTY_CYCLE = 50
        
        # Determine direction
        if angle_deg > 0:
            # initialize pwm signal to control motor
            self.pi.set_PWM_range(BACKWARD_RIGHT, 100)
            self.pi.set_PWM_range(FORWARD_LEFT, 100)
            self.pi.set_PWM_frequency(BACKWARD_RIGHT, frequency=100)
            self.pi.set_PWM_frequency(FORWARD_LEFT, frequency=100)
            self.pi.set_PWM_dutycycle(BACKWARD_RIGHT, FAST_DUTY_CYCLE)
            self.pi.set_PWM_dutycycle(FORWARD_LEFT, FAST_DUTY_CYCLE)
            while True:
                heading = self.imu.get_heading()
                if self._within_tolerance(current=heading, target=target_heading, tolerance=25):
                    self.pi.set_PWM_dutycycle(BACKWARD_RIGHT, SLOW_DUTY_CYCLE)
                    self.pi.set_PWM_dutycycle(FORWARD_LEFT, SLOW_DUTY_CYCLE)
                if self._has_passed_clockwise(start_heading, target_heading, heading):
                    break

        elif angle_deg < 0:
            # initialize pwm signal to control motor
            self.pi.set_PWM_range(BACKWARD_LEFT, 100)
            self.pi.set_PWM_range(FORWARD_RIGHT, 100)
            self.pi.set_PWM_frequency(FORWARD_RIGHT, frequency=100)
            self.pi.set_PWM_frequency(BACKWARD_LEFT, frequency=100)
            self.pi.set_PWM_dutycycle(FORWARD_RIGHT, FAST_DUTY_CYCLE)
            self.pi.set_PWM_dutycycle(BACKWARD_LEFT, FAST_DUTY_CYCLE)
            while True:
                heading = self.imu.get_heading()
                if self._within_tolerance(current=heading, target=target_heading, tolerance=25):
                    self.pi.set_PWM_dutycycle(FORWARD_RIGHT, SLOW_DUTY_CYCLE)
                    self.pi.set_PWM_dutycycle(BACKWARD_LEFT, SLOW_DUTY_CYCLE)
                if self._has_passed_counterclockwise(start_heading, target_heading, heading):
                    break

        self.stop_motion()
        time.sleep(0.3) # wait for robot to settle
        return self.imu.get_heading()
        
    @staticmethod
    def _has_passed_clockwise(start, target, current):
        """Checks if we passed the target heading going clockwise."""
        if start < target:
            return current >= target
        return current >= target and current < start

    @staticmethod
    def _has_passed_counterclockwise(start, target, current):
        """Checks if we passed the target heading going counterclockwise."""
        if start > target:
            return current <= target
        return current <= target and current > start
    
    @staticmethod
    def _within_tolerance(current, target, tolerance=10):
        # Compute smallest angular difference
        diff = abs((current - target + 180) % 360 - 180)
        return diff <= tolerance

    def open_gripper(self):
        # self.set_gripper_pwm(6.5)
        self.pi.set_servo_pulsewidth(SERVO_GPIO, 1300)
        
    def close_gripper(self):
        # self.set_gripper_pwm(2.8)
        self.pi.set_servo_pulsewidth(SERVO_GPIO, 600)
    
    def game_over(self):
        self.pi.set_PWM_dutycycle(SERVO_GPIO, 0)
        self.stop_motion()
        self.odom.kill()

        
if __name__ == "__main__":
    
    def key_input(event, rmc:roboMotorControl):
    
        print("Key: ", event)
        key_press = event
        
        try:    # check if input is an angle or a direction
            angle = int(key_press)
            rmc.orient_to(angle)
            print(f"New heading: {rmc.imu.get_heading()}")
            return
        except ValueError:
            pass
        
        tf = 1
        if key_press.lower()=='w':
            # rmc.forward(1.397)
            rmc.forward(2)
            time.sleep(0.4)
            left, right = rmc.odom.get_distance()
            print("Distance rolled - Left: ", left, "\tRight: ", right)
        elif key_press.lower()=='s':
            rmc.backward(tf)
        elif key_press.lower()=='a':
            rmc.rotate_by(-90)
        elif key_press.lower()=='d':
            rmc.rotate_by(90)
        elif key_press.lower()=='g':
            rmc.open_gripper()
        elif key_press.lower()=='h':
            rmc.close_gripper()
        elif key_press.lower()=='i':
            print("Heading: ", rmc.imu.get_heading())
        else:
            print("Invalid Keypress")
    
    rmc = roboMotorControl()
    while True:
        key_press = input("Select driving mode: ")
        if key_press == 'p':
            break
        key_input(key_press, rmc)
        print()

    rmc.game_over()