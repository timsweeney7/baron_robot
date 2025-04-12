import RPi.GPIO as gpio
import time
from imu import IMU
import pigpio


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
        self.pwm = gpio.PWM(36, 50) # 50Hz
        self.pwm.start(3.0) # 3% duty cycle
 
        # IMU for rotation calculations
        self.imu = IMU()
        
    def stop_motion(self):
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
        self.stop_motion()
        
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
        self.stop_motion()   
        
    def pivot_left_time(self, tf):
        # Left wheels
        gpio.output(31, True)
        gpio.output(33, False)
        # right wheels
        gpio.output(35, True)
        gpio.output(37, False)
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
        
        
    def rotate_by(self, angle_deg):
        """
        Rotates robot by the given angle (degrees).
        Positive: clockwise
        Negative: counterclockwise
        """
        start_heading = self.imu.get_heading()  # 0–360
        target_heading = (start_heading + angle_deg) % 360
        DUTY_CYCLE = 50
        
        # Determine direction
        if angle_deg > 0:
            # initialize pwm signal to control motor
            LEFT_MOTOR = 31  
            RIGHT_MOTOR = 35  
            pwm1 = gpio.PWM(RIGHT_MOTOR, 50) # 50Hz
            pwm2 = gpio.PWM(LEFT_MOTOR, 50)  # 50Hz
            pwm1.start(DUTY_CYCLE)
            pwm2.start(DUTY_CYCLE)
            while True:
                heading = self.imu.get_heading()
                if self._has_passed_clockwise(start_heading, target_heading, heading):
                    break

        elif angle_deg < 0:
            # initialize pwm signal to control motor
            LEFT_MOTOR = 33 
            RIGHT_MOTOR = 37 
            pwm1 = gpio.PWM(RIGHT_MOTOR, 50) # 50Hz
            pwm2 = gpio.PWM(LEFT_MOTOR, 50)  # 50Hz
            pwm1.start(DUTY_CYCLE)
            pwm2.start(DUTY_CYCLE)
            while True:
                heading = self.imu.get_heading()
                if self._has_passed_counterclockwise(start_heading, target_heading, heading):
                    break

        self.stop_motion()

    @staticmethod
    def _has_passed_clockwise(start, target, current):
        """Checks if we passed the target heading going clockwise."""
        if start < target:
            return current >= target
        return current >= target or current < start

    @staticmethod
    def _has_passed_counterclockwise(start, target, current):
        """Checks if we passed the target heading going counterclockwise."""
        if start > target:
            return current <= target
        return current <= target or current > start

    def open_gripper(self):
        self.set_gripper_pwm(6.5)
        
    def close_gripper(self):
        self.set_gripper_pwm(2.8)
        
    def set_gripper_pwm(self, pwm_speed):
        # bound the input so that we don't break the motor
        if pwm_speed < 2.8:
            pwm_speed = 2.8
            print("PWM Speed attempted to be set out of bounds - low!")
        if pwm_speed > 6.5:
            pwm_speed = 6.5
            print("PWM Speed attempted to be set out of bounds - high!")
            
        self.pwm.ChangeDutyCycle(pwm_speed)
        return pwm_speed
    
    def game_over(self):
        self.pwm.stop()
        gpio.cleanup()  
        
        
if __name__ == "__main__":
    
    def key_input(event, rmc:roboMotorControl):
    
        print("Key: ", event)
        key_press = event
        
        try:
            angle = int(key_press)
            rmc.orient_to(angle)
            print(f"New heading: {rmc.imu.get_heading()}")
            return
        except ValueError:
            pass
        
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
        elif key_press.lower()=='i':
            rmc.imu.get_heading()
        else:
            print("Invalid Keypress")
    
rmc = roboMotorControl()
while True:
    key_press = input("Select driving mode: ")
    if key_press == 'p':
        break
    key_input(key_press, rmc)
    print()

rmc.pwm.stop()
gpio.cleanup()