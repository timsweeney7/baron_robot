import time
import pigpio
import numpy as np

from utilities.imu import IMU
from utilities.odometry import Odometer


SERVO_GPIO = 16

FORWARD_LEFT = 6
BACKWARD_LEFT = 13
BACKWARD_RIGHT = 19
FORWARD_RIGHT = 26


class PID_Control:

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0

    def update(self, setpoint, measurement, mv):

        error = setpoint - measurement
        P = error * self.Kp
        self.integral = self.integral + error
        I = self.integral * self.Ki
        mv = mv + P + I
        # print("[RMC] error: ", error)
        # print("mv: ", mv)
        return mv


class RobotMotorControl:

    def __init__(self, imu: IMU, odom: Odometer):
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
        self.imu = imu

        # Create an odometer
        self.odom = odom

        # Create a variable to store the path
        self.movements = []

    def stop_motion(self):
        # set all pins low
        self.pi.write(FORWARD_LEFT, False)
        self.pi.write(BACKWARD_LEFT, False)
        self.pi.write(BACKWARD_RIGHT, False)
        self.pi.write(FORWARD_RIGHT, False)

    def set_path(self, path):
        """
        Sets the path for the robot to follow.

        Returns a list of angles and disstances in alternating order, starting with the angle.
        For example: [angle1, distance1, angle2, distance2, ...]
        """
        self.movements = []

        if len(path) < 2:
            print("[RMC] Path is too short")
            return False
        base = path[0]
        inc = path[1]

        # heading = self.imu.get_heading()
        angle = np.degrees(np.arctan2(inc[1] - base[1], inc[0] - base[0]))
        print(f"Angle: {angle}")
        self.movements.append(("Angle", angle))
        distance = np.sqrt((inc[0] - base[0]) ** 2 + (inc[1] - base[1]) ** 2)
        print(f"Distance: {distance}")
        # loop over the path converting to angles and distances
        # if the next point has the same heading as the previous point,
        # then we can just add the distance
        for i in range(2, len(path)):
            check = path[i]
            angle_check = np.degrees(np.arctan2(check[1] - inc[1], check[0] - inc[0]))
            print(f"Angle check: {angle_check}")
            if abs(angle_check - angle) < 0.01:
                print("[RMC] Same angle as before")
                distance += np.sqrt((check[0] - inc[0]) ** 2 + (check[1] - inc[1]) ** 2)
                print(f"Distance: {distance}")
            else:
                self.movements.append(("Distance", distance))
                print("[RMC] New angle")
                self.movements.append(("Angle", angle_check))
                distance = np.sqrt((check[0] - inc[0]) ** 2 + (check[1] - inc[1]) ** 2)

            print(f"Distance: {distance}")
            angle = angle_check
            inc = check

        self.movements.append(("Distance", distance))
        
        print("[RMC] Movements set")
        for i in self.movements:
            print(f"\t{i}")

    def drive_path(self):
        """
        Drives the robot along the loaded path.
        """
        for step in self.movements:
            if step[0] == "Angle":
                angle = step[1]
                print(f"[RMC] Goal Angle: {angle}")
                self.orient_to(angle)

            elif step[0] == "Distance":
                distance = step[1]
                print(f"[RMC] Goal Distance: {distance}")
                traveled = self.forward(distance)
                print(f"[RMC] Distance traveled: {traveled}")

        # stop motion
        self.stop_motion()
        time.sleep(0.4)

    def forward(self, distance):
        """
        distance = meters
        """
        pid = PID_Control(Kp=1.75, Ki=0, Kd=0)

        distance_in_ticks = self.odom.distance_to_ticks(distance)
        DUTY_CYCLE = 59
        self.pi.set_PWM_dutycycle(FORWARD_RIGHT, DUTY_CYCLE)
        self.pi.set_PWM_dutycycle(FORWARD_LEFT, DUTY_CYCLE)
        new_duty_cycle = DUTY_CYCLE

        start_heading = self.imu.get_heading()
        # print(f"[RMC][Forward] Start heading: {start_heading}")
        # print()

        self.odom.reset()

        near_goal_switch = True

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

            # print(f"[RMC][Forward] Current heading: {current_heading}")

            left_ticks, right_ticks = self.odom.get_ticks()
            if left_ticks > (distance_in_ticks - 30) or right_ticks > (
                distance_in_ticks - 30
            ):
                if near_goal_switch:
                    near_goal_switch = False
                    self.pi.set_PWM_dutycycle(FORWARD_LEFT, DUTY_CYCLE * 0.6)
                    self.pi.set_PWM_dutycycle(FORWARD_RIGHT, new_duty_cycle * 0.6)

            if left_ticks > distance_in_ticks or right_ticks > distance_in_ticks:
                break

            new_duty_cycle = pid.update(
                setpoint=start_heading, measurement=current_heading, mv=new_duty_cycle
            )

            # print(f"[RMC][Forward] New duty cycle: {new_duty_cycle}")
            self.pi.set_PWM_dutycycle(FORWARD_RIGHT, new_duty_cycle)
            # print()

            time.sleep(0.2)

        self.stop_motion()
        time.sleep(0.4)
        return self.odom.get_distance()

    def backward(self, tf):

        # TODO: Add PID control and distance measurement

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
            heading = self.rotate_by(option_1)
        else:
            heading = self.rotate_by(option_2)
        return heading

    def rotate_by(self, angle_deg):
        """
        Rotates robot by the given angle (degrees).
        Positive: counterclockwise
        Negative: clockwise
        """

        DEBUG = True

        start_heading = self.imu.get_heading()  # 0–360
        target_heading = (start_heading + angle_deg) % 360
        if DEBUG:
            print(
                f"[RMC][rotate_by]   degrees: {angle_deg} Start Heading: {start_heading}",
                end=" ",
            )
            print(f"\t Target Heading: {target_heading}")

        # FAST_DUTY_CYCLE = 80
        # SLOW_DUTY_CYCLE = 50

        FAST_DUTY_CYCLE = 50
        SLOW_DUTY_CYCLE = 35

        # Ignore very small adjustments
        if -0.5 <= angle_deg <= 0.5:
            pass
        # Determine direction to turn
        elif angle_deg >= 0:
            # print(f"[DEBUG] Rotating Left")
            # initialize pwm signal to control motor
            self.pi.set_PWM_range(FORWARD_RIGHT, 100)
            self.pi.set_PWM_range(BACKWARD_LEFT, 100)
            self.pi.set_PWM_frequency(FORWARD_RIGHT, frequency=100)
            self.pi.set_PWM_frequency(BACKWARD_LEFT, frequency=100)
            self.pi.set_PWM_dutycycle(FORWARD_RIGHT, FAST_DUTY_CYCLE)
            self.pi.set_PWM_dutycycle(BACKWARD_LEFT, FAST_DUTY_CYCLE)
            while True:
                heading = self.imu.get_heading()
                # if DEBUG == True:
                #     print(f"[RMC][rotate_by ccw] Heading: {heading}")
                if self._within_tolerance(
                    current=heading, target=target_heading, tolerance=25
                ):
                    # print("[DEBUG] Within Tolerance")
                    self.pi.set_PWM_dutycycle(FORWARD_RIGHT, SLOW_DUTY_CYCLE)
                    self.pi.set_PWM_dutycycle(BACKWARD_LEFT, SLOW_DUTY_CYCLE)
                if self._has_passed_counterclockwise(
                    start_heading, target_heading, heading
                ):
                    break
                time.sleep(0.01)

        elif angle_deg < 0:
            # print(f"[DEBUG] Rotating Right")
            # initialize pwm signal to control motor
            self.pi.set_PWM_range(FORWARD_LEFT, 100)
            self.pi.set_PWM_range(BACKWARD_RIGHT, 100)
            self.pi.set_PWM_frequency(BACKWARD_RIGHT, frequency=100)
            self.pi.set_PWM_frequency(FORWARD_LEFT, frequency=100)
            self.pi.set_PWM_dutycycle(BACKWARD_RIGHT, FAST_DUTY_CYCLE)
            self.pi.set_PWM_dutycycle(FORWARD_LEFT, FAST_DUTY_CYCLE)
            while True:
                heading = self.imu.get_heading()
                # if DEBUG == True:
                #     print(f"[RMC][rotate_by cw] Heading: {heading}")
                if self._within_tolerance(
                    current=heading, target=target_heading, tolerance=25
                ):
                    self.pi.set_PWM_dutycycle(BACKWARD_RIGHT, SLOW_DUTY_CYCLE)
                    self.pi.set_PWM_dutycycle(FORWARD_LEFT, SLOW_DUTY_CYCLE)
                if self._has_passed_clockwise(start_heading, target_heading, heading):
                    break
                time.sleep(0.01)

        self.stop_motion()
        time.sleep(0.3)  # wait for robot to settle
        # if DEBUG:
        #     print(f"[RMC] Ending Heading: {self.imu.get_heading()}")
        return self.imu.get_heading()

    @staticmethod
    def _has_passed_counterclockwise(start, target, current):
        """Checks if we passed the target heading going counterclockwise."""
        # print(f"[DEBUG] start: {start}, target:{target}, current:{current}")
        if start <= target:
            return current >= target or current < start
        return current >= target and current < start

    @staticmethod
    def _has_passed_clockwise(start, target, current):
        """Checks if we passed the target heading going clockwise."""
        # print(f"[DEBUG] start: {start}, target:{target}, current:{current}")
        if start >= target:
            return current <= target or current > start
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

    from utilities.imu import IMU
    from utilities.odometry import Odometer

    def key_input(event, rmc: RobotMotorControl):

        print("Key: ", event)
        key_press = event

        try:  # check if input is an angle or a direction
            angle = float(key_press)
            rmc.orient_to(angle)
            print(f"New heading: {rmc.imu.get_heading()}")
            return
        except ValueError:
            pass

        tf = 1
        if key_press.lower() == "w":
            # rmc.forward(1.397)
            rmc.forward(2)
            time.sleep(0.4)
            left, right = rmc.odom.get_distance()
            print("Distance rolled - Left: ", left, "\tRight: ", right)
        elif key_press.lower() == "s":
            rmc.backward(tf)
        elif key_press.lower() == "a":
            rmc.rotate_by(90)
        elif key_press.lower() == "d":
            rmc.rotate_by(-90)
        elif key_press.lower() == "g":
            rmc.open_gripper()
        elif key_press.lower() == "h":
            rmc.close_gripper()
        elif key_press.lower() == "i":
            print("Heading: ", rmc.imu.get_heading())
        else:
            print("Invalid Keypress")

    imu = IMU()
    odom = Odometer()
    rmc = RobotMotorControl(imu=imu, odom=odom)
    while True:
        key_press = input("Select driving mode: ")
        if key_press == "q":
            break
        key_input(key_press, rmc)
        print()

    rmc.game_over()
