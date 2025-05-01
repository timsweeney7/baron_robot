import serial
import re
import threading
import queue


class IMU:

    def __init__(self):

        self.heading = 0

        # connect to the arduino nano
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)

        # ignore the first 10 prints from the IMU
        count = 0
        while True:
            if self.ser.in_waiting > 0:
                count += 1
                line = self.ser.readline()
                if count <= 10:
                    continue
                else:
                    break

        self._running = False
        self.lock = threading.Lock()
        self._T1 = threading.Thread(target=self.start)
        self._T1.start()

    def kill(self):
        self._running = False

    def start(self):

        print("[IMU] Started thread")
        self._running = True

        while self._running:
            while self.ser.in_waiting < 1:
                continue
            line = str(self.ser.readline())
            data = re.findall("\d+\.\d+", line)
            _heading = float(data[0])
            _heading = _heading * -1 % 360

            with self.lock:
                self.heading = _heading

    def get_heading(self):
        """Returns the current heading of the robot in degrees"""
        with self.lock:
            return self.heading


if __name__ == "__main__":

    from utilities.RobotMotorControl import RobotMotorControl
    from utilities.odometry import Odometer

    imu = IMU()
    odom = Odometer()
    rmc = RobotMotorControl(imu=imu, odom=odom)

    while True:

        rmc.rotate_by(180)
