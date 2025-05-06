from jetbot import Robot
from SCSCtrl import TTLServo
from SensorInterface import ArduinoInterface
from Receiver import Receiver
import time

class BananasRobot(Robot):
    def __init__(self):

        super().__init__()

        self.power_left = 0.2    # Default power to run the left tracks, between 0-1, manually measured
        self.power_right = 0.23 # Default power to run the right tracks, between 0-1, manually measured
        self.speed = 3  # Manually measured velocity (cm/s) for the default power
        self.angspeed = 0   # Manually measured angular velocity (deg/s) for the default power

        #

        # Arm offsets, calculated manually to calibrate the arm
        self.azimuth_offset = 70
        self.elevator1_offset = -15
        self.elevator2_offset = -79
        self.elevator3_offset = -27

        # Open an interface to the drone
        self.receiver = Receiver(5001)
        self.receiver.start()
        self.receiver.setAction(self.inputHandler)

        # Open interface to the arduino
        self.arduino = ArduinoInterface()
        self.arduino.connect("/dev/ttyUSB0")

    def inputHandler(self, input):
        print(f"Received Command {input}")
        cmd = ""
        values_part = ""
        try:
            if ':' in input:
                cmd, values_part = input.split(":", 1)
            else:
                cmd = input
        except ValueError:
            print("Incorrect input")

        if cmd == "set":
            try:
                angles = [int(val) for val in values_part.split(",") if val]
                self.setArm(angles)
            except ValueError:
                print("Incorrect values")
        elif cmd == "open":
            print("opening")
            self.open()
        elif cmd == "close":
            print("Opening")
            self.close()
        elif cmd == "forward":
            try:
                dst = int(values_part)
                self.f(dst)
            except ValueError:
                print("Incorrect values")
        elif cmd == "backward":
            try:
                dst = int(values_part)
                self.b(dst)
            except ValueError:
                print("Incorrect values")

    def exit(self):
        self.arduino.disconnect()
        self.receiver.stop()

    def setspeed(self, s):
        # Helper function to calibrate the speed
        self.speed = s

    def setanglespeed(self, s):
        # Helper function to calibrate the speed
        self.angspeed = s

    def rt(self, t):
        # Helper function to calibrate the speed
        self.set_motors(self.power_left, -self.power_right)
        time.sleep(t)
        self.stop()

    def ft(self, t):
        # Helper function to calibrate the speed
        self.set_motors(self.power_left, self.power_right)
        time.sleep(t)
        self.stop()

    def f(self, distance):
        t = distance / self.speed
        self.set_motors(self.power_left, self.power_right)
        time.sleep(t)
        self.stop()

    def b(self, distance):
        t = distance / self.speed
        self.set_motors(-self.power_left, -self.power_right)
        time.sleep(t)
        self.stop()

    def r(self, angle):
        t = angle / self.speed
        self.set_motors(self.power_left, -self.power_right)
        time.sleep(t)
        self.stop()

    def l(self, angle):
        t = angle / self.speed
        self.set_motors(-self.power_left, self.power_right)
        time.sleep(t)
        self.stop()

    def fs(self, steps):
        self.set_motors(self.power_left, self.power_right)
        self.arduino.send_and_receive(f"countdown:{steps}", timeout=100)
        self.stop()

    def bs(self, steps):
        self.set_motors(-self.power_left, -self.power_right)
        self.arduino.send_and_receive(f"countdown:{steps}", timeout=100)
        self.stop()

    def set_azimuth(self, ang):
        TTLServo.servoAngleCtrl(1, ang + self.azimuth_offset, 1, 50)

    def set_elevator1(self, ang):
        TTLServo.servoAngleCtrl(2, ang + self.elevator1_offset, 1, 50)

    def set_elevator2(self, ang):
        TTLServo.servoAngleCtrl(3, -ang + self.elevator2_offset, 1, 50)

    def set_elevator3(self, ang):
        TTLServo.servoAngleCtrl(5, ang + self.elevator3_offset, 1, 50)

    def setArm(self, angles):
        self.set_azimuth(angles[0])
        self.set_elevator1(angles[1])
        self.set_elevator2(angles[2])
        self.set_elevator3(angles[3])

    def close(self):
        self.arduino.send_txt("close")

    def open(self):
        self.arduino.send_txt("open")