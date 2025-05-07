import serial
import time

class ArduinoInterface:
    """Basic Python-Arduino interface for sending and receiving txt data Arduinos Serial Monitor"""
    def __init__(self):
        self.arduino = None
        self.connected = False


    def connect(self, arduino_port="COM3", baudrate=9600):
        try:
            self.arduino = serial.Serial(arduino_port, baudrate, timeout=1)
            self.connected = True
            time.sleep(2)
            print("Connected to Arduino on", arduino_port)
        except serial.SerialException:
            self.arduino = None
            self.connected = False
            print("Connection Failed")

    def disconnect(self):
        if self.connected:
            self.arduino.close()
            self.connected = False
            self.arduino = None

    def send_txt(self, str):
        if self.connected:
            self.arduino.write((str + "\n").encode())

    def send_and_receive(self, str):
        self.send_txt(str)
        start_time = time.time()  # Record the start time

        # Loop until response is received, or timeout occurs
        while True:
            if self.arduino.in_waiting:  # If data is available, read it
                response = self.arduino.readline().decode().strip()
                return response

            # Check if timeout (2 seconds) has been reached
            if time.time() - start_time > 2:
                print("Timeout: No response from Arduino")
                return ""  # Return an empty response

            time.sleep(0.1)