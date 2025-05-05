import socket
import struct
import threading
import time

class Sender:
    """Class for maintaining connection and sending data from the drone to the robot"""
    def __init__(self, target_ip, target_port):
        self.ip = target_ip
        self.port = target_port
        self.tcp_socket = None
        self.tcp_connected = False
        self.running = False

        self.connection_thread = None

    def start(self):
        """Function for starting the connection"""
        self.running = True

        # Create and start a Thread for running the 'maintain_connection()' funtion
        self.connection_thread = threading.Thread(target=self.maintain_connection, daemon=True)
        self.connection_thread.start()

    def stop(self):
        """Function for ending the connection"""
        self.running = False
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except Exception as e:
                print("Error closing the TCP socket:", e)
        print("Connection stopped")

    def maintain_connection(self):
        """Function for maintaining the connection to the robot"""
        while self.running:
            if not self.tcp_connected:
                # If not connected, attempt connection
                self.attempt_connection()
                # After an attempt wait a moment before checking again
                time.sleep(1)

    def attempt_connection(self):
        """Function for establishing a connection to the robot"""
        try:
            print(f"Attempting to connect to {self.ip}:{self.port}...")
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.connect((self.ip, self.port))
            self.tcp_connected = True
            print(f"Connected at {self.ip}:{self.port}")
        except Exception as e:
            print(f"Connection failed: {e}")
            self.tcp_connected = False

    def send_tcp_data(self, data):
        """Function sending str data to the robot"""

        # Don't try sending if there is no connection
        if not self.tcp_connected:
            print("TCP connection is not established.")
            return

        # Encode the input 'data' to the utf-8 format
        data = data.encode('utf-8')
        message = struct.pack("<I", len(data)) + data
        try:
            self.tcp_socket.sendall(message)
            print(f" Sent TCP data: {len(data)} bytes")
        except Exception as e:
            print(" Error sending TCP data:", e)
            self.tcp_connected = False
            try:
                self.tcp_socket.close()
            except Exception:
                pass