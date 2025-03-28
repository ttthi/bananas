import socket
import struct
import threading
import time

class Receiver:
    """Class for receiving data from the drone"""
    def __init__(self, tcp_port):
        self.tcp_port = tcp_port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Action that is executed when receiving data
        self.action = lambda text: print(text)

        # Allow reusing the address
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Set up the TCP server
        self.server_socket.bind(("0.0.0.0", self.tcp_port))
        self.server_socket.listen(5)
        print(f"Server listening on port {self.tcp_port}")

        self.running = False
        self.connection = None
        self.tcp_receive_thread = None

    def start(self):
        """Function for starting the receiver"""
        self.running = True

        # Create and start a Thread for running the receive_loop()' funtion
        self.tcp_receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.tcp_receive_thread.start()

    def stop(self):
        """Function for stopping the receiver"""
        self.running = False
        try:
            if self.connection:
                self.connection.close()
            self.server_socket.close()
        except Exception as e:
            print("Error closing sockets:", e)
        print("Receiver stopped.")

    def receive_loop(self):
        """Function for continiously listening for connections, and receiving data"""
        while self.running:
            try:
                if self.connection is None:
                    print("Waiting for connection...")
                    self.connection, addr = self.server_socket.accept()
                    print(f"Accepted connection from {addr}")

                # Read the first 4 bytes to determine the message length
                size = b""
                while len(size) < 4 and self.running:
                    packet = self.connection.recv(4 - len(size))
                    if not packet:
                        break
                    size += packet
                if not size:
                    raise ConnectionError("Connection lost while receiving data")
                msg_length = struct.unpack("<I", size)[0]

                # Receive the data in chunks
                data = b""
                while len(data) < msg_length:
                    packet = self.connection.recv(256)
                    if not packet:
                        raise ConnectionError("Connection lost while receiving data")
                    data += packet

                self.action(data.decode("utf-8"))
            except Exception as e:
                print("TCP receive error:", e)
                # Close connection and prepare to accept a new one
                if self.connection:
                    self.connection.close()
                self.connection = None
                # Wait before trying to accept a new connection
                time.sleep(1)

    def _recv_all(self, n):
        data = b""
        while len(data) < n and self.running:
            packet = self.connection.recv(n - len(data))
            if not packet:
                break
            data += packet
        return data

    def handle_tcp_data(self, data):
        print("[BananasRobot] Received TCP data:", data)

    def setAction(self, action):
        self.action = action