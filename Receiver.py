import socket
import struct

def receive_data(port):
    # Create a TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("0.0.0.0", port))
    server_socket.listen(1)
    print(f"Server listening on port {port}")

    # Accept a connection
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    # Read the first 4 bytes to determine the message length
    raw_size = conn.recv(4)
    if not raw_size:
        print("No data received")
        return None
    data_size = struct.unpack(">I", raw_size)[0]

    # Receive the data in chunks
    data = b""
    while len(data) < data_size:
        packet = conn.recv(4096)
        if not packet:
            break
        data += packet

    conn.close()
    server_socket.close()
    return data


if __name__ == "__main__":
    port = 5001
    data = receive_data(port)
    if data:
        print(f"Received: {data}")
        # Process or save your point cloud or other heavy data here