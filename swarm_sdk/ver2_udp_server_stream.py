import cv2
import numpy as np
import socket

# Create a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Set up the IP address and port for the server to bind to
host = '0.0.0.0'  # Bind to all available network interfaces
port = 11111

# Bind the socket to the IP address and port
server_socket.bind((host, port))

print("UDP server is listening on {}:{}".format(host, port))

# Decode and display the first received frame
data, client_address = server_socket.recvfrom(65535)
frame = np.frombuffer(data, dtype=np.uint8)
print(f"Frame content: {frame}")

frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
print(f"Frame content: {frame}")


if frame is not None:
    cv2.imshow('Video Stream', frame)
    cv2.waitKey(0)

# Release the resources and close the socket
cv2.destroyAllWindows()
server_socket.close()
