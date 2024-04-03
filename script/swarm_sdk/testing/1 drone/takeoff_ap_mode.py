import time
import socket

# Tello drone's IP address and port
tello_ip = '192.168.10.1	'
tello_port = 8889

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a local port (optional)
local_port = 8889
sock.bind(('', local_port))

# Command to send to the Tello drone
command = 'command'

# Send the command to the Tello drone
sock.sendto(command.encode(), (tello_ip, tello_port))

# Wait for the response
response, address = sock.recvfrom(1024)

# Print the response
print(response.decode())

# Takeoff command
takeoff_command = 'takeoff'

# Send the takeoff command
sock.sendto(takeoff_command.encode(), (tello_ip, tello_port))

# Wait for the response
response, address = sock.recvfrom(1024)

# Print the response
print(response.decode())

# Wait for a few seconds
time.sleep(2)
