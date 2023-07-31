import cv2
import time
import numpy as np
import socket
#import Drones.video_decoder.

# Create a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Set up the IP address and port for the server to bind to
host = '0.0.0.0'  # Bind to all available network interfaces
port = 11111

# Bind the socket to the IP address and port
server_socket.bind((host, port))

print("UDP server is listening on {}:{}".format(host, port))

# Decode and display the video stream
while True:
    time.sleep(3)
    data, client_address = server_socket.recvfrom(65535)
    print(f"Received data size: {len(data)}")

    print('checkpoint 1 before try')
    try:
        frame = np.frombuffer(data, dtype=np.uint8)
        np.save('frame_data.npy', frame)

        print('checkpoint 2 before ')
        print(f"Frame content: {frame}")

        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        print('checkpoint 1 after ')
        if frame is not None:
            print('checkpoint final ')
            cv2.imshow('Video Stream', frame)

    except Exception as e:
        print(f"OH NOOOOOO ooooooooooooooooooooooooooooooooo: {e}")

    # Press 'q' key to exit the video stream display
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the resources and close the socket
cv2.destroyAllWindows()
server_socket.close()
