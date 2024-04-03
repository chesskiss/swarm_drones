
import time
from tellopy import Tello

# Create Tello object
tello = Tello()

# Connect to Tello drone
tello.connect()

# Wait for the connection to be established
time.sleep(2)
'''
# Take off
tello.takeoff()

# Wait for a few seconds
time.sleep(2)
'''
# Land
tello.land()

# Disconnect from Tello drone
tello.quit()
