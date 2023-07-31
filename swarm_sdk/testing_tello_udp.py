from tello_udp import *

address = 'udp://@0.0.0.0:11111'
drone = Tello()

stream = BackgroundFrameRead(drone, address)
stream.start()