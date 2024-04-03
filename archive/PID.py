from time import sleep
import tellopy

# Define constants
ALTITUDE_SETPOINT = 100  # Desired altitude in cm
KP = 0.6  # Proportional gain
KI = 0.1  # Integral gain
KD = 0.3  # Derivative gain
DT = 0.1  # Time step

# Define variables
last_error = 0
integral = 0


def pid_controller(current_altitude):
    global last_error, integral

    # Calculate error
    error = ALTITUDE_SETPOINT - current_altitude

    # Calculate proportional term
    p_term = KP * error

    # Calculate integral term
    integral += error * DT
    i_term = KI * integral

    # Calculate derivative term
    d_term = KD * (error - last_error) / DT
    last_error = error

    # Calculate total output
    output = p_term + i_term + d_term

    # Limit output to range [0, 100]
    output = max(min(output, 100), 0)

    return output


def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)


def test():
    drone = tellopy.Tello()
    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

        drone.connect()
        #drone.wait_for_connection(60.0)
        drone.takeoff()
        sleep(5)

        while True:
            altitude = drone.get_height()  # Get current altitude
            throttle = pid_controller(altitude)  # Calculate throttle output
            drone.up(int(throttle))  # Adjust altitude
            sleep(DT)

    except Exception as ex:
        print(ex)

    finally:
        drone.land()
        drone.quit()


if __name__ == '__main__':
    test()
