import time
import math
from pymavlink import mavutil

# Connect to the vehicle
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the heartbeat
connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (connection.target_system, connection.target_component))

def set_yaw(yaw_angle, yaw_rate, direction, is_relative):
    """
    Sends a MAVLink command to set the yaw angle.
    
    Args:
    yaw_angle: Desired yaw angle in degrees.
    yaw_rate: Yaw rate in degrees per second.
    direction: 1 for clockwise, -1 for counterclockwise.
    is_relative: 1 if the angle is relative to current heading, 0 if absolute.
    """
    connection.mav.command_long_send(
        connection.target_system,    
        connection.target_component, 
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0, 
        yaw_angle,    
        yaw_rate,     
        direction,    
        is_relative,  
        0, 0, 0       
    )

def get_current_heading():
    """
    Returns the current heading of the vehicle.
    """
    message = connection.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    if message is None:
        print("Timeout or no message received")
        return None
    return math.degrees(message.yaw)

def turnLeft():
    # Get initial heading
    initial_heading = get_current_heading()
    if initial_heading is None:
        print("Unable to get initial heading. Exiting.")
        connection.close()
        exit()

    # Set yaw command to turn 10 degree counterclockwise (left)
    set_yaw(10, 10, -1, 1)

    # Wait for the vehicle to turn
    while True:
        current_heading = get_current_heading()
        if current_heading is None:
            print("Unable to get current heading. Exiting.")
            break

        print(f"Current heading: {current_heading:.2f}")

        # Check if the drone has turned by approximately 1 degree
        if abs(current_heading - (initial_heading - 1)) < 1:
            print("Turned by 10 degree left.")
            break

        time.sleep(1)

# Close the connection
connection.close()
