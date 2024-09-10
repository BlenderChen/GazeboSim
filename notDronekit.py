from pymavlink import mavutil
import threading 
import time
import math

def set_yaw(connection ,yaw_angle, yaw_rate, direction, is_relative):
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

def get_current_heading(connection):
    """
    Returns the current heading of the vehicle.
    """
    message = connection.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    if message is None:
        print("Timeout or no message received")
        return None
    return math.degrees(message.yaw)

def get_location_metres(original_lat, original_lon, dNorth, dEast):
    """
    Returns a new latitude/longitude `LocationGlobal` object offset by 
    dNorth and dEast meters from the original latitude/longitude.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth (meters)
    
    # Calculate new latitude
    new_latitude = original_lat + (dNorth / earth_radius) * (180 / math.pi)
    
    # Calculate new longitude
    new_longitude = original_lon + (dEast / (earth_radius * math.cos(math.pi * original_lat / 180))) * (180 / math.pi)
    
    return new_latitude, new_longitude

def get_distance_metres(lat1, lon1, lat2, lon2):
    """
    Returns the ground distance in meters between two `LocationGlobal` objects.
    """
    earth_radius = 6378137.0  # Radius of the Earth in meters
    
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + \
        math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = earth_radius * c
    
    return distance

# def connect_vehicle(connection_string):
#     """
#     Connects to the vehicle using MAVLink.
#     """
#     master = mavutil.mavlink_connection(connection_string)
#     master.wait_heartbeat()
#     print("Connected to vehicle")
#     return master

def connect_vehicle(connection_string):
    """
    Connects to the vehicle using MAVLink.
    """
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Connected to vehicle")
    master.set_mode('GUIDED')
    print("Mode set to GUIDED")
    return master

# def arm_and_takeoff(master, target_altitude):
#     """
#     Arms the vehicle and flies to target_altitude using pymavlink.
#     """
#     print("Basic pre-arm checks")
#     # while not master.armed():
#     #     print("Waiting for arming...")
#     #     time.sleep(1)

#     print("Arming motors")
#     master.set_mode('GUIDED')
#     master.armed=True
#     master.arducopter_arm()
    

#     while not master.armed():
#         print("Waiting for arming...")
#         time.sleep(1)

#     print("Taking off!")
#     master.mav.command_long_send(master.target_system, master.target_component,
#                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude)
    
#     while True:
#         msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#         current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
#         print(f"Altitude: {current_altitude}")

#         if current_altitude >= target_altitude * 0.95:
#             print("Reached target altitude")
#             break
#         time.sleep(1)

def arm_and_takeoff(master, target_altitude):
    """
    Arms the vehicle and flies to target_altitude using pymavlink.
    """
    print("Basic pre-arm checks")

    print("Arming motors...")
    master.mav.command_long_send(master.target_system, master.target_component,
                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    print("Waiting for arming...")
    time.sleep(5)
    print("Motors ARMED")

    master.arducopter_arm()

    print("Taking off!")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude)

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
        print(f"Altitude: {current_altitude}")

        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.2)


#hover
def hover(master): 
    """
    Hover at the current location using pymavlink.
    """
    print("Setting vehicle to hover at current location")
    master.set_mode('GUIDED')

    # Get current location
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    current_alt = msg.relative_alt / 1000.0

    # Hover at the current position
    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,
        1,
        0, 0, 0, 0,
        int(current_lat * 1e7),
        int(current_lon * 1e7),
        current_alt
    )

def send_uart_data(serial_port, data):
    """
    Send data via UART.
    """
    serial_port.write(data.encode())
#alititude
def adjust_altitude(master, altitude_change):
    """
    Adjust the vehicle's altitude by a specified change (in meters).
    """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    initial_altitude = msg.relative_alt / 1000.0  # Convert from millimeters to meters
    target_altitude = initial_altitude + altitude_change
    print(f"Initial altitude: {initial_altitude:.2f} meters")
    print(f"Target altitude: {target_altitude:.2f} meters")

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_altitude:.2f} meters")
        remaining_distance = target_altitude - current_altitude
        print(f"Remaining distance: {remaining_distance:.2f} meters")

        if abs(remaining_distance) < 0.1:
            print(f"Reached target altitude: {target_altitude:.2f} meters")
            send_ned_velocity(master, 0, 0, 0, 1)  # Stop the drone
            break

        velocity_z = -min(max(remaining_distance * 1.0, -2.0), 2.0)  # Increased scaling factor and velocity limit
        print(f"Sending velocity_z: {velocity_z:.2f} m/s (negative means ascending)")
        send_ned_velocity(master, 0, 0, velocity_z, 1)

        time.sleep(0.1)

def send_ned_velocity(master, velocity_x, velocity_y, velocity_z, duration):
    """
    Sends velocity commands in the NED (North-East-Down) frame to move the vehicle.
    """
    msg = master.mav.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        master.target_system,  # target system
        master.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (ignore position, only use velocity)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )

    for _ in range(duration):
        master.mav.send(msg)
        time.sleep(0.1)
#backward
def move_backward_from_initial(vehicle, distance):
    """
    Move the vehicle backward by a specified distance from its initial location.
    """
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    initial_lat = msg.lat / 1e7
    initial_lon = msg.lon / 1e7
    print(f"Initial position: lat={initial_lat}, lon={initial_lon}")

    target_lat, target_lon = get_location_metres(initial_lat, initial_lon, -distance, 0)

    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        distance_to_target = get_distance_metres(current_lat, current_lon, target_lat, target_lon)
        print(f"Distance to target: {distance_to_target} meters")

        if distance_to_target < 1:
            print("Reached target location")
            break

        send_ned_velocity(vehicle, -1, 0, 0, 1)  # velocity_x = -1 m/s (backward)
#forward
def move_forward_from_initial(vehicle, distance):
    """
    Move the vehicle forward by a specified distance from its initial location.
    """
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    initial_lat = msg.lat / 1e7
    initial_lon = msg.lon / 1e7
    print(f"Initial position: lat={initial_lat}, lon={initial_lon}")

    target_lat, target_lon = get_location_metres(initial_lat, initial_lon, distance, 0)

    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        distance_to_target = get_distance_metres(current_lat, current_lon, target_lat, target_lon)
        print(f"Distance to target: {distance_to_target} meters")

        if distance_to_target < 1:
            print("Reached target location")
            break

        send_ned_velocity(vehicle, 1, 0, 0, 1)  # velocity_x = 1 m/s (forward)
#left
def move_left_from_initial(vehicle, distance):
    """
    Move the vehicle left by a specified distance from its initial location.
    """
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    initial_lat = msg.lat / 1e7
    initial_lon = msg.lon / 1e7
    print(f"Initial position: lat={initial_lat}, lon={initial_lon}")

    target_lat, target_lon = get_location_metres(initial_lat, initial_lon, 0, -distance)

    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        distance_to_target = get_distance_metres(current_lat, current_lon, target_lat, target_lon)
        print(f"Distance to target: {distance_to_target:.2f} meters")

        if distance_to_target < 1:
            print("Reached target location")
            break

        send_ned_velocity(vehicle, 0, -1, 0, 1)  # velocity_y = -1 m/s (left)
#right
def move_right_from_initial(vehicle, distance):
    """
    Move the vehicle right by a specified distance from its initial location.
    """
    # Get initial GPS position
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    initial_lat = msg.lat / 1e7
    initial_lon = msg.lon / 1e7
    print(f"Initial position: lat={initial_lat}, lon={initial_lon}")

    # Calculate the new target location right from the initial location
    target_lat, target_lon = get_location_metres(initial_lat, initial_lon, 0, distance)

    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        # Calculate the distance from the current position to the target location
        distance_to_target = get_distance_metres(current_lat, current_lon, target_lat, target_lon)
        print(f"Distance to target: {distance_to_target:.2f} meters")

        if distance_to_target < 1:
            print("Reached target location")
            break

        # Send right velocity command
        send_ned_velocity(vehicle, 0, 1, 0, 1)  # velocity_y = 1 m/s (right)
#turnLeft   
def turnLeft(connection ,yaw_angle, yaw_rate):
    # Get initial heading
    initial_heading = get_current_heading(connection)
    if initial_heading is None:
        print("Unable to get initial heading. Exiting.")
        connection.close()
        exit()

    # Set yaw command to turn 10 degree counterclockwise (left)
    set_yaw(connection ,yaw_angle, yaw_rate, -1, 1)

    # Wait for the vehicle to turn
    while True:
        current_heading = get_current_heading(connection)
        if current_heading is None:
            print("Unable to get current heading. Exiting.")
            break

        print(f"Current heading: {current_heading:.2f}")

        # Check if the drone has turned by approximately 1 degree
        if abs(current_heading - (initial_heading - 1)) < 1:
            print("Turned by 10 degree left.")
            break

        time.sleep(0.1)
#turnRight
def turnRight(connection, yaw_angle, yaw_rate):
    # Get initial heading
    initial_heading = get_current_heading(connection)
    if initial_heading is None:
        print("Unable to get initial heading. Exiting.")
        connection.close()
        exit()

    # Set yaw command to turn 10 degree clockwise
    set_yaw(connection, yaw_angle, yaw_rate, 1, 1)

    # Wait for the vehicle to turn
    while True:
        current_heading = get_current_heading(connection)
        if current_heading is None:
            print("Unable to get current heading. Exiting.")
            break

        print(f"Current heading: {current_heading:.2f}")

        # Check if the drone has turned by approximately 1 degree
        if abs(current_heading - (initial_heading + 1)) < 1:
            print("Turned by 10 degree.")
            break

        time.sleep(0.1)
#land
def descend_and_land(master):
    """
    Descends the vehicle to the ground and lands it.
    """
    print("Descending to ground and landing")

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
    print(f"Current altitude: {current_altitude:.2f} meters")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        altitude = msg.relative_alt / 1000.0  # Altitude in meters
        print(f"Altitude: {altitude:.2f} meters")

        if altitude <= 0.1:
            print("Landed successfully")
            break

        time.sleep(0.1)

#GPS
def get_initial_altitude(master):
    """
    Gets the initial altitude of the drone.
    """
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            initial_altitude = msg.relative_alt / 1000.0  # Altitude in meters
            print(f"Initial altitude: {initial_altitude:.2f} meters")
            return initial_altitude

def fly_to_gps_location(master, target_latitude, target_longitude, target_altitude):
    """
    Commands the vehicle to fly to the specified GPS coordinates (latitude, longitude, and altitude).
    """
    print(f"Flying to GPS coordinates: Latitude={target_latitude}, Longitude={target_longitude}, Altitude={target_altitude}")

    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        0,  # seq (waypoint number)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
        2,  # current (0=first, 1=not first)
        1,  # autocontinue
        0, 0, 0, 0,  # param1-4 (not used)
        int(target_latitude * 1e7),  # latitude in 1e7 degrees
        int(target_longitude * 1e7),  # longitude in 1e7 degrees
        target_altitude * 1000  # altitude in millimeters
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0  # in meters
        
        print(f"Current Position: Lat={current_lat:.7f}, Lon={current_lon:.7f}, Alt={current_alt:.2f} meters")
        
        if abs(current_lat - target_latitude) < 0.00001 and abs(current_lon - target_longitude) < 0.00001 and abs(current_alt - target_altitude) < 0.5:
            print("Reached target location")
            break

        time.sleep(0.1)

def main():
    #global connection
    connection0 = connect_vehicle('udp:127.0.0.1:14550')
    connection1 = connect_vehicle('udp:127.0.0.1:14551')
    connection2 = connect_vehicle('udp:127.0.0.1:14552')
    connection3 = connect_vehicle('udp:127.0.0.1:14553')
    connection4 = connect_vehicle('udp:127.0.0.1:14554')
    
    # Example usage: Arm and take off to 10 meters
    arm_and_takeoff(connection0, 10)
    arm_and_takeoff(connection1, 10)
    #arm_and_takeoff(connection2, 10)
    #arm_and_takeoff(connection3, 10)
    #arm_and_takeoff(connection4, 10)

    # Example usage: Move forward 10 meters
    move_forward_from_initial(connection0, 10)
    
    # Example usage: Turn right
    turnRight(connection0, 10, 10)
    
    # Example usage: Land the vehicle
    descend_and_land(connection0)
    
if __name__ == "__main__":
    main()
