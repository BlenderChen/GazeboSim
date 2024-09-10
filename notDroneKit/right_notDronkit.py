from pymavlink import mavutil
import time
import math

def connect_to_vehicle(connection_string):
    """
    Connects to the vehicle using pymavlink.
    """
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Heartbeat received. Connected to vehicle.")
    return master

def get_location_metres(original_lat, original_lon, dNorth, dEast):
    """
    Returns a tuple containing the latitude and longitude `dNorth` and `dEast` metres
    from the specified original location (original_lat, original_lon).
    """
    earth_radius = 6378137.0  # Radius of the Earth in meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_lat / 180.0))
    newlat = original_lat + (dLat * 180.0 / math.pi)
    newlon = original_lon + (dLon * 180.0 / math.pi)
    return newlat, newlon

def get_distance_metres(lat1, lon1, lat2, lon2):
    """
    Returns the ground distance in metres between two GPS points.
    """
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    return math.sqrt((dlat ** 2) + (dlon ** 2)) * 1.113195e5

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Sends velocity commands in the NED (North-East-Down) frame to move the vehicle.
    """
    msg = vehicle.mav.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        vehicle.target_system,  # target system
        vehicle.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (ignore position, only use velocity)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    for _ in range(duration):
        vehicle.mav.send(msg)
        time.sleep(1)

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

# Main execution
if __name__ == "__main__":
    connection_string = 'udp:127.0.0.1:14550'  # Update this to your connection string
    vehicle = connect_to_vehicle(connection_string)
    
    # Move the drone 5 meters right from its initial location
    move_right_from_initial(vehicle, 5)

    # Close vehicle object before exiting the script
    print("Closing vehicle connection")
    vehicle.close()
