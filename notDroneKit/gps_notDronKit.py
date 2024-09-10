from pymavlink import mavutil
import time

def connect_vehicle(connection_string):
    """
    Connects to the vehicle using MAVLink.
    """
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Connected to vehicle")
    return master

def arm_and_takeoff(master, target_altitude):
    """
    Arms the vehicle and flies to target_altitude using pymavlink.
    """
    print("Basic pre-arm checks")
    while not master.motors_armed():
        print("Waiting for arming...")
        time.sleep(1)

    print("Arming motors")
    master.arducopter_arm()
    master.set_mode('GUIDED')

    while not master.motors_armed():
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude)
    
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
        print(f"Altitude: {current_altitude:.2f} meters")

        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

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
    # Replace this with the actual connection string for your setup
    connection_string = 'udp:127.0.0.1:14550'
    
    # Connect to the vehicle
    master = connect_vehicle(connection_string)

    # Arm and takeoff to an initial altitude of 10 meters
    arm_and_takeoff(master, 10)

    # Target GPS coordinates
    target_latitude = 47.397742  # Example latitude
    target_longitude = 8.545594  # Example longitude
    target_altitude = 20  # Target altitude in meters

    # Fly to the specified GPS coordinates
    fly_to_gps_location(master, target_latitude, target_longitude, target_altitude)
    
    # Optionally, land the vehicle after reaching the target location
    print("Landing the vehicle")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
        print(f"Altitude: {current_altitude:.2f} meters")

        if current_altitude <= 0.1:
            print("Landed successfully")
            break

        time.sleep(1)

if __name__ == "__main__":
    main()
