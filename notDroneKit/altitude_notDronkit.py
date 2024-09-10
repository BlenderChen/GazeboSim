import time
from pymavlink import mavutil

def adjust_altitude(master, altitude_change):
    """
    Adjust the vehicle's altitude by a specified change (in meters).
    
    Args:
    master: MAVLink connection object
    altitude_change: Change in altitude in meters (positive for ascent, negative for descent)
    """
    # Get the current altitude
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    initial_altitude = msg.relative_alt / 1000.0  # Convert from millimeters to meters
    target_altitude = initial_altitude + altitude_change
    print(f"Initial altitude: {initial_altitude:.2f} meters")
    print(f"Target altitude: {target_altitude:.2f} meters")

    # Loop to adjust altitude
    while True:
        # Get the current altitude
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_altitude:.2f} meters")

        # Calculate the remaining distance to the target altitude
        remaining_distance = target_altitude - current_altitude
        print(f"Remaining distance: {remaining_distance:.2f} meters")

        # Check if the drone is within a small threshold of the target altitude
        if abs(remaining_distance) < 0.1:
            print(f"Reached target altitude: {target_altitude:.2f} meters")
            send_ned_velocity(master, 0, 0, 0, 1)  # Stop the drone
            break

        # Increase the velocity for faster altitude adjustment
        velocity_z = -min(max(remaining_distance * 1.0, -2.0), 2.0)  # Increased scaling factor and velocity limit

        print(f"Sending velocity_z: {velocity_z:.2f} m/s (negative means ascending)")

        # Send the command to adjust the altitude
        send_ned_velocity(master, 0, 0, velocity_z, 1)

        time.sleep(0.1)

def send_ned_velocity(master, velocity_x, velocity_y, velocity_z, duration):
    """
    Sends velocity commands in the NED (North-East-Down) frame to move the vehicle.
    
    Args:
    master: MAVLink connection object
    velocity_x: Velocity in the North direction (m/s)
    velocity_y: Velocity in the East direction (m/s)
    velocity_z: Velocity in the Down direction (m/s) - positive for down, negative for up
    duration: Duration to maintain the velocity command (seconds)
    """
    # Create the MAVLink message to set the target velocity
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

    # Send command to vehicle at a higher rate for the specified duration
    for _ in range(duration):
        master.mav.send(msg)
        time.sleep(0.1)

# Connect to the vehicle via MAVLink
connection_string = 'udp:127.0.0.1:14550'
print('Connecting to vehicle on:', connection_string)
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Heartbeat received from system")

# Example usage: Adjust the drone's altitude by a specified value
altitude_change = -5  # Change in meters: Positive for up, negative for down
adjust_altitude(master, altitude_change)

# Close connection
print("Close vehicle connection")
master.close()
