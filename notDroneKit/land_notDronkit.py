import time
from pymavlink import mavutil

def connect_to_vehicle(connection_string):
    """
    Connects to the vehicle using pymavlink.
    """
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Heartbeat received. Connected to the vehicle.")
    return master

def set_mode(master, mode):
    """
    Sets the flight mode of the vehicle.
    """
    if mode not in master.mode_mapping():
        print(f"Mode {mode} not available")
        return

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Wait for acknowledgment of the mode change
    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(f"Mode set to {mode}")
            break

def arm_vehicle(master):
    """
    Arms the vehicle.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming motors")
    master.motors_armed_wait()
    print("Motors armed")

def descend_and_land(master):
    """
    Descends the vehicle to the ground and lands it.
    """
    print("Descending to ground and landing")


    # Get the current altitude
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
    print(f"Current altitude: {current_altitude:.2f} meters")

    # Send command to land
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    # Wait until the vehicle lands
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        altitude = msg.relative_alt / 1000.0  # Altitude in meters
        print(f"Altitude: {altitude:.2f} meters")
        if altitude <= 0.1:  # Check if the vehicle is close to the ground
            print("Landed successfully.")
            # Disarm the vehicle
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            break
        time.sleep(1)

# Main script execution
if __name__ == "__main__":
    connection_string = 'udp:127.0.0.1:14550'  # Change this to your connection string
    master = connect_to_vehicle(connection_string)

    # Arm the vehicle
    arm_vehicle(master)

    # Descend and land
    descend_and_land(master)

    # Close the connection
    print("Closing vehicle connection")
    master.close()
