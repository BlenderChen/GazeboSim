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
        print(f"Altitude: {current_altitude}")

        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

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

# Connect to the vehicle
connection_string = 'udp:127.0.0.1:14550'  # Replace with your actual connection string
master = connect_vehicle(connection_string)

# Arm and take off to 10 meters altitude
arm_and_takeoff(master, 10)

# Hover at the current position
hover(master)

# Example data to send via UART
data_to_send = "Hovering at 10 meters altitude"

# Set up simulated UART connection (using mavutil for this example)
uart_port = mavutil.mavlink_connection('udp:127.0.0.1:14552')
uart_port.baudrate = 9600  # Example baud rate

# Send data via UART
send_uart_data(uart_port, data_to_send)

# Keep the vehicle hovering and sending data
try:
    while True:
        # Maintain hover by sending the same location command periodically
        hover(master)

        # Send data periodically
        send_uart_data(uart_port, data_to_send)

        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")

# Close connections
print("Close vehicle object and UART connection")
master.close()
uart_port.close()
