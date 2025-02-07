from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to vehicle
print("Connecting to vehicle...")
vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    velocity_x: Forward/back (-) in meters/second
    velocity_y: Left/right (-) in meters/second
    velocity_z: Up/down (-) in meters/second
    duration: Time in seconds to move at this velocity
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)

    # Send command to vehicle
    for x in range(0, int(duration * 10)):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def condition_yaw(heading):
    """
    Rotate vehicle to specified heading in degrees.
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,       # confirmation
        heading, # heading in degrees
        0,       # yaw speed deg/s
        1,       # direction -1 ccw, 1 cw
        1,       # relative offset 1, absolute angle 0
        0, 0, 0) # param 5-7 not used

    vehicle.send_mavlink(msg)

try:
    # Wait for the vehicle to be armed
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Starting movement sequence...")
    
    # Move forward 1 meter
    print("Moving forward 1 meter...")
    send_ned_velocity(0.5, 0, 0, 2)  # Move at 0.5 m/s for 2 seconds
    time.sleep(2)  # Wait for movement to complete

    # Move right 2 meters
    print("Moving right 2 meters...")
    send_ned_velocity(0, 0.5, 0, 4)  # Move at 0.5 m/s for 4 seconds
    time.sleep(2)  # Wait for movement to complete

    # Yaw 180 degrees
    print("Rotating 180 degrees...")
    condition_yaw(180)
    time.sleep(3)  # Wait for rotation to complete

    # Increase altitude by 1 meter
    print("Increasing altitude by 1 meter...")
    send_ned_velocity(0, 0, -0.5, 2)  # Move up at 0.5 m/s for 2 seconds
    time.sleep(2)  # Wait for movement to complete

    print("Movement sequence completed!")

except KeyboardInterrupt:
    print("User interrupted!")
finally:
    # Close vehicle object
    print("Closing vehicle object")
    vehicle.close()