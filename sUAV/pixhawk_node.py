from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Connect to the vehicle
logger.info("Connecting to vehicle...")
vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)

target_omega = 10 # deg/sec
target_velocity = 0.5 # m/s
type_mask = 0b0000111111111000

def check_system_health():
    """Verify system health before mode switch"""
    if not vehicle.gps_0.fix_type >= 3:
        logger.warning("Inadequate GPS fix")
        return False
    if vehicle.battery.level < 25:
        logger.warning("Low battery")
        return False
    return True

def rotate_yaw(degrees):
    """Rotates the yaw of the vehicle to a specified degree"""

    # Create the message to send to the pixhawk
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        degrees,    # param 1, yaw in degrees
        target_omega,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        1, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # Calculate how long it should be rotating and get the initial start time
    duration = degrees/target_omega
    start_time = time.time()

    # Rotate until the time is exceeded or GUIDED mode is no longer the current mode
    while (time.time() - start_time < duration + 1):
        if (vehicle.mode.name != "GUIDED"):
            logger.info("Mode changed, stopping movement")
            return False
        time.sleep(0.1)
    
    logger.info("Rotation movement complete")
    return True

def set_speed(speed):
    """Sets the speed of the vehicle before any movement"""

    # First, set the speed limit
    speed_msg = vehicle.message_factory.command_long_encode(
        0, 0,                                   # target system, target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
        0,                                      # confirmation
        1,                                      # param 1: speed type (1=ground speed)
        speed,                               # param 2: speed (m/s)
        -1,                                     # param 3: throttle (-1=no change)
        0, 0, 0, 0                              # param 4-7: not used
    )

    # Sends the speed to the pixhawk
    vehicle.send_mavlink(speed_msg)
    logger.info(f"Setting speed limit to {target_velocity} m/s")
    time.sleep(0.5)  # Give time for the command to be processed

def forward(distance):
    """Moves the vehicle forward to a specified distance"""

    # Set the speed of the vehicle
    set_speed(target_velocity)

    # Create the message to the pixhawk on the distance that it should be moved to
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, #command
        type_mask, #Mask
        distance, 0, 0,    # param 1, distance in meters forward
        0, 0, 0,          # param 2, velocity
        0, 0, 0,          # param 3, acceleration
        0, 0)    # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
    # Calculate time needed to travel the distance at given speed and get the start time
    start_time = time.time()
    travel_time = distance /target_velocity

    # Log the movement
    logger.info(f"Moving forward {distance}m at {target_velocity}m/s for ~{travel_time:.1f} seconds")

    # Sleep until the rotation is done or the mode is switched out of guided
    while (time.time() - start_time < travel_time + 1):
        if vehicle.mode.name != "GUIDED":
            logger.info("Mode changed, stopping movement")
            return False
        time.sleep(0.1)

    logger.info("Forward movement complete")
    return True

def main():
    """Main function for creating the movement and ensuring that the vehicle is safe"""

    # Main loop with heartbeat monitoring
    while True:
        
        # Check if connection is still alive
        if vehicle.last_heartbeat > 2:
            logger.warning("Lost connection to vehicle!")
        
        # Checks if the vehicle is in guided mode
        if vehicle.mode.name == "GUIDED":

            # Checks that the system is healthy
            if check_system_health():

                # Wait before executing the movement
                time.sleep(5)

                # You can also log the current vehicle heading
                logger.info(f"Current vehicle heading: {vehicle.heading}")

                # Rotate 90 degrees
                if (rotate_yaw(90)):
                    
                    # Log the final rotation
                    logger.info(f"Final vehicle heading: {vehicle.heading}")

                    # Move forward 5 meters
                    forward(5)

        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\nStopping...")
    finally:
        vehicle.close()
