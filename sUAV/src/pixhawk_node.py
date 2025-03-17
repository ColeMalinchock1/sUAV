from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time
import logging
import math

# Constants
TARGET_OMEGA = 10  # Increased deg/sec for faster rotation
TARGET_VELOCITY = 0.5  # m/s
# Use the correct mask that ONLY enables velocity control
VELOCITY_TYPE_MASK = 0b0000111111000111

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
    """Rotates the yaw of the vehicle by a specified degree - more efficiently"""
    # Store initial heading for verification
    initial_heading = vehicle.heading
    logger.info(f"Initial heading before rotation: {initial_heading}")
    
    # Target heading calculation
    target_heading = (initial_heading + degrees) % 360
    logger.info(f"Target heading: {target_heading}")

    # Create the yaw command - more direct with higher priority
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        1,  # confirmation - increased priority
        degrees,  # param 1, yaw in degrees
        TARGET_OMEGA,  # param 2, yaw speed deg/s
        1,  # param 3, direction 1=cw, -1=ccw
        1,  # param 4, relative=1, absolute=0
        0, 0, 0)  # param 5-7 not used
    
    # Send command to vehicle
    vehicle.send_mavlink(msg)
    logger.info("Yaw command sent")
    
    # Quick verification only
    expected_duration = abs(degrees) / TARGET_OMEGA
    logger.info(f"Expected rotation duration: {expected_duration:.1f} seconds")
    
    # Wait just enough time for the command to be processed
    time.sleep(expected_duration + 1)
    
    # Log completion without waiting for exact heading
    logger.info(f"Rotation command completed. New heading: {vehicle.heading}")
    return True

def forward_velocity(distance):
    """Moves the vehicle forward using velocity control - more efficient"""
    logger.info(f"Moving forward {distance}m at {TARGET_VELOCITY}m/s")
    
    # Log starting position
    if vehicle.location.global_relative_frame.lat:
        start_lat = vehicle.location.global_relative_frame.lat
        start_lon = vehicle.location.global_relative_frame.lon
        logger.info(f"Starting position: Lat {start_lat}, Lon {start_lon}")
    
    # Set high precision velocity message
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,         # time_boot_ms
        0, 0,      # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame - relative to vehicle orientation
        VELOCITY_TYPE_MASK,  # type_mask - enables velocity control
        0, 0, 0,  # x, y, z positions (not used)
        TARGET_VELOCITY, 0, 0,  # x, y, z velocity in m/s (forward, right, down)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)
    
    # Send velocity command once
    vehicle.send_mavlink(msg)
    logger.info("Forward velocity command sent")
    
    # Calculate time needed for movement
    travel_time = distance / TARGET_VELOCITY
    
    # Wait just enough time for the movement to complete
    time.sleep(travel_time + 0.5)
    
    # Send stop command
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,         # time_boot_ms
        0, 0,      # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        VELOCITY_TYPE_MASK,  # type_mask - enables velocity control
        0, 0, 0,  # x, y, z positions (not used)
        0, 0, 0,  # x, y, z velocity (all zeros = stop)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)
    vehicle.send_mavlink(stop_msg)
    
    # Log final position
    if vehicle.location.global_relative_frame.lat:
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        logger.info(f"Ending position: Lat {current_lat}, Lon {current_lon}")
    
    logger.info("Forward movement command completed")
    return True

def main():
    """Main function for drone movement sequence"""
    mission_completed = False
    
    # Main program logic - streamlined
    if vehicle.mode.name == "GUIDED":
        if check_system_health():
            logger.info("Starting movement sequence in GUIDED mode")
            
            # Just a short stabilization pause
            time.sleep(1)
            
            # Log starting heading
            logger.info(f"Starting vehicle heading: {vehicle.heading}")
            
            # Rotate 90 degrees
            logger.info("Executing 90-degree rotation")
            rotate_yaw(90)
            
            # Just a short pause between commands
            time.sleep(0.5)
            
            # Move forward
            logger.info("Executing forward movement")
            forward_velocity(5)
            
            logger.info("Movement sequence completed")
            mission_completed = True
        else:
            logger.info("Unhealthy")

if __name__ == "__main__":
    try:
        # Set up logging
        logging.basicConfig(level=logging.INFO)
        logger = logging.getLogger(__name__)
        
        # Connect to the vehicle
        logger.info("Connecting to vehicle...")
        vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)
        logger.info("Connected to vehicle")

        # Run the main function
        while True:
            main()
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("\nStopping due to keyboard interrupt...")
    except Exception as e:
        logger.error(f"Error occurred: {str(e)}")
    finally:
        # Ensure clean disconnect
        vehicle.close()
        logger.info("Vehicle connection closed")
