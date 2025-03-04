from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time
import logging
import math

# Constants
TARGET_OMEGA = 10  # deg/sec
TARGET_VELOCITY = 0.5  # m/s
# CORRECT MASK FOR VELOCITY CONTROL - enables velocity control in x, y, z
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
    """Rotates the yaw of the vehicle by a specified degree"""
    # Store initial heading for verification
    initial_heading = vehicle.heading
    logger.info(f"Initial heading before rotation: {initial_heading}")
    
    # Target heading calculation
    target_heading = (initial_heading + degrees) % 360
    logger.info(f"Target heading after rotation: {target_heading}")

    # Create the yaw command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        degrees,  # param 1, yaw in degrees
        TARGET_OMEGA,  # param 2, yaw speed deg/s
        1,  # param 3, direction 1=cw, -1=ccw
        1,  # param 4, relative=1, absolute=0
        0, 0, 0)  # param 5-7 not used
    
    # Send command to vehicle
    vehicle.send_mavlink(msg)
    
    # Create and log acknowledgment (just for confirmation of receipt)
    ack = vehicle.message_factory.command_ack_encode(
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        mavutil.mavlink.MAV_RESULT_ACCEPTED  # result
    )
    logger.info(f"Yaw command sent, acknowledgment: {ack.result}")
    
    # Calculate minimum expected rotation time
    expected_duration = abs(degrees) / TARGET_OMEGA
    logger.info(f"Expected rotation duration: {expected_duration} seconds")
    
    # Wait for rotation with heading verification
    start_time = time.time()
    max_wait_time = expected_duration + 5  # Add safety margin
    
    # Wait for rotation to complete with active monitoring
    while time.time() - start_time < max_wait_time:
        current_heading = vehicle.heading
        
        # Calculate heading difference (handles wraparound at 0/360)
        heading_diff = abs((current_heading - target_heading + 180) % 360 - 180)
        
        logger.info(f"Current heading: {current_heading}, heading difference: {heading_diff}°")
        
        # Check if we've reached target heading within tolerance
        if heading_diff < 5:  # 5 degree tolerance
            logger.info(f"Reached target heading: {current_heading}")
            time.sleep(1)  # Short pause to stabilize
            return True
            
        # If mode changed, exit
        if vehicle.mode.name != "GUIDED":
            logger.info("Mode changed, stopping rotation")
            return False
            
        time.sleep(0.5)  # Check more frequently
    
    # If we get here, the rotation didn't complete within the time limit
    logger.warning(f"Rotation timeout. Current heading: {vehicle.heading}, Target: {target_heading}")
    return False

def forward_velocity(distance):
    """Moves the vehicle forward using velocity control"""
    logger.info(f"Attempting to move forward {distance}m at {TARGET_VELOCITY}m/s")
    
    # Calculate time needed for movement
    travel_time = distance / TARGET_VELOCITY
    logger.info(f"Expected travel time: {travel_time:.1f} seconds")
    
    # Set the message to use VELOCITY control (not position)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame - BODY_NED is relative to vehicle orientation
        VELOCITY_TYPE_MASK,  # type_mask - enables velocity control
        0, 0, 0,  # x, y, z positions (not used)
        TARGET_VELOCITY, 0, 0,  # x, y, z velocity in m/s (forward, right, down)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    
    # Log GPS starting position if available
    if vehicle.location.global_relative_frame.lat:
        start_lat = vehicle.location.global_relative_frame.lat
        start_lon = vehicle.location.global_relative_frame.lon
        logger.info(f"Starting position: Lat {start_lat}, Lon {start_lon}")
    
    # Send command and start movement
    start_time = time.time()
    logger.info("Starting forward movement")
    
    # Continue sending velocity commands until time elapses
    while time.time() - start_time < travel_time + 2:  # Add buffer time
        # Re-send velocity command (needed for continuous movement)
        vehicle.send_mavlink(msg)
        
        # Check if we're still in GUIDED mode
        if vehicle.mode.name != "GUIDED":
            logger.info("Mode changed, stopping forward movement")
            break
            
        # Log current position if GPS is available
        if vehicle.location.global_relative_frame.lat:
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            
            # Log position and calculate approximate distance moved
            if 'start_lat' in locals():
                # Rough conversion to meters (approximation)
                dlat = current_lat - start_lat
                dlon = current_lon - start_lon
                earth_radius = 6378137.0  # meters
                meters_lat = dlat * math.pi * earth_radius / 180.0
                meters_lon = dlon * math.pi * earth_radius * math.cos(math.radians(current_lat)) / 180.0
                dist_moved = math.sqrt(meters_lat**2 + meters_lon**2)
                
                logger.info(f"Current position: Lat {current_lat}, Lon {current_lon}, Moved ~{dist_moved:.2f}m")
        
        # Wait before sending next command
        time.sleep(0.5)
    
    # Send stop command (zero velocity)
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        VELOCITY_TYPE_MASK,  # type_mask - enables velocity control
        0, 0, 0,  # x, y, z positions (not used)
        0, 0, 0,  # x, y, z velocity (all zeros - stop movement)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    vehicle.send_mavlink(stop_msg)
    
    logger.info("Forward movement complete")
    return True

def main():
    """Main function for drone movement sequence"""
    mission_completed = False
    
    while True:
        # Check if connection is still alive
        if vehicle.last_heartbeat > 2:
            logger.warning("Lost connection to vehicle!")
        
        # Execute mission if in GUIDED mode and not already completed
        if vehicle.mode.name == "GUIDED" and not mission_completed:
            # Verify system health
            if check_system_health():
                logger.info("In GUIDED Mode, beginning movement sequence")
                
                # Wait for stability
                logger.info("Waiting 5 seconds before movement...")
                time.sleep(5)
                
                # Log starting heading
                logger.info(f"Starting vehicle heading: {vehicle.heading}")
                
                # First, rotate 90 degrees
                logger.info("Starting 90-degree rotation")
                if rotate_yaw(90):
                    logger.info(f"Rotation complete. New heading: {vehicle.heading}")
                    
                    # Wait before next movement
                    time.sleep(2)
                    
                    # Now move forward
                    logger.info("Starting forward movement")
                    if forward_velocity(5):
                        logger.info("Movement sequence completed successfully")
                        mission_completed = True
                    else:
                        logger.warning("Forward movement failed")
                else:
                    logger.warning("Rotation failed")
        
        # Brief pause in the main loop
        time.sleep(0.1)

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
        main()
    except KeyboardInterrupt:
        logger.info("\nStopping due to keyboard interrupt...")
    except Exception as e:
        logger.error(f"Error occurred: {str(e)}")
    finally:
        # Ensure clean disconnect
        vehicle.close()
        logger.info("Vehicle connection closed")