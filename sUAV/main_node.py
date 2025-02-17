from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time
import logging
import math

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Connect to the vehicle
logger.info("Connecting to vehicle...")
vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)

# Global variables
in_guided_mode = False  # Track if we're in GUIDED mode
last_mode_switch_time = 0  # Track last mode switch time
mission_not_started = True
MODE_SWITCH_COOLDOWN = 1.0  # Cooldown period between mode switches (seconds)
CHANNEL_THRESHOLD = 1200  # RC channel threshold for mode switching
EARTH_RADIUS = 6378137.0  # WGS84 equatorial radius

def check_system_health():
    """Verify system health before mode switch"""
    if not vehicle.gps_0.fix_type >= 3:
        logger.warning("Inadequate GPS fix")
        return False
    if vehicle.battery.level < 25:
        logger.warning("Low battery")
        return False
    return True

def get_current_global_position():
    return vehicle.location.global_frame, vehicle.attitude

def relative_to_global_position(delta_x, delta_y, delta_z, delta_yaw):

    pos, attitude = get_current_global_position()

    current_lat = pos.lat
    current_lon = pos.lon
    current_alt = pos.alt
    current_yaw = attitude.yaw

    # Convert lat/lon to radians
    lat_rad = math.radians(current_lat)
    lon_rad = math.radians(current_lon)
    
    # Calculate meters per degree at current latitude
    meters_per_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
    meters_per_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)
    
    # Calculate new lat/lon
    new_lat = current_lat + (delta_y / meters_per_lat)
    new_lon = current_lon + (delta_x / meters_per_lon)
    
    # Calculate new altitude (simple addition)
    new_alt = current_alt + delta_z
    
    # Calculate new yaw (normalized to 0-360 degrees)
    new_yaw = (current_yaw + delta_yaw) % 360
    
    return (new_lat, new_lon, new_alt, new_yaw)

def switch_mode(target_mode):
    """
    Switch vehicle mode with additional checks and verification
    Returns True if mode switch was successful
    """
    global in_guided_mode, last_mode_switch_time
    
    current_time = time.time()

    if not check_system_health():
        return False
    
    # Check if enough time has passed since last mode switch
    if current_time - last_mode_switch_time < MODE_SWITCH_COOLDOWN:
        logger.debug("Mode switch cooldown in effect")
        return False
        
    # Only attempt mode switch if armed
    if not vehicle.armed:
        logger.warning("Vehicle not armed - mode switch blocked")
        return False
    
    try:
        # Set the mode
        vehicle.mode = VehicleMode(target_mode)
        
        # Wait for mode change to take effect
        start_time = time.time()
        while vehicle.mode.name != target_mode:
            if time.time() - start_time > 3:  # 3-second timeout
                logger.error(f"Mode switch to {target_mode} timed out!")
                return False
            time.sleep(0.1)
        
        # Update tracking variables
        in_guided_mode = (target_mode == "GUIDED")
        last_mode_switch_time = current_time
        logger.info(f"Successfully switched to {target_mode} mode")
        return True
        
    except Exception as e:
        logger.error(f"Mode switch failed: {e}")
        return False

try:
    logger.info("Monitoring RC channels...")

    @vehicle.on_message('RC_CHANNELS')
    def rc_listener(self, name, message):
        # Check if channel 12 is above threshold
        if message.chan12_raw > CHANNEL_THRESHOLD:
            logger.info(f"Channel 12 activated: {message.chan12_raw}")
            
            if not in_guided_mode:
                switch_mode("GUIDED")
            else:
                switch_mode("LOITER")

    # Main loop with heartbeat monitoring
    while True:
        # Check if connection is still alive
        if vehicle.last_heartbeat > 2:
            logger.warning("Lost connection to vehicle!")

        if vehicle.mode.name == "GUIDED" and mission_not_started:
            time.sleep(5)
            mission_not_started = False
            msg = vehicle.message_factory.command_long_encode(
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
                0, #confirmation
                90,    # param 1, yaw in degrees
                10,          # param 2, yaw speed deg/s
                1,          # param 3, direction -1 ccw, 1 cw
                1, # param 4, relative offset 1, absolute angle 0
                0, 0, 0)    # param 5 ~ 7 not used
            # send command to vehicle
            vehicle.send_mavlink(msg)

            ack = vehicle.message_factory.command_ack_encode(
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                mavutil.mavlink.MAV_RESULT_ACCEPTED  # result
            )

            # Print the acknowledgment result
            logger.info(f"Yaw command acknowledgment: {ack.result}")

            # You can also log the current vehicle heading
            logger.info(f"Current vehicle heading: {vehicle.heading}")

            time.sleep(5)
            
            # pos = relative_to_global_position(4, 0, 0, 0)

            # a_location = LocationGlobal(pos[0], pos[1], pos[2])
            # vehicle.simple_goto(a_location)

            # Log final heading after movement
            logger.info(f"Final vehicle heading: {vehicle.heading}")

        time.sleep(0.1)

except KeyboardInterrupt:
    logger.info("\nStopping...")
finally:
    vehicle.close()