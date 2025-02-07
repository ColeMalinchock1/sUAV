from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Connect to the vehicle
logger.info("Connecting to vehicle...")
vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)

# Global variables
in_auto_mode = False  # Track if we're in AUTO mode
last_mode_switch_time = 0  # Track last mode switch time
MODE_SWITCH_COOLDOWN = 1.0  # Cooldown period between mode switches (seconds)
CHANNEL_THRESHOLD = 1500  # RC channel threshold for mode switching

def switch_mode(target_mode):
    """
    Switch vehicle mode with additional checks and verification
    Returns True if mode switch was successful
    """
    global in_auto_mode, last_mode_switch_time
    
    current_time = time.time()
    
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
        in_auto_mode = (target_mode == "AUTO")
        last_mode_switch_time = current_time
        logger.info(f"Successfully switched to {target_mode} mode")
        return True
        
    except Exception as e:
        logger.error(f"Mode switch failed: {e}")
        return False

def switch_to_mission_mode():
    """Switch vehicle to AUTO mode"""
    return switch_mode("AUTO")

def switch_to_manual_mode():
    """Switch vehicle back to STABILIZE mode"""
    return switch_mode("STABILIZE")

try:
    logger.info("Monitoring RC channels...")
    
    @vehicle.on_message('RC_CHANNELS')
    def rc_listener(self, name, message):
        # Check if channel 12 is above threshold
        if message.chan12_raw > CHANNEL_THRESHOLD:
            logger.info(f"Channel 12 activated: {message.chan12_raw}")
            
            if not in_auto_mode:
                switch_to_mission_mode()
            else:
                switch_to_manual_mode()
                
    # Main loop with heartbeat monitoring
    while True:
        # Check if connection is still alive
        if vehicle.last_heartbeat > 2:
            logger.warning("Lost connection to vehicle!")
        time.sleep(0.1)

except KeyboardInterrupt:
    logger.info("\nStopping...")
finally:
    vehicle.close()