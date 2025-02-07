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
in_guided_mode = False  # Track if we're in GUIDED mode
last_mode_switch_time = 0  # Track last mode switch time
MODE_SWITCH_COOLDOWN = 1.0  # Cooldown period between mode switches (seconds)
CHANNEL_THRESHOLD = 1500  # RC channel threshold for mode switching

def check_system_health():
    """Verify system health before mode switch"""
    if not vehicle.gps_0.fix_type >= 3:
        logger.warning("Inadequate GPS fix")
        return False
    if vehicle.battery.level < 25:
        logger.warning("Low battery")
        return False
    return True

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
        time.sleep(0.1)

except KeyboardInterrupt:
    logger.info("\nStopping...")
finally:
    vehicle.close()