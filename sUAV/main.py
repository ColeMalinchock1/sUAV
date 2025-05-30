"""Main function to be ran when running the FWC project"""

# Import necessary libraries
from sUAV.src.pixhawk_commands import PixhawkCommands
from sUAV.lib.logger import Logger
from sUAV.lib.constants import *
import time

# Initialize the pixhawk and logger
pixhawk = logger = None

def main():
    """Main function to run the obstacle avoidance algorithm"""
    logger.info("Waiting to switch to guided mode")
    while(pixhawk.get_mode() != "GUIDED"):
        time.sleep(0.1)
    logger.info("Switched to guided mode")

    x_velocity = 1 # [m/s]
    y_velocity = 0 # [m/s]
    altitude = 0 # [m]
    yaw = 90 # [deg/s]
    
    logger.info("Current mission: Square Movement")
    logger.info(f"X Velocity: {x_velocity} m/s")
    logger.info(f"Y Velocity: {y_velocity} m/s")
    logger.info(f"Altitude: {altitude} m")
    logger.info(f"Yaw: {yaw} degrees")

    time.sleep(5)

    logger.info("Beginning maneuver")
    for i in range(4):
        # Rotate first (if not the first iteration)
        if i > 0:
            logger.info(f"Turning {yaw} degrees")
            pixhawk.command_YAW(yaw)

            # Wait for rotation to complete and stabilize
            time.sleep((yaw/YAW_SPEED) + 2)
        
        # Now move forward in the new direction for 3 seconds
        logger.info("Moving forward for 3 seconds")
        pixhawk.command_XYA(x_velocity, y_velocity, altitude)
        time.sleep(3)
        
        # Stop the drone
        logger.info("Stopping for 1 second")
        pixhawk.command_XYA(0, 0, 0)
        time.sleep(1)

if __name__ == "__main__":

    try:

        # Creates the logger
        logger = Logger(app_name="Drone_Flight", log_dir=LOG_DIR, udp_host="192.168.111.210",  udp_port=9999)

        # Checks if the logger was created correctly
        # Else report it and continue
        if logger:

            logger.info("Logger created")

            # Creates the pixhawk with the logger
            if not DEBUG_MODE:
                pixhawk = PixhawkCommands(logger)

            # Checks if the pixhawk is created correctly
            # Else report it and continue
            if pixhawk:

                logger.info("Pixhawk connected, running main")
                
                # Runs main function
                main()
            else:
                logger.critical("Unable to create pixhawk communication")
        else:
            print("Unable to create logger")

    # Catch a keyboard interrupt or other exception
    except KeyboardInterrupt:
        if logger:
            logger.info("Keyboard interrupt caught")
        else:
            print("Keyboard interrupt caught - no logger")

    # Finally close out the system
    finally:

        # Check if the logger and pixhawk were created successfully and closes the communication with the pixhawk
        # Else if there is just a pixhawk, it closes the communication with the pixhawk
        if logger and pixhawk:
            logger.info("Closing the vehicle communication")
            pixhawk.close_vehicle() 
        elif pixhawk:
            print("Closing the vehicle communication - no logger")
            pixhawk.close_vehicle()

        print("Complete :)")
        