"""Main function to be ran when running the FWC project"""

# Import necessary libraries
from sUAV.src.pixhawk_commands import PixhawkCommands
from sUAV.lib.logger import Logger
from sUAV.lib.constants import *
from sUAV.src.obstacle_avoidance import ObstacleAvoidance

import time

# Initialize the pixhawk and logger
pixhawk = logger = obstacle_avoidance = None

mission = [[5, 5]]

def main(mission):
    """Main function to run the obstacle avoidance algorithm"""
    logger.info("Waiting to switch to guided mode")
    while(pixhawk.get_mode() != "GUIDED"):
        time.sleep(0.1)
    logger.info("Switched to guided mode")

    time.sleep(2)

    logger.info(f"Mission: {mission}")
    logger.info("Beginning mission")

    while (obstacle_avoidance.current_waypoint):
        obstacle_avoidance.proceed_to_waypoint()
        time.sleep(0.1)

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
                
                # Creates the obstacle avoidance system
                obstacle_avoidance = ObstacleAvoidance(mission, logger, pixhawk)

            # Checks if the pixhawk is created correctly
            # Else report it and continue
            if pixhawk:

                logger.info("Pixhawk connected, running main")
                
                # Runs main function
                main(mission)
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
        