"""Main function to be ran when running the FWC project"""

# Import necessary libraries
from sUAV.src.pixhawk_commands import PixhawkCommands
from sUAV.lib.logger import Logger
from sUAV.lib.constants import *
from sUAV.src.waypoint_follower import WaypointFollower

import time

# Initialize the pixhawk and logger
pixhawk = logger = obstacle_avoidance = None

def control_loop():
    """Main function to run the obstacle avoidance algorithm"""
    

    time.sleep(2)
    logger.info("Beginning mission")

    while (obstacle_avoidance.current_waypoint):
        obstacle_avoidance.proceed_to_waypoint()
        time.sleep(0.1)

def initialize():

    # Waiting to get mission
    logger.info("Waiting for mission")
    mission = pixhawk.get_mission()
    logger.info("Received mission")

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
                obstacle_avoidance = WaypointFollower(logger, pixhawk)

            # Checks if the pixhawk is created correctly
            # Else report it and continue
            if pixhawk:

                logger.info("Pixhawk connected")

                initialize()
                
                control_loop()
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
        