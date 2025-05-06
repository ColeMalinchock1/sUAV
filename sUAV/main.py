"""Main function to be ran when running the FWC project"""

# Import necessary libraries
from sUAV.src.pixhawk_commands import PixhawkCommands
from sUAV.lib.logger import Logger
from sUAV.lib.constants import *
from sUAV.src.obstacle_avoidance import ObstacleAvoidance

import time

# Initialize the pixhawk and logger
pixhawk = logger = obstacle_avoidance = None

def control_loop(waypoints):
    """Main function to run the obstacle avoidance algorithm"""

    time.sleep(2)
    logger.info("Beginning mission")

    waypoint_reached = True

    counter = 0

    # Loops through all the waypoints
    for waypoint in waypoints:

        logger.info(f"Current waypoint: {counter}/{len(waypoints)}")
        logger.info(f"Waypoint location: {waypoint.lat}, {waypoint.lon}, {waypoint.alt}")

        # Checks if the waypoint was reached
        if waypoint_reached:

            # Commands the pixhawk to go to the next waypoint and makes waypoint reached false
            pixhawk.goto(waypoint)
            waypoint_reached = False

        # Exits the while loop when the waypoint is reached
        while not waypoint_reached:

            # Checks if there are any obstacles detected with the lidar
            if obstacle_avoidance.detect_obstacle():
                obstacle_avoidance.maneuver(waypoint)

            # Checks if the pixhawk completed the simple goto
            waypoint_reached = pixhawk.waypoint_reached()

            time.sleep(0.1)
        
        logger.info("Waypoint reached!")
        


def initialize():

    # Waiting to get mission
    logger.info("Waiting for mission")
    previous_time = time.time()
    while True:
        if (time.time() - previous_time > 3):
            waypoints = pixhawk.get_mission()
            if waypoints is not None:
                logger.info("Received mission")
                break 
            logger.info("No mission found, waiting...")
            previous_time = time.time()
        time.sleep(0.1)
    
    return waypoints
    
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

                logger.info("Pixhawk connected")

                obstacle_avoidance = ObstacleAvoidance(pixhawk, logger)

                waypoints = initialize()
                
                control_loop(waypoints)
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
        