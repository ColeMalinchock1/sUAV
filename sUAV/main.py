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

    x_velocity = 0 # [m/s]
    y_velocity = 0 # [m/s]
    altitude = 0 # [m]
    yaw = 90 # [deg/s]
    
    logger.info("Current mission: 4 Rotation")
    logger.info(f"X Velocity: {x_velocity} m/s")
    logger.info(f"Y Velocity: {y_velocity} m/s")
    logger.info(f"Altitude: {altitude} m")
    logger.info(f"Yaw: {yaw} degrees")

    time.sleep(5)

    # Moves the drone forward for 2 seconds
    # pixhawk.command_XYA(x_velocity, y_velocity, altitude)

    pixhawk.command_YAW(yaw)

    logger.info(str(time.time()))

    time.sleep((yaw/YAW_SPEED) + 5)

    time.sleep(2)

    pixhawk.command_YAW(yaw)

    logger.info(str(time.time()))

    time.sleep((yaw/YAW_SPEED) + 5)

    time.sleep(2)

    pixhawk.command_YAW(yaw)

    logger.info(str(time.time()))

    time.sleep((yaw/YAW_SPEED) + 5)

    time.sleep(2)

    # time.sleep(5)

    # Stop the drone after flying for 5 seconds
    # pixhawk.command_XYA(0, 0, altitude)

    # time.sleep(1)

    # Rotates the drone 90 degrees to the right
    pixhawk.command_YAW(yaw)

    logger.info(str(time.time()))

    time.sleep((yaw/YAW_SPEED) + 5)

if __name__ == "__main__":

    try:

        # Creates the logger
        logger = Logger()

        # Checks if the logger was created correctly
        # Else report it and continue
        if logger:

            logger.info("Logger created")

            # Creates the pixhawk with the logger
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
        