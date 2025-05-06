""" Obstacle Avoidance algorithm to read from scan and complete the maneuver """

from sUAV.src.lidar import Lidar
import time

class ObstacleAvoidance:

    def __init__(self, pixhawk, logger):
        self.pixhawk = pixhawk
        self.logger = logger
        self.lidar = Lidar()

    def detect_obstacle(self):
        """ Uses the lidar to check if there is an obstacle detected """

        # Gets the scan from the lidar
        obstacle = self.lidar.get_scan()

        # If there is an obstacle report it
        if (obstacle is not None):
            self.logger.info("Obstacle detected.")
            return True
        else:
            return False

    def maneuver(self, waypoint):

        # Stop the drone before maneuver
        self.logger.info("Stopping.")
        self.pixhawk.command_XYA(0, 0, 0)
        self.logger.info("Stopped.")

        # Continue maneuvering while there is an obstacle
        while self.detect_obstacle():
            time.sleep(0.1)
        
        # Command the pixhawk to continue to waypoint after maneuvering
        self.pixhawk.goto(waypoint)
