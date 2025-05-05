from sUAV.lib.constants import *

import math
import time

class WaypointFollower:

    def __init__(self, logger, pixhawk):
        self.current_waypoint_idx = 0
        self.pixhawk = pixhawk
        self.logger = logger

    def proceed_to_waypoint(self):
        if self.current_waypoint:
            current_position = self.pixhawk.get_local_position()
            self.logger.info(f"Current position: {current_position.east}, {current_position.north}")
            if (self.distance((self.current_waypoint[0], self.current_waypoint[1]), (current_position.east, current_position.north))) < WAYPOINT_RADIUS_M:
                self.logger.info("Waypoint reached! Stopping...")
                self.traveling = False
                self.pixhawk.command_XYA(0, 0, 0)
                self._next_waypoint()
            elif not self.traveling:
                self.traveling = True
                current_yaw = self.pixhawk.get_yaw()

                theta_to_waypoint = math.atan2(self.current_waypoint[0] - current_position.east, self.current_waypoint[1] - current_position.north)

                delta_yaw = current_yaw - theta_to_waypoint

                self.pixhawk.command_YAW(delta_yaw)

                time.sleep((delta_yaw / YAW_SPEED) + 1)

                self.pixhawk.command_XYA(1, 0, 0)

    def distance(self, point1, point2):
        """ Gets the distance between two points """
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def _next_waypoint(self):
        if self.current_waypoint_idx < len(self.waypoints) - 1:
            self.current_waypoint_idx += 1
            self.logger.info("Proceeding to next waypoint")

            self.current_waypoint = self.waypoints[self.current_waypoint_idx]
            
            self.logger.info(f"Current waypoint: {self.current_waypoint[0]}, {self.current_waypoint[1]}")
        else:
            self.current_waypoint = None
            self.logger.info("No more waypoints")
