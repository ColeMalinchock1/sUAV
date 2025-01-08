
"""
This node is used to read the depth from the Zed and lidar and is capable of detecting if an obstacle is obstructing its current path.
It creates a new waypoint in the case of needing to route around an obstacle.
"""

#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray

import numpy as np
import threading
import curses
import math
from lib.rerouter import Rerouter
import time

THRESHOLD = 1.0 # meters
TIMEOUT = 3.0

class ObstacleDetectorNode():

    def __init__(self):
        self.stdscr = curses.initscr()
        self.scan_array = self.y_array = None
        self.last_scan_time = time.time()
        self.rerouter = Rerouter()
        self.main()

    def scan_2d_callback(self, msg):
        """Callback for lidar 2d scan"""

        points = []
        y_distances = []
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            depth = point[0]  # Assuming 'x' is the depth information
            points.append(depth)
            y_distances.append(point[1])
            
        # Convert the list to a numpy array for further processing
        self.scan_array = np.array(points)
        self.y_array = np.array(y_distances)

        self.last_scan_time = time.time()

    def main(self, args=None):
        
        rclpy.init(args=args)
        node = Node("object_avoidance_node")
        scan_subscription = node.create_subscription(PointCloud2, '/scan_2D', self.scan_2d_callback, 5)
        scan_publisher = node.create_publisher(Float64MultiArray, '/pixhawk_commands_topic', 1)

        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        rate = node.create_rate(20, node.get_clock())

        offset = 20

        while rclpy.ok():

            obstacle_message = [0.0, 0.0, 0.0, 0.0]

            # Checks if the scan has been received
            # Else return that scan is not received
            if time.time() - self.last_scan_time < TIMEOUT and self.scan_array is not None:

                # Sets received as true
                receiving_scan = True
                
                # Loops through the scan_array
                for i in range(self.scan_array.size):
                    
                    # Checks if any of the values are less than the threshold
                    if self.scan_array[i] < THRESHOLD:
                        theta = self.rerouter.obstacle_detected(self.scan_array, self.y_array)
                        
                        if theta == "STOP":
                            obstacle_message = [2.0, 0.0, 0.0, 0.0]
                        else:
                            obstacle_message = [1.0, math.sin(math.radians(theta)), math.cos(math.radians(theta)), 0.0]
                        
                        break

            else:
                receiving_scan = False

            msg = Float64MultiArray()
            msg.data = obstacle_message
            scan_publisher.publish(msg)

            rate.sleep()

            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'OBSTACLE AVOIDANCE NODE')

            self.stdscr.addstr(3, 5, 'Receiving Scan: %s         ' % str(receiving_scan))
            self.stdscr.addstr(4, 5, 'Obstacle Detected: %s      ' % str(obstacle_message[0]))
            self.stdscr.addstr(5, 5, 'X Adjust: %s         ' % str(obstacle_message[1]))
            self.stdscr.addstr(6, 5, 'Y Adjust: %s         ' % str(obstacle_message[2]))
            self.stdscr.addstr(7, 5, 'Z Adjust: %s         ' % str(obstacle_message[3]))

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    ObstacleDetectorNode()