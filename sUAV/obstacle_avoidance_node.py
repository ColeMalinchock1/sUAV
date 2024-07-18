
"""
This node is used to read the depth from the Zed and lidar and is capable of detecting if an obstacle is obstructing its current path.
It creates a new waypoint in the case of needing to route around an obstacle.
"""

#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from lib.rerouter import Rerouter as rr

import numpy as np
import threading
import curses

stdscr = curses.initscr()

scan_array = None

THRESHOLD = 2 # meters

def scan_2d_callback(msg):
    """Callback for lidar 2d scan"""

    global scan_array

    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        depth = point[0]  # Assuming 'z' is the depth information
        points.append(depth)
    
    # Convert the list to a numpy array for further processing
    scan_array = np.array(points)

def main(args=None):
    
    rclpy.init(args=args)
    node = Node("object_avoidance_node")
    scan_subscription = node.create_subscription(PointCloud2, '/scan_2D', scan_2d_callback, 5)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    # Initialize the messages
    obstacle_detected = False
    direction = "Straight"

    while rclpy.ok():

        # Checks if the scan has been received
        # Else return that scan is not received
        if scan_array is not None:

            # Sets received as true
            receiving_scan = True

            # Initializes the obstacle detected as false
            obstacle_detected = False
            
            # Loops through the scan_array
            for i in range(scan_array.size):
                
                # Creates a quarter of the width of the scan
                quarter_width = scan_array.size / 4

                # Checks if any of the values are less than the threshold
                if scan_array[i] < THRESHOLD:

                    # Sets the obstacle detected as True
                    obstacle_detected = True

                    # Checks if the obstacle is a quarter width to the left, right,
                    if i < quarter_width:
                        direction = "Right"
                    elif i > 3 * quarter_width:
                        direction = "Left"
                    else:
                        direction = "Stop"

        else:
            receiving_scan = False

        rate.sleep()

        stdscr.refresh()
        stdscr.addstr(1, 5, 'NODE OBSTACLE AVOIDANCE')

        stdscr.addstr(3, 5, 'Receiving Scan: %s         ' % str(receiving_scan))
        stdscr.addstr(4, 5, 'Obstacle Detected: %s      ' % str(obstacle_detected))
        stdscr.addstr(5, 5, 'Move To The: %s            ' % str(direction))

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()