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
from rerouter import Rerouter
import time

THRESHOLD = 1.0  # meters
TIMEOUT = 3.0

class ObstacleDetectorNode():
    def __init__(self):
        self.stdscr = curses.initscr()
        self.scan_array = self.y_array = None
        self.last_scan_time = time.time()
        self.rerouter = Rerouter()
        self.main()

    def scan_3d_callback(self, msg):
        """Callback for lidar 3d scan - extracts center rows"""
        points_dict = {}  # Dictionary to store points by row
        
        # Read all points and organize by vertical position (z)
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            z_key = round(point[2], 2)  # Round z to group similar heights
            if z_key not in points_dict:
                points_dict[z_key] = []
            points_dict[z_key].append((point[0], point[1]))  # Store x,y pairs

        # Sort z values and get the middle rows
        z_values = sorted(points_dict.keys())
        if len(z_values) >= 2:
            middle_idx = len(z_values) // 2
            z1, z2 = z_values[middle_idx-1:middle_idx+1]  # Get two center rows
            
            # Combine points from both center rows
            center_points = points_dict[z1] + points_dict[z2]
            
            # Extract x (depth) and y values
            points = [p[0] for p in center_points]  # x values (depth)
            y_distances = [p[1] for p in center_points]  # y values
            
            # Convert lists to numpy arrays
            self.scan_array = np.array(points)
            self.y_array = np.array(y_distances)
            self.last_scan_time = time.time()

    def main(self, args=None):
        rclpy.init(args=args)
        node = Node("object_avoidance_node")
        
        # Change subscription to scan_3D
        scan_subscription = node.create_subscription(
            PointCloud2, 
            '/scan_3D', 
            self.scan_3d_callback, 
            5
        )
        
        scan_publisher = node.create_publisher(
            Float64MultiArray, 
            '/obstacle_avoidance/vector', 
            1
        )
        
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()
        rate = node.create_rate(20, node.get_clock())
        
        while rclpy.ok():
            obstacle_message = [0.0, 0.0, 0.0, 0.0]
            receiving_scan = False
            
            if time.time() - self.last_scan_time < TIMEOUT and self.scan_array is not None:
                receiving_scan = True
                
                # Check for obstacles in the combined center rows
                for i in range(self.scan_array.size):
                    if self.scan_array[i] < THRESHOLD:
                        theta = self.rerouter.obstacle_detected(
                            self.scan_array, 
                            self.y_array
                        )
                        if theta == "STOP":
                            obstacle_message = [2.0, 0.0, 0.0, 0.0]
                        else:
                            obstacle_message = [
                                1.0,
                                math.sin(math.radians(theta)),
                                math.cos(math.radians(theta)),
                                0.0
                            ]
                        break
            
            msg = Float64MultiArray()
            msg.data = obstacle_message
            scan_publisher.publish(msg)
            rate.sleep()
            
            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'OBSTACLE AVOIDANCE NODE')
            self.stdscr.addstr(3, 5, 'Receiving Scan: %s ' % str(receiving_scan))
            self.stdscr.addstr(4, 5, 'Obstacle Detected: %s ' % str(obstacle_message[0]))
            self.stdscr.addstr(5, 5, 'X Adjust: %s ' % str(obstacle_message[1]))
            self.stdscr.addstr(6, 5, 'Y Adjust: %s ' % str(obstacle_message[2]))
            self.stdscr.addstr(7, 5, 'Z Adjust: %s ' % str(obstacle_message[3]))

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    ObstacleDetectorNode()