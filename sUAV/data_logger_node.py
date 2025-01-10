#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import threading

from cv_bridge import CvBridge
import cv2 as cv

import math
import os
import csv
import time
import curses

FIELD_NAMES = ["TIME", "MODE", "Obstacle Detected", "X Offset", "Y Offset", "Z Offset", "Mode", "Direction", "ZED X", "ZED Y", "ZED Z", "ZED YAW", "GPS LAT", "GPS LON", "GPS ALT", "GPS HEADING"]
FILE = "/home/emrl-3172/ros2_ws/src/sUAV/log/data_log.csv"
IMAGE_DIR = "/home/emrl-3172/ros2_ws/src/sUAV/log/images/"

class DataRecorderNode():

    def __init__(self):
        self.receiving_pixhawk = self.saving_data = self.receiving_obstacle_avoidance = self.receiving_controller = self.receiving_gps = self.receiving_pose = False
        self.mode = self.controller_direction = self.controller_mode = self.coordinate = self.pose = None
        self.stdscr = curses.initscr()
        self.main()

    def pixhawk_mode_callback(self, msg):
        self.receiving_pixhawk = True

        self.mode = msg.data

    def vector_callback(self, msg):
        self.receiving_obstacle_avoidance = True

        self.obstacle_detected = msg.data[0]
        self.obstacle_avoidance_x = msg.data[1]
        self.obstacle_avoidance_y = msg.data[2]
        self.obstacle_avoidance_z = msg.data[3]

    def controller_mode_callback(self, msg):
        self.controller_mode = msg.data
        self.receiving_controller = True

    def controller_direction_callback(self, msg):
        self.controller_direction = msg.data
        self.receiving_controller = True

    def pose_callback(self, msg):

        # Set the positional data to be saved from the zed node from the message
        zed_x = msg.pose.position.x
        zed_y = msg.pose.position.y
        zed_z = msg.pose.position.z

        # Get the pose orientation in quaternion from the message
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w

        # Calculate the yaw of the zed from the quaternion variables
        zed_yaw = math.degrees(math.atan2(2.0 * (q_y * q_z + q_w * q_x), q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z))

        # Sets the zed received to True
        self.pose = [zed_x, zed_y, zed_z, zed_yaw]
        self.receiving_pose = True

    def lat_lon_callback(self, msg):

        lat = msg.data[0]
        lon = msg.data[1]
        alt = msg.data[2]
        heading = msg.data[3]

        self.coordinate = [lat, lon, alt, heading]
        self.receiving_gps = True

    def log_data(self, data):
        """Logs the data from this node to be saved and analyzed after collecting the data"""

        # Saves the data as a dictionary
        saved_data = {
            FIELD_NAMES[0]: data[0],
            FIELD_NAMES[1]: data[1],
            FIELD_NAMES[2]: data[2],
            FIELD_NAMES[3]: data[3],
            FIELD_NAMES[4]: data[4],
            FIELD_NAMES[5]: data[5],
            FIELD_NAMES[6]: data[6],
            FIELD_NAMES[7]: data[7],
            FIELD_NAMES[8]: data[8],
            FIELD_NAMES[9]: data[9],
            FIELD_NAMES[10]: data[10],
            FIELD_NAMES[11]: data[11],
            FIELD_NAMES[12]: data[12],
            FIELD_NAMES[13]: data[13],
            FIELD_NAMES[14]: data[14],
            FIELD_NAMES[15]: data[15]
        }

        # Checks if the file exists already
        file_exists = os.path.isfile(FILE)

        # Opens the file
        with open(FILE, 'a', newline='') as csvfile:

            # Makes a writer for a dictionary and sets the field names
            writer = csv.DictWriter(csvfile, fieldnames=FIELD_NAMES)

            # Checks if the file does not exist and then writes a header for it
            if not file_exists:
                writer.writeheader()

            # Writes a row on the csv with the data
            writer.writerow(saved_data)

    def main(self, args=None):
        """
        Main function
        """

        rclpy.init(args=args)
        node = Node('data_logger_node')
        lat_lon_subscription = node.create_subscription(Float64MultiArray, '/pixhawk/gps', self.lat_lon_callback, 1)
        zed_subscription = node.create_subscription(PoseStamped, '/zed/zed_node/pose', self.pose_callback, 1)
        pixhawk_subscription = node.create_subscription(String, '/pixhawk/logger', self.pixhawk_mode_callback, 1)
        obstacle_subscription = node.create_subscription(Float64MultiArray, '/obstacle_avoidance/vector', self.vector_callback, 1)
        mode_subscription = node.create_subscription(String, '/controller/mode', self.controller_mode_callback, 1)
        direction_subscription = node.create_subscription(String, '/controller/direction', self.controller_direction_callback, 1)

        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        FREQ = 20
        rate = node.create_rate(FREQ, node.get_clock())

        while rclpy.ok():
            
            

            if self.receiving_pixhawk and self.receiving_obstacle_avoidance and self.receiving_controller and self.receiving_gps and self.receiving_pose:
                
                self.saving_data = True

                timestamp = time.time()

                data = [timestamp, self.mode, self.obstacle_detected, self.obstacle_avoidance_x, self.obstacle_avoidance_y, self.obstacle_avoidance_z, self.controller_mode, self.controller_direction, self.pose[0], self.pose[1], self.pose[2], self.pose[3], self.coordinate[0], self.coordinate[1], self.coordinate[2], self.coordinate[3]]

                self.log_data(data)

                

            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'DATA LOGGER NODE')

            self.stdscr.addstr(3, 5, 'Receiving Pixhawk: %s                    ' % str(self.receiving_pixhawk))
            self.stdscr.addstr(4, 5, 'Receiving Obstacle Avoidance: %s         ' % str(self.receiving_obstacle_avoidance))
            self.stdscr.addstr(5, 5, 'Receiving Controller: %s                 ' % str(self.receiving_controller))
            self.stdscr.addstr(6, 5, 'Receiving GPS: %s                 ' % str(self.receiving_gps))
            self.stdscr.addstr(7, 5, 'Receiving ZED: %s                 ' % str(self.receiving_pose))
            self.stdscr.addstr(8, 5, 'Saving Data: %s         ' % str(self.saving_data))
            
            # rate.sleep()

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    DataRecorderNode()
