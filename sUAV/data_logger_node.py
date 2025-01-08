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

FIELD_NAMES = ["TIME", "MODE", "Obstacle Detected", "X Offset", "Y Offset", "Z Offset"]
FILE = "/home/emrl-3172/ros2_ws/src/sUAV/log/data_log.csv"
IMAGE_DIR = "/home/emrl-3172/ros2_ws/src/sUAV/log/images/"

class DataRecorderNode():

    def __init__(self):
        self.receiving_pixhawk = self.saving_data = self.receiving_obstacle_avoidance = False
        self.mode = None
        self.stdscr = curses.initscr()
        self.main()

    def pixhawk_mode_callback(self, msg):
        self.receiving_pixhawk = True

        self.mode = msg.data

    def pixhawk_command_callback(self, msg):
        self.receiving_obstacle_avoidance = True

        self.obstacle_detected = msg.data[0]
        self.obstacle_avoidance_x = msg.data[1]
        self.obstacle_avoidance_y = msg.data[2]
        self.obstacle_avoidance_z = msg.data[3]

    def log_data(self, data):
        """Logs the data from this node to be saved and analyzed after collecting the data"""

        # Saves the data as a dictionary
        saved_data = {
            FIELD_NAMES[0]: data[0],
            FIELD_NAMES[1]: data[1],
            FIELD_NAMES[2]: data[2],
            FIELD_NAMES[3]: data[3],
            FIELD_NAMES[4]: data[4],
            FIELD_NAMES[5]: data[5]
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
        pixhawk_subscription = node.create_subscription(String, 'pixhawk_logger_topic', self.pixhawk_mode_callback, 1)
        obstacle_subscription = node.create_subscription(Float64MultiArray, '/pixhawk_commands_topic', self.pixhawk_command_callback, 1)

        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        FREQ = 20
        rate = node.create_rate(FREQ, node.get_clock())

        while rclpy.ok():

            if self.receiving_pixhawk and self.receiving_obstacle_avoidance:
                
                self.saving_data = True

                timestamp = time.time()

                data = [timestamp, self.mode, self.obstacle_detected, self.obstacle_avoidance_x, self.obstacle_avoidance_y, self.obstacle_avoidance_z]

                self.log_data(data)

            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'DATA LOGGER NODE')

            self.stdscr.addstr(3, 5, 'Receiving Pixhawk: %s         ' % str(self.receiving_pixhawk))
            self.stdscr.addstr(4, 5, 'Receiving Obstacle Avoidance: %s         ' % str(self.receiving_obstacle_avoidance))
            self.stdscr.addstr(5, 5, 'Saving Data: %s         ' % str(self.saving_data))
            
            # rate.sleep()

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    DataRecorderNode()
