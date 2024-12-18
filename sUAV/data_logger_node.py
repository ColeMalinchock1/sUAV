#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2 as cv

import math
import os
import csv
import time
import curses

FIELD_NAMES = ["TIME", "MODE"]
FILE = "/home/emrl-3172/ros2_ws/src/sUAV/log/data_log.csv"
IMAGE_DIR = "/home/emrl-3172/ros2_ws/src/sUAV/log/images/"

class DataRecorderNode():

    def __init__(self):
        self.receiving_pixhawk = self.saving_data = False
        self.mode = None
        self.stdscr = curses.initscr()
        self.main()

    def pixhawk_callback(self, msg):
        self.receiving_pixhawk = True

        self.mode = msg.data

    def log_data(self, data):
        """Logs the data from this node to be saved and analyzed after collecting the data"""

        # Saves the data as a dictionary
        saved_data = {
            FIELD_NAMES[0]: data[0],
            FIELD_NAMES[1]: data[1]
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
        pixhawk_subscription = node.create_subscription(String, 'pixhawk_logger_topic', pixhawk_callback, 1)

        FREQ = 20
        rate = node.create_rate(FREQ, node.get_clock())

        while rclpy.ok():

            if self.receiving_pixhawk:
                timestamp = time.time()

                data = [timestamp, self.mode]

                self.log_data(data)

            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'DATA LOGGER NODE')

            self.stdscr.addstr(3, 5, 'Receiving Pixhawk: %s         ' % str(self.receiving_pixhawk))
            self.stdscr.addstr(4, 5, 'Saving Data: %s         ' % str(self.saving_data))
            
            rate.sleep()

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    DataRecorderNode()
