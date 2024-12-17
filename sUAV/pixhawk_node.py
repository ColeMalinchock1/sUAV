#!/usr/bin/env python

# Module Docstring
"""
Module Name: pixhawk_node.py
Description: ROS2 node that receives information and sends commands with the pixhawk
Author: Cole Malinchock
Date: 12/17/2024
"""

# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

# Personal libraries
from lib.rerouter import Rerouter as rr
from lib.pixhawk_commands import PixhawkCommands as commands

# Other libraries
import curses
import time

class PixhawkNode():
    def __init__(self):
        self.stdscr = curses.initscr()
        self.pixhawk_commands = []
        self.pixhawk_data = []
        self.main()

    def pixhawk_commands_callback(self, msg):
        self.pixhawk_commandsi = msg.data

    def main(self, args=None):

        rclpy.init(args=args)
        node = Node('pixhawk_node')
        
        logger_publisher = node.create_publisher(Float64MultiArray, 'pixhawk_logger_topic', 1)
        controller_publisher = node.create_publisher(Float64MultiArray, 'pixhawk_controller_topic', 1)

        controller_subscriber = node.create_subscription(Float64MultiArray, 'pixhawk_commands_topic', pixhawk_commands_callback, 1)

        FREQ = 20
        rate = node.create_rate(FREQ, node.get_clock())
        
        pixhawk = commands(serial_port='/dev/ttyTHS1', baud_rate=57600)

        last_time = time.time()
        
        last_error = None

        while rclpy.ok():                
            
            coordinate, msg = pixhawk.get_current_latlon()
            
            if coordinate:

                gps_info = pixhawk.get_gps_info()
                if 

                msg = Float64MultiArray()
                msg.data = coordinate
            
            # rate.sleep()
            # print("HERE")
            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'CENTRAL NODE')

            self.stdscr.addstr(3, 5, 'Mode: %s         ' % str(MODE))
            self.stdscr.addstr(4, 5, 'Condition: %s      ' % condition)
            self.stdscr.addstr(5, 5, 'Wait: %s      ' % str(wait))
            self.stdscr.addstr(6, 5, 'Time: %s      ' % str(time.time() - last_time))

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    PixhawkNode()
