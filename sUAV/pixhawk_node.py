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
from std_msgs.msg import Bool, Float64MultiArray, String

# Personal libraries
from lib.rerouter import Rerouter as rr
from lib.pixhawk_commands import PixhawkCommands as commands

# Other libraries
import curses
import time

class PixhawkNode():
    def __init__(self):
        self.stdscr = curses.initscr()
        self.obstacle_detected = None
        self.obstacle_avoidance = None
        self.main()

    def pixhawk_commands_callback(self, msg):
        self.obstacle_detected = msg.data[0]
        self.obstacle_avoidance_x = msg.data[1]
        self.obstacle_avoidance_y = msg.data[2]
        self.obstacle_avoidance_z = msg.data[3]

    def main(self, args=None):

        rclpy.init(args=args)
        node = Node('pixhawk_node')
        
        logger_publisher = node.create_publisher(String, 'pixhawk_logger_topic', 1)

        controller_subscriber = node.create_subscription(Float64MultiArray, 'pixhawk_commands_topic', pixhawk_commands_callback, 1)

        FREQ = 20
        rate = node.create_rate(FREQ, node.get_clock())
        
        pixhawk = commands(serial_port='/dev/ttyTHS1', baud_rate=57600)

        last_time = time.time()
        
        previously_guided = False

        while rclpy.ok():                
            
            if self.obstacle_detected and time.time() - last_time > 1.0:
                pixhawk.set_guided_mode()

                pixhawk.move_to_relative_position(
                    self.obstacle_avoidance_x,
                    self.obstacle_avoidance_y,
                    self.obstacle_avoidance_z
                )
                previously_guided = True
                MODE = "GUIDED"
                last_time = time.time()
            elif previously_guided:
                pixhawk.set_auto_mode()
                previously_guided = False
                MODE = "AUTO"
            else:
                MODE = "AUTO"

            msg = String()
            msg.data = MODE
            logger_publisher.publish(msg)
            
            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'PIXHAWK NODE')

            self.stdscr.addstr(3, 5, 'Obstacle Detected: %s         ' % str(self.obstacle_detected))
            self.stdscr.addstr(4, 5, 'Mode: %s      ' % MODE)

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    PixhawkNode()
