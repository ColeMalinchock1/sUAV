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
from std_msgs.msg import Float64MultiArray, String

# Personal libraries
from lib.rerouter import Rerouter as rr
from lib.pixhawk_commands import PixhawkCommands as commands

# Other libraries
import curses
import time
import threading

class PixhawkNode():
    def __init__(self):
        self.stdscr = curses.initscr()
        self.controller_mode = "None"
        self.controller_direction = None
        self.obstacle_detected = None
        self.obstacle_avoidance = None
        self.adjusted_mode = False
        self.main()

    def obstacle_detector_callback(self, msg):
        self.obstacle_detected = msg.data[0]
        self.obstacle_avoidance_x = msg.data[1]
        self.obstacle_avoidance_y = msg.data[2]
        self.obstacle_avoidance_z = msg.data[3]

    def controller_mode_callback(self, msg):
        self.controller_mode = msg.data

    def controller_direction_callback(self, msg):
        self.controller_direction = msg.data

    def main(self, args=None):

        rclpy.init(args=args)
        node = Node('pixhawk_node')
        
        logger_publisher = node.create_publisher(String, '/pixhawk/logger', 1)

        obstacle_subscriber = node.create_subscription(Float64MultiArray, '/obstacle_avoidance/vector', self.obstacle_detector_callback, 1)
        controller_mode_subscriber = node.create_subscription(String, '/controller/mode', self.controller_mode_callback, 1)
        controller_direction_subscriber = node.create_subscription(String, '/controller/direction', self.controller_direction_callback, 1)

        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()
        
        pixhawk = commands(serial_port='/dev/ttyTHS1', baud_rate=57600)
        
        previously_guided = False

        while rclpy.ok():                
            
            if self.controller_mode == "Auto":
                if MODE == "None":
                    pixhawk.store_current_mode()
                    self.adjusted_mode = True
                
                if self.obstacle_detected == 2.0:
                    pixhawk.set_guided_mode()
                    MODE = "STOPPED"
                    previously_guided = True
                elif self.obstacle_detected == 1.0:
                    pixhawk.set_guided_mode()

                    pixhawk.move_to_relative_position(
                        self.obstacle_avoidance_x,
                        self.obstacle_avoidance_y,
                        self.obstacle_avoidance_z
                    )
                    previously_guided = True
                    MODE = "GUIDED"
                elif previously_guided:
                    pixhawk.set_auto_mode()
                    previously_guided = False
                    MODE = "AUTO"
                else:
                    MODE = "AUTO"

            elif self.controller_mode == "GUIDED":
                if MODE == "None":
                    pixhawk.store_current_mode()
                    self.adjusted_mode = True
                pixhawk.set_guided_mode()
                MODE = "GUIDED"
                if self.controller_direction == "Left":
                    pixhawk.move_to_relative_position(0, -1, 0)
                elif self.controller_direction == "Right":
                    pixhawk.move_to_relative_position(0, 1, 0)
                elif self.controller_direction == "Forward":
                    pixhawk.move_to_relative_position(1, 0, 0)
                elif self.controller_direction == "Backward":
                    pixhawk.move_to_relative_position(-1, 0, 0)
                
            else:
                if self.adjusted_mode:
                    pixhawk.restore_mode()
                    self.adjusted_mode = False
                MODE = "None"

            msg = String()
            msg.data = MODE
            logger_publisher.publish(msg)
            
            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'PIXHAWK NODE')

            self.stdscr.addstr(3, 5, 'Obstacle Detected: %s         ' % str(self.obstacle_detected))
            self.stdscr.addstr(4, 5, 'Mode: %s                      ' % MODE)

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    PixhawkNode()
