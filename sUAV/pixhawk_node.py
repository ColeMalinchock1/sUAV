#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool

from lib.rerouter import Rerouter as rr
from lib.pixhawk_commands import PixhawkCommands as commands

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
        self.MODE = "None"  # Made MODE an instance variable
        self.command_status = "OK"  # Track command status
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

    def handle_mode_transition(self, pixhawk, target_mode):
        """Handle mode transitions with verification"""
        if target_mode == "GUIDED":
            success, error = pixhawk.set_guided_mode()
        elif target_mode == "AUTO":
            success, error = pixhawk.set_auto_mode()
        else:
            return False, f"Unknown mode: {target_mode}"

        if not success:
            self.command_status = f"Mode change failed: {error}"
            return False
        self.command_status = "OK"
        return True

    def handle_movement(self, pixhawk, x, y, z):
        """Handle movement commands with verification"""
        success, error = pixhawk.move_to_relative_position(x, y, z)
        if not success:
            self.command_status = f"Movement failed: {error}"
            return False
        self.command_status = "OK"
        return True

    def main(self, args=None):
        rclpy.init(args=args)
        node = Node('pixhawk_node')
        
        # Publishers
        logger_publisher = node.create_publisher(String, '/pixhawk/logger', 1)
        latlon_publisher = node.create_publisher(Float64MultiArray, '/pixhawk/gps', 1)
        status_publisher = node.create_publisher(String, '/pixhawk/command_status', 1)

        # Subscribers
        obstacle_subscriber = node.create_subscription(
            Float64MultiArray, '/obstacle_avoidance/vector', 
            self.obstacle_detector_callback, 1)
        controller_mode_subscriber = node.create_subscription(
            String, '/controller/mode', 
            self.controller_mode_callback, 1)
        controller_direction_subscriber = node.create_subscription(
            String, '/controller/direction', 
            self.controller_direction_callback, 1)

        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()
        
        pixhawk = commands(serial_port='/dev/ttyTHS1', baud_rate=57600)
        previously_guided = False

        while rclpy.ok():                
            if self.controller_mode == "Auto":
                if self.MODE == "None":
                    success, error = pixhawk.store_current_mode()
                    if success:
                        self.adjusted_mode = True
                        self.command_status = "OK"
                    else:
                        self.command_status = f"Failed to store mode: {error}"
                
                if self.obstacle_detected == 2.0:
                    if self.handle_mode_transition(pixhawk, "ALT_HOLD"):
                        self.MODE = "STOPPED"
                        previously_guided = True
                
                elif self.obstacle_detected == 1.0:
                    if self.handle_mode_transition(pixhawk, "GUIDED"):
                        if self.handle_movement(pixhawk, 
                                             self.obstacle_avoidance_x,
                                             self.obstacle_avoidance_y,
                                             self.obstacle_avoidance_z):
                            previously_guided = True
                            self.MODE = "GUIDED"
                
                elif previously_guided:
                    if self.handle_mode_transition(pixhawk, "AUTO"):
                        previously_guided = False
                        self.MODE = "AUTO"
                else:
                    self.MODE = "AUTO"

            elif self.controller_mode == "GUIDED":
                if self.MODE == "None":
                    success, error = pixhawk.store_current_mode()
                    if success:
                        self.adjusted_mode = True
                        self.command_status = "OK"
                    else:
                        self.command_status = f"Failed to store mode: {error}"
                
                if self.handle_mode_transition(pixhawk, "GUIDED"):
                    self.MODE = "GUIDED"
                    if self.controller_direction == "Left":
                        self.handle_movement(pixhawk, 0, -1, 0)
                    elif self.controller_direction == "Right":
                        self.handle_movement(pixhawk, 0, 1, 0)
                    elif self.controller_direction == "Forward":
                        self.handle_movement(pixhawk, 1, 0, 0)
                    elif self.controller_direction == "Backward":
                        self.handle_movement(pixhawk, -1, 0, 0)
                
            else:
                if self.adjusted_mode:
                    success, error = pixhawk.restore_mode()
                    if success:
                        self.adjusted_mode = False
                        self.command_status = "OK"
                    else:
                        self.command_status = f"Failed to restore mode: {error}"
                self.MODE = "None"

            # Publish status
            status_msg = String()
            status_msg.data = self.command_status
            status_publisher.publish(status_msg)

            # Publish mode
            mode_msg = String()
            mode_msg.data = self.MODE
            logger_publisher.publish(mode_msg)

            # Publish GPS coordinates
            coordinate, error = pixhawk.get_current_latlon()
            if coordinate is not None:
                msg = Float64MultiArray()
                msg.data = coordinate
                latlon_publisher.publish(msg)
            
            self.stdscr.refresh()
            self.stdscr.addstr(1, 5, 'PIXHAWK NODE')
            self.stdscr.addstr(3, 5, f'Obstacle Detected: {str(self.obstacle_detected)}         ')
            self.stdscr.addstr(4, 5, f'Mode: {self.MODE}                      ')
            self.stdscr.addstr(5, 5, f'Status: {self.command_status}          ')

        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    PixhawkNode()