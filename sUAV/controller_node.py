#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import threading
import curses

class ControllerNode():
    
    def __init__(self):
        self.stdscr = curses.initscr()

        # Steering should be bounded between [-100, +100]
        self.mode = None
        self.direction = None
          
        self.main()

    def joy_callback(self, msg):

        # Check what buttons are pressed and set it to the corresponding mode
        # A is auto
        if (msg.buttons[0] == 1):
            self.mode = "Auto"
        elif (msg.buttons[1] == 1):
            self.mode = "Guided"
        elif (msg.buttons[2] == 1):
            self.mode = "None"
            
        # Gets the direction from the pad
        if(msg.buttons[3] == 1):
            self.direction = "Left"
        elif(msg.buttons[4] == 1):
            self.direction = "Right"
        elif(msg.buttons[4] == 1):
            self.direction = "Forward"
        elif(msg.buttons[4] == 1):
            self.direction = "Backward"

    def main(self, args=None):

        rclpy.init(args=args)
        node = Node("xbox_controller_node")

        # Subscription to joy topic - gets info from controller
        joy_sub = node.create_subscription(Joy, "joy", joy_callback, 5)
        # Publishers to manual throttle and steering - publishes bounded number pre-PWM
        mode_pub = node.create_publisher(String, "/controller/mode", 1)
        direction_pub = node.create_publisher(String, "/controller/direction", 1)

        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        rate = node.create_rate(20, node.get_clock())
            
        while rclpy.ok():

            try:
                # If we're in manual mode, we send the actuation
                mode_msg = String()
                mode_msg.data = self.mode

                direction_msg = String()
                direction_msg.data = self.direction

                mode_pub.publish(mode_msg)
                direction_pub.publish(direction_msg)
                
                self.stdscr.refresh()
                self.stdscr.addstr(1, 25, 'Xbox Controller Node')
                self.stdscr.addstr(2, 25, 'Throttle: %s          ' % self.mode)
                self.stdscr.addstr(3, 25, 'Steering: %s          ' % self.direction)
                rate.sleep()
            except KeyboardInterrupt:
                curses.endwin()
                print("Ctrl+C captured, ending...")
                break
        
        rclpy.shutdown()

def start():
    ControllerNode()

if __name__ == '__main__':
    start()