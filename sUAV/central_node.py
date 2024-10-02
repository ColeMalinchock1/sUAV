#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

from lib.rerouter import Rerouter as rr
from lib.pixhawk_commands import PixhawkCommands as commands

import curses
import time

stdscr = curses.initscr()

opening_theta = 0.0
obstacle_detected = False
opening_detected = True

# 3 Different modes for getting around obstacles
# 1 - None
# 2 - Stop
# 3 - Move around
MODE = 1

def obstacle_detected_callback(msg):
    global obstacle_detected

    obstacle_detected = msg.data

def main(args=None):
    """
    Main function
    """

    rclpy.init(args=args)
    node = Node('main_node')
    obstacle_subscription = node.create_subscription(Bool, 'obstacle_detection', obstacle_detected_callback, 1)
    latlon_publisher = node.create_publisher(Float64MultiArray, 'lat_lon_topic', 1)

    FREQ = 20
    rate = node.create_rate(FREQ, node.get_clock())

    pixhawk = commands(serial_port='/dev/ttyTHS1', baud_rate=57600)

    condition = "CONTINUING MISSION"

    last_time = time.time()

    wait = False

    while rclpy.ok():

        if MODE == 2:
            if not wait:

                if obstacle_detected:
                    pixhawk.hold_position()
                    condition = "HOLDING POSITION"
                    last_time = time.time()
                    wait = True
                else:
                    pixhawk.resume_mission()
                    condition = "CONTINUING MISSION"
            else:
                if time.time() - last_time > 10:
                    wait = False

            
        
        coordinate = pixhawk.get_current_latlon()
        print(coordinate)
        
        if coordinate is not None:
            msg = Float64MultiArray()
            msg.data = coordinate
            latlon_publisher.publish(msg)

        rate.sleep()

        stdscr.refresh()
        stdscr.addstr(1, 5, 'CENTRAL NODE')

        stdscr.addstr(3, 5, 'Mode: %s         ' % str(MODE))
        stdscr.addstr(4, 5, 'Condition: %s      ' % condition)
        stdscr.addstr(5, 5, 'Wait: %s      ' % str(wait))
        stdscr.addstr(6, 5, 'Time: %s      ' % str(time.time() - last_time))

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()