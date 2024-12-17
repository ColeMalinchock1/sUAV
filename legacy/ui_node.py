import rclpy
from std_msgs.msg import Float64MultiArray
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses
import os

current_x = current_y = current_z = distance_to_waypoint = 0.0

waypoints = []

# Allows the user to input as many x and y points until they enter x. It then allows the user to double check the results
def user_input():
    global waypoints

    clean_x_input = clean_y_input = False

    # Continue getting inputs until x is input
    while True:
        x = input("X: ")
        if x == "x":
            break
        else:
            try:
                x = float(x)
                clean_x_input = True
            except ValueError:
                clean_x_input = False

        y = input("Y: ")
        if y == "x":
            break
        else:
            try:
                y = float(y)
                clean_y_input = True
            except ValueError:
                clean_y_input = False
        z = input("Z: ")
        if z == "x":
            break
        else:
            try:
                z = float(z)
                clean_z_input = True
            except ValueError:
                clean_z_input = False

        if clean_x_input and clean_y_input and clean_z_input:
            waypoints.append(x)
            waypoints.append(y)
            waypoints.append(z)
            print("Points added")
        else:
            print("Invalid input")

    # Double check that the waypoints are correct
    print(waypoints)
    all_good = input("Enter x if the points are incorrect: ")
    if all_good == "x":
        waypoints = []
        user_input()

def current_pos_callback(msg):
    global current_x, current_y, current_z

    current_x = msg.data[0]
    current_y = msg.data[1]
    current_z = msg.data[2]


def main():
    global waypoints, current_x, current_y, current_z

    rclpy.init()
    ui_node = rclpy.create_node('ui_node')

    pub_waypoints_to_pure_pursuit = ui_node.create_publisher(Float64MultiArray, "ui_topic", 1)  # publishing one value for now as a test, later change the data type and values
    sub_vehicle_current_position = ui_node.create_subscription(Float64MultiArray, "current_pos_topic", current_pos_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(ui_node,), daemon=True)
    thread.start()

    FREQ = 10
    rate = ui_node.create_rate(FREQ, ui_node.get_clock())

    print("Enter first location (Ex: 1.54)")
    print("Enter x when you are done")

    user_input()

    os.system('clear')

    stdscr = curses.initscr()

    while rclpy.ok():
        msg = Float64MultiArray()
        msg.data = waypoints
        pub_waypoints_to_pure_pursuit.publish(msg)
        stdscr.refresh()
        stdscr.addstr(1, 5, 'UI NODE')

        stdscr.addstr(3, 5, 'Waypoint X :  %.4f		         ' % float(waypoints[0]))
        stdscr.addstr(4, 5, 'Waypoint Y :  %.4f	                ' % float(waypoints[1]))
        stdscr.addstr(5, 5, 'Waypoint Z :  %.4f	                ' % float(waypoints[2]))

        stdscr.addstr(7, 5, 'Current X :  %.4f		         ' % float(current_x))
        stdscr.addstr(8, 5, 'Current Y :  %.4f	                ' % float(current_y))
        stdscr.addstr(9, 5, 'Current Z :  %.4f	                ' % float(current_z))

        stdscr.addstr(11, 5, 'Distance to waypoint :  %.4f            ' % float(math.sqrt((waypoints[0] - current_x)**2 + (waypoints[1] - current_y)**2 + (waypoints[2] - current_z)**2)))

        rate.sleep()

    ui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()