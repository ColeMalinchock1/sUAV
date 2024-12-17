#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2 as cv

import math
import os
import csv
import time
import curses
import threading

receiving_gps = receiving_pose = receiving_frame = saving_data = False

pose = coordinate = frame = None

FIELD_NAMES = ["timestamp", "zed x", "zed y", "zed z", "zed yaw", "lat", "lon", "alt", "heading"]
FILE = "/home/emrl-3172/ros2_ws/src/sUAV/log/data_log.csv"
IMAGE_DIR = "/home/emrl-3172/ros2_ws/src/sUAV/log/images/"

br = CvBridge()

def image_callback(msg):
    global frame, receiving_frame

    frame = br.imgmsg_to_cv2(msg)

    receiving_frame = True


def pose_callback(msg):

    global pose, receiving_pose

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
    pose = [zed_x, zed_y, zed_z, zed_yaw]
    receiving_pose = True

def lat_lon_callback(msg):
    
    global coordinate, receiving_gps

    lat = msg.data[0]
    lon = msg.data[1]
    alt = msg.data[2]
    heading = msg.data[3]

    coordinate = [lat, lon, alt, heading]
    receiving_gps = True


def log_data(data):
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
        FIELD_NAMES[8]: data[8]
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

def main(args=None):
    
    """
    Main function
    """
    
    print("Beginning")

    rclpy.init(args=args)
    node = Node('data_node')
    lat_lon_subscription = node.create_subscription(Float64MultiArray, 'lat_lon_topic', lat_lon_callback, 1)
    zed_subscription = node.create_subscription(PoseStamped, '/zed/zed_node/pose', pose_callback, 1)
    frame_subscription = node.create_subscription(Image, '/zed/zed_node/left/image_rect_color', image_callback, 1)

    FREQ = 20
    rate = node.create_rate(FREQ, node.get_clock())

    previous_coordinate = None

    count = 0

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    while rclpy.ok():

        if previous_coordinate != coordinate and pose is not None and coordinate is not None:
            timestamp = time.time()

            data = [timestamp, pose[0], pose[1], pose[2], pose[3], coordinate[0], coordinate[1], coordinate[2], coordinate[3]]

            log_data(data)

            previous_coordinate = coordinate

        if frame is not None:
            cv.imwrite(IMAGE_DIR + str(count) + ".jpg", frame)
            count += 1

        print("Receiving gps: ", str(receiving_gps))
        print("Receiving pose: ", str(receiving_pose))
        print("Receiving frames: ", str(receiving_frame))
        print("Saving data: ", str(saving_data))
        
        # rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
