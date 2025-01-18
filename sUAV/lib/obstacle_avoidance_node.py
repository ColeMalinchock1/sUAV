#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import threading
import time
import math
import numpy as np

MAX_THRESHOLD = 5.0 #meters
current_position = current_orientation = current_waypoint = 0

def quaternion_to_euler_angle_vectorized2(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

def zed_pose_callback(msg):
    """Handle ZED pose messages."""
    global current_position, current_orientation

    # Extract position
    current_position = (
        msg.pose.position.x,
        msg.pose.position.y
    )
    
    # Extract orientation
    orientation_q = msg.pose.orientation
    _, _, yaw = quaternion_to_euler_angle_vectorized2(orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z)
    current_orientation = yaw

# def lidar_image_callback(msg):

def lidar_3d_callback(msg):
        """Handle incoming lidar messages."""

        # Initialize arrays for the two center rows
        center_row1_points = []
        center_row2_points = []
        
        # Extract the height and width
        height = msg.height
        width = msg.width
        
        # Define center rows
        center_row1 = height // 2
        center_row2 = center_row1 + 1 if height > 1 else center_row1

        # Reset state
        stopped = False
        obstacle_detected = False

        # Iterate through points with their indices
        for idx, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            x, y, z = point
            
            # Calculate row and column indices
            row = idx // width
            
            # Process only points from the two center rows
            if row == center_row1:
                depth = np.sqrt(x**2 + y**2 + z**2)
                angle = -math.degrees(np.arctan2(y, x))
                center_row1_points.append((depth, angle))
            elif row == center_row2:
                depth = np.sqrt(x**2 + y**2 + z**2)
                angle = -math.degrees(np.arctan2(y, x))
                center_row2_points.append((depth, angle))

        # Average the points from both rows
        depth_angle_array = []
        min_length = min(len(center_row1_points), len(center_row2_points))
        
        for i in range(min_length):
            avg_depth = (center_row1_points[i][0] + center_row2_points[i][0]) / 2
            avg_angle = (center_row1_points[i][1] + center_row2_points[i][1]) / 2
            if avg_depth == 0:
                avg_depth = 100.0
            
            
            if avg_depth < MAX_THRESHOLD and not stopped:
                obstacle_detected = True
                print(f"Depth: {avg_depth}, Angle: {avg_angle}")
                
            depth_angle_array.append((avg_depth, avg_angle))
        
        if obstacle_detected:
            print("OBSTACLE")
        

def main():

    rclpy.init()
    node = rclpy.create_node('obstacle_avoidance_node')
    node.create_subscription(PointCloud2, "/scan_3D", lidar_3d_callback, 10)
    node.create_subscription(PoseStamped, "/zed/zed_node/pose", zed_pose_callback, 10)

    node.create_publisher(Float64MultiArray, "/obstacle_avoidance/log", 1)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())

    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    