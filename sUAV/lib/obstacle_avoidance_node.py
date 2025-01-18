#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import threading
import time
import math
import numpy as np

def zed_callback(self, msg):
    """Handle ZED pose messages."""
    # Extract position
    self.current_position = (
        msg.pose.position.x,
        msg.pose.position.y
    )
    
    # Extract orientation
    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    self.current_orientation = yaw

def lidar_callback(self, msg):
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
        self.stopped = False
        self.obstacle_detected = False

        # Iterate through points with their indices
        for idx, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            x, y, z = point
            
            # Calculate row and column indices
            row = idx // width
            
            # Process only points from the two center rows
            if row == center_row1:
                depth = np.sqrt(x**2 + y**2 + z**2)
                angle = np.arctan2(y, x)
                center_row1_points.append((depth, angle))
            elif row == center_row2:
                depth = np.sqrt(x**2 + y**2 + z**2)
                angle = np.arctan2(y, x)
                center_row2_points.append((depth, angle))

        # Average the points from both rows
        depth_angle_array = []
        min_length = min(len(center_row1_points), len(center_row2_points))
        
        for i in range(min_length):
            avg_depth = (center_row1_points[i][0] + center_row2_points[i][0]) / 2
            avg_angle = (center_row1_points[i][1] + center_row2_points[i][1]) / 2
            
            if avg_depth < self.MAX_THRESHOLD and not self.stopped:
                self.obstacle_detected = True
                
            depth_angle_array.append((avg_depth, avg_angle))
        
        # Initialize target_yaw as None
        target_yaw = None
            
        if self.obstacle_detected and self.current_waypoint is not None:
            # Calculate avoidance maneuver
            target_yaw = self.obstacle_avoider.calculate_avoidance_yaw(
                depth_angle_array,
                self.current_waypoint
            )

        # Save the data
        self.save_data(depth_angle_array, target_yaw)

def main():

    rclpy.init()
    node = rclpy.create_node('obstacle_avoidance_node')
    node.create_subscription(PointCloud2, "/scan_3D", lidar_callback, 10)
    node.create_subscription(PoseStamped, "/zed/pose", zed_callback, 10)

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
    