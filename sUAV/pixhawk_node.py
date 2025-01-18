#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from dronekit import connect, VehicleMode
import time
import argparse
import logging
import numpy as np
from datetime import datetime

from lib.obstacle_detector import ObstacleDetector

class DroneControlNode(Node):
    def __init__(self, connection_string):
        super().__init__('drone_control_node')
        
        # Initialize the obstacle detector
        self.obstacle_detector = ObstacleDetector()

        # Set up logging
        self.setup_logging()
        
        # Connect to the drone
        self.get_logger().info(f"Connecting to vehicle on: {connection_string}")
        self.vehicle = connect(connection_string, wait_ready=True, baud=57600, timeout=60)
        
        # Create subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/scan_3D',
            self.lidar_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('target_altitude', 2.0)
        self.target_altitude = self.get_parameter('target_altitude').value
        
        self.get_logger().info("Drone control node initialized")

    def setup_logging(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        logging.basicConfig(
            filename=f'drone_log_{timestamp}.log',
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        logging.getLogger('').addHandler(console)

    def arm_and_takeoff(self):
        """Arms vehicle and fly to target altitude."""
        self.get_logger().info("Basic pre-arm checks")
        
        while not self.vehicle.is_armable:
            self.get_logger().info("Waiting for vehicle to initialize...")
            time.sleep(1)
        
        self.get_logger().info("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        
        while self.vehicle.gps_0.fix_type < 3:
            self.get_logger().info("Waiting for GPS lock...")
            time.sleep(1)
            
        while not self.vehicle.armed:
            self.get_logger().info("Waiting for arming...")
            time.sleep(1)
            
        self.get_logger().info("Taking off!")
        self.vehicle.simple_takeoff(self.target_altitude)
        
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            self.get_logger().info(f"Altitude: {current_altitude}")
            if current_altitude >= self.target_altitude * 0.95:
                self.get_logger().info("Reached target altitude")
                break
            time.sleep(1)

    def run_mission(self):
        """Executes the pre-loaded mission."""
        self.get_logger().info("Starting pre-loaded mission")
        self.vehicle.mode = VehicleMode("AUTO")
        
        while not self.vehicle.mode.name == "AUTO":
            self.get_logger().info("Waiting for AUTO mode...")
            time.sleep(1)
            
        self.get_logger().info("AUTO mode confirmed - executing mission")
        
        while True:
            if self.vehicle.mode.name != "AUTO":
                self.get_logger().info("Mission completed")
                break
            self.get_logger().info(f"Current altitude: {self.vehicle.location.global_relative_frame.alt}")
            time.sleep(2)

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

        # Initialize the stopped and obstacle detected boolean
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
                self._initial_obstacle_detected()
                
            depth_angle_array.append((avg_depth, avg_angle))
        
    def _initial_obstacle_detected(self):
        """Ran as soon as an obstacle is initially detected"""

        # Stops the vehicle and sets the self.stopped to true
        self.get_logger().info("Obstacle detected, stopping to scan")
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.stopped = True


    def cleanup(self):
        """Cleanup function called on node shutdown."""
        self.get_logger().info("Shutting down drone control node")
        if self.vehicle:
            self.vehicle.close()

def main():
    rclpy.init()
    
    # Parse connection string
    parser = argparse.ArgumentParser(description='ROS2 Drone Control Node')
    parser.add_argument('--connect', 
                       help="Vehicle connection target string",
                       default='/dev/ttyTHS1')
    args = parser.parse_args()
    
    # Create and spin node
    try:
        node = DroneControlNode(args.connect)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.cleanup()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()