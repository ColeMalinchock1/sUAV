#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Simple LIDAR5B reader
import serial
import numpy as np
import time

from sUAV.lib.constants import *

class LIDAR:

    def __init__(self):

        # Initialize global variables
        self.buffercounter, self.cpc, self.lengthLSB, self.lengthMSB, self.data_length = 0, 0, 0, 0, 0
        self.step = HEADER1
        self.receivedData = []

        # Define thresholds for filtering
        self.min_valid_distance = 200  # Minimum valid distance (mm)
        self.max_valid_distance = 16000  # Maximum valid distance (mm)

        self.ser = self.angles = None

        self.lidar_setup()

    def parser(self, data):

        if self.step != CHECKSUM:  # CPC is a variable for storing checksum
            self.cpc = self.cpc ^ data

        if self.step == HEADER1 and data == NORMAL_MODE:
            self.step = HEADER2
            
        elif self.step == HEADER2 and data == PRODUCT_CODE:
            self.step = HEADER3
            
        elif self.step == HEADER3 and data == DEFAULT_ID:
            self.step = LENGTH_LSB
            self.cpc = 0
            
        elif self.step == LENGTH_LSB:
            self.step = LENGTH_MSB
            self.lengthLSB = data
            
        elif self.step == LENGTH_MSB:
            self.step = PAYLOAD_HEADER
            self.lengthMSB = data
            self.data_length = ((self.lengthMSB << 8) & 0xff00) | (self.lengthLSB & 0x00ff)
            
        elif self.step == PAYLOAD_HEADER:
            self.step = PAYLOAD_DATA
            if self.data_length == 1:
                self.step = CHECKSUM
            self.buffercounter = 0
            self.receivedData = []
            
        elif self.step == PAYLOAD_DATA:
            self.receivedData.append(data)
            self.buffercounter = self.buffercounter+1
            if self.buffercounter >= self.data_length - 1:
                self.step = CHECKSUM
                
        elif self.step == CHECKSUM:
            self.step = HEADER1
            
            if self.cpc == data:
                return True
        else:
            self.step = HEADER1
            return False

    def filter_and_analyze_lidar_data(self, scan_data, angles):
        """
        Filter LIDAR data to remove invalid readings and identify obstacle boundaries.
        
        Args:
            scan_data: List of distance measurements
            angles: List of corresponding angles
            
        Returns:
            Tuple of (filtered_data, obstacle_info)
            where filtered_data is a list of (angle, distance) pairs,
            and obstacle_info is a dict containing obstacle boundary information
        """
        
        filtered_data = []
        valid_readings = []  # Store valid readings for obstacle detection
        
        # First pass: filter data
        for i, distance in enumerate(scan_data):
            if i < len(angles):
                if self.min_valid_distance <= distance < self.max_valid_distance:
                    filtered_data.append((angles[i], distance))
                    valid_readings.append((angles[i], distance))
                else:
                    # Add the angle with None distance to maintain the scan pattern
                    filtered_data.append((angles[i], None))
        
        # Initialize obstacle info
        obstacle_info = {
            "detected": False,
            "left_angle": None,
            "right_angle": None,
            "center_angle": None,
            "min_distance": None,
            "width_degrees": None
        }
        
        # Second pass: detect obstacle boundaries if we have valid readings
        if valid_readings:
            obstacle_info["detected"] = True
            
            # Sort by angle to ensure consecutive angles
            valid_readings.sort(key=lambda x: x[0])
            
            # Find the minimum distance and its corresponding angle
            min_distance_reading = min(valid_readings, key=lambda x: x[1])
            obstacle_info["min_distance"] = min_distance_reading[1]
            obstacle_info["center_angle"] = min_distance_reading[0]
            
            # Find left and right boundaries
            obstacle_info["left_angle"] = valid_readings[0][0]
            obstacle_info["right_angle"] = valid_readings[-1][0]
            
            # Calculate width in degrees
            obstacle_info["width_degrees"] = abs(obstacle_info["right_angle"] - obstacle_info["left_angle"])
        
        return filtered_data, obstacle_info

    def find_obstacle_clusters(self, filtered_data, angle_threshold=5.0):
        """
        Find clusters of consecutive readings that might represent a single obstacle.
        
        Args:
            filtered_data: List of (angle, distance) pairs
            angle_threshold: Maximum angle gap to consider points as part of the same obstacle
            
        Returns:
            List of clusters, each cluster being a list of (angle, distance) points
        """
        # Only work with valid readings
        valid_points = [(angle, dist) for angle, dist in filtered_data if dist is not None]
        
        if not valid_points:
            return []
            
        # Sort by angle
        valid_points.sort(key=lambda x: x[0])
        
        clusters = []
        current_cluster = [valid_points[0]]
        
        # Cluster points based on angle proximity
        for i in range(1, len(valid_points)):
            prev_angle = valid_points[i-1][0]
            curr_angle = valid_points[i][0]
            
            # If points are close enough in angle, add to current cluster
            if abs(curr_angle - prev_angle) <= angle_threshold:
                current_cluster.append(valid_points[i])
            else:
                # Start a new cluster
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = [valid_points[i]]
        
        # Add the last cluster if it exists
        if current_cluster:
            clusters.append(current_cluster)
            
        return clusters

    def get_largest_obstacle(self, clusters):
        """
        Find the largest obstacle cluster and return its properties.
        
        Args:
            clusters: List of clusters from find_obstacle_clusters()
            
        Returns:
            Dict with obstacle information
        """
        if not clusters:
            return {
                "detected": False,
                "left_angle": None,
                "right_angle": None,
                "center_angle": None,
                "min_distance": None,
                "width_degrees": None
            }
        
        # Find the largest cluster by angular width
        largest_cluster = max(clusters, key=lambda c: abs(c[-1][0] - c[0][0]))
        
        # Find the minimum distance reading in this cluster
        min_dist_point = min(largest_cluster, key=lambda p: p[1])
        
        return {
            "detected": True,
            "left_angle": largest_cluster[0][0],
            "right_angle": largest_cluster[-1][0],
            "center_angle": min_dist_point[0],
            "min_distance": min_dist_point[1],
            "width_degrees": abs(largest_cluster[-1][0] - largest_cluster[0][0])
        }
    
    def lidar_setup(self):
        # Setup serial port
        self.ser = serial.Serial(
            port=LIDAR_PORT,
            baudrate=LIDAR_BAUD,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        
        print("Connected to: " + self.ser.portstr)
        time.sleep(1)
        print("Is port open? ", self.ser.isOpen())
        
        # Send commands to start LIDAR
        self.ser.write(bytes(SENSITIVITY))
        time.sleep(0.1)
        self.ser.write(bytes(RUN_2D))  # Using 2D mode
        
        # Setup angle array for output
        self.angles = np.linspace(-120, 120, num=321)  # Full 240° range

    def get_scan(self):
        
        # Read available data
        if self.ser.in_waiting > 0:
            readdata = self.ser.read(self.ser.in_waiting)
            
            # Parse each byte
            for i in range(len(readdata)):
                if self.parser(readdata[i]):
                    # If parser returns True, we have a complete packet
                    if len(self.receivedData) > 0:
                        # Process the data by combining every two bytes
                        scan_data = []
                        for j in range(0, len(self.receivedData), 2):
                            if j+1 < len(self.receivedData):  # Ensure we have a pair
                                distance = (self.receivedData[j] << 8) | self.receivedData[j+1]
                                scan_data.append(distance)
                        
                        # Process and filter scan data
                        if len(scan_data) > 0:
                            # Only use angles that correspond to our data points
                            scan_angles = self.angles[:len(scan_data)] if len(scan_data) < len(self.angles) else self.angles
                            
                            # First filter the data based on valid distance ranges
                            filtered_data = []
                            for i, distance in enumerate(scan_data):
                                if i < len(scan_angles):
                                    if 200 <= distance < 16000:
                                        filtered_data.append((scan_angles[i], distance))
                                    else:
                                        filtered_data.append((scan_angles[i], None))
                            
                            # Find obstacle clusters
                            clusters = self.find_obstacle_clusters(filtered_data)
                            
                            # Get the largest obstacle (assumed to be the single obstacle)
                            obstacle_info = self.get_largest_obstacle(clusters)
                            
                            # Output the filtered scan with minimal data
                            
                            valid_points = [(a, d) for a, d in filtered_data if d is not None]
                            if valid_points:
                                pass
                                # We could print these points if needed
                                # for angle, distance in valid_points:
                                #     print(f"Angle: {angle:.1f}°, Distance: {distance} mm")
                            else:
                                print("No valid readings detected")
                            
                            # Output obstacle information if detected
                            if obstacle_info["detected"]:
                                print("\n--- Obstacle Detected ---")
                                print(f"Left boundary: {obstacle_info['left_angle']:.1f}°")
                                print(f"Right boundary: {obstacle_info['right_angle']:.1f}°")
                                print(f"Center angle: {obstacle_info['center_angle']:.1f}°")
                                print(f"Minimum distance: {obstacle_info['min_distance']} mm")
                                print(f"Angular width: {obstacle_info['width_degrees']:.1f}°")
                            else:
                                print("\nNo obstacles detected")
                                
                            print("--- End of Scan ---\n")

    def close(self):
        if self.ser and self.ser.isOpen():
            self.ser.write(bytes(COMMAND_STOP))
            self.ser.close()
            print("LiDAR serial port closed")
