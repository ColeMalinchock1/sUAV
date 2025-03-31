#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Simple LIDAR5B reader
import serial
import numpy as np
import time

# Command definitions
RUN_2D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03]
RUN_3D = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A]
RUN_DUAL = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x07, 0x00, 0x05]
COMMAND_STOP = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x00]
SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x11, 0x64, 0x77]  # sensitivity of 100

# Parser state machine constants
HEADER1, HEADER2, HEADER3, LENGTH_LSB, LENGTH_MSB, PAYLOAD_HEADER, PAYLOAD_DATA, CHECKSUM = 0, 1, 2, 3, 4, 5, 6, 7
NORMAL_MODE = 0x5A
PRODUCT_CODE = 0x77
DEFAULT_ID = 0xFF

# Initialize global variables
buffercounter, CPC, lengthLSB, lengthMSB, data_length = 0, 0, 0, 0, 0
step = HEADER1
receivedData = []

def Parser(data):
    global step, CPC, lengthLSB, lengthMSB, data_length, buffercounter, receivedData
    if step != CHECKSUM:  # CPC is a variable for storing checksum
        CPC = CPC ^ data

    if step == HEADER1 and data == NORMAL_MODE:
        step = HEADER2
        
    elif step == HEADER2 and data == PRODUCT_CODE:
        step = HEADER3
        
    elif step == HEADER3 and data == DEFAULT_ID:
        step = LENGTH_LSB
        CPC = 0
        
    elif step == LENGTH_LSB:
        step = LENGTH_MSB
        lengthLSB = data
        
    elif step == LENGTH_MSB:
        step = PAYLOAD_HEADER
        lengthMSB = data
        data_length = ((lengthMSB << 8) & 0xff00) | (lengthLSB & 0x00ff)
        
    elif step == PAYLOAD_HEADER:
        step = PAYLOAD_DATA
        if data_length == 1:
            step = CHECKSUM
        buffercounter = 0
        receivedData = []
        
    elif step == PAYLOAD_DATA:
        receivedData.append(data)
        buffercounter = buffercounter+1
        if buffercounter >= data_length - 1:
            step = CHECKSUM
            
    elif step == CHECKSUM:
        step = HEADER1
        
        if CPC == data:
            return True
    else:
        step = HEADER1
        return False

def filter_and_analyze_lidar_data(scan_data, angles):
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
    # Define thresholds for filtering
    min_valid_distance = 200  # Minimum valid distance (mm)
    max_valid_distance = 16000  # Maximum valid distance (mm)
    
    filtered_data = []
    valid_readings = []  # Store valid readings for obstacle detection
    
    # First pass: filter data
    for i, distance in enumerate(scan_data):
        if i < len(angles):
            if min_valid_distance <= distance < max_valid_distance:
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

def find_obstacle_clusters(filtered_data, angle_threshold=5.0):
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

def get_largest_obstacle(clusters):
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

def main():
    try:
        # Setup serial port
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        
        print("Connected to: " + ser.portstr)
        time.sleep(1)
        print("Is port open? ", ser.isOpen())
        
        # Send commands to start LIDAR
        ser.write(bytes(SENSITIVITY))
        time.sleep(0.1)
        ser.write(bytes(RUN_2D))  # Using 2D mode
        
        # Setup angle array for output
        angles = np.linspace(-120, 120, num=321)  # Full 240° range
        
        print("Starting LIDAR scan...")
        print("Press Ctrl+C to stop")
        
        # Main loop
        while True:
            # Read available data
            if ser.in_waiting > 0:
                readdata = ser.read(ser.in_waiting)
                
                # Parse each byte
                for i in range(len(readdata)):
                    if Parser(readdata[i]):
                        # If parser returns True, we have a complete packet
                        if len(receivedData) > 0:
                            # Process the data by combining every two bytes
                            scan_data = []
                            for j in range(0, len(receivedData), 2):
                                if j+1 < len(receivedData):  # Ensure we have a pair
                                    distance = (receivedData[j] << 8) | receivedData[j+1]
                                    scan_data.append(distance)
                            
                            # Process and filter scan data
                            if len(scan_data) > 0:
                                # Only use angles that correspond to our data points
                                scan_angles = angles[:len(scan_data)] if len(scan_data) < len(angles) else angles
                                
                                # First filter the data based on valid distance ranges
                                filtered_data = []
                                for i, distance in enumerate(scan_data):
                                    if i < len(scan_angles):
                                        if 200 <= distance < 16000:
                                            filtered_data.append((scan_angles[i], distance))
                                        else:
                                            filtered_data.append((scan_angles[i], None))
                                
                                # Find obstacle clusters
                                clusters = find_obstacle_clusters(filtered_data)
                                
                                # Get the largest obstacle (assumed to be the single obstacle)
                                obstacle_info = get_largest_obstacle(clusters)
                                
                                # Output the filtered scan with minimal data
                                print("\n--- New LIDAR Scan Summary ---")
                                valid_points = [(a, d) for a, d in filtered_data if d is not None]
                                if valid_points:
                                    print(f"Valid readings: {len(valid_points)}")
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
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nStopping LIDAR...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.isOpen():
            ser.write(bytes(COMMAND_STOP))
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()