#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import threading
import time
import math
import numpy as np

MAX_THRESHOLD = 2.0 #meters
DRONE_WIDTH = 1.0 #meters
RADIUS = 0.2 #meters
current_position = None
detecting_obstacles = False
current_yaw = waypoint_idx = 0

waypoints = [(3, 0)]

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

def waypoints_callback(msg):
    global waypoints

    waypoints = msg.data

def zed_pose_callback(msg):
    """Handle ZED pose messages."""
    global current_position, current_yaw

    # Extract position
    current_position = (
        -msg.pose.position.x,
        msg.pose.position.y
    )
    
    # Extract yaw
    orientation_q = msg.pose.orientation
    _, _, yaw = quaternion_to_euler_angle_vectorized2(orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z)
    current_yaw = normalize_yaw(yaw)
    
def detect_obstacles(center_row1_points, center_row2_points):

    # Average the points from both rows
    depth_angle_array = []
    min_length = min(len(center_row1_points), len(center_row2_points))
    
    # Initializes the checks
    scanning_obstacle = False
    obstacles = []
    previous_point = None

    # Scanning through all the points int the rows
    for i in range(min_length):

        # Getting the average depth and angle of the middle top and bottom rows of the scan
        avg_depth = (center_row1_points[i][0] + center_row2_points[i][0]) / 2
        avg_angle = (center_row1_points[i][1] + center_row2_points[i][1]) / 2

        # Checks if the average depth is 0 and sets it to a max 
        if avg_depth == 0:
            avg_depth = 100.0
        
        # Checks if the depth is less than the threshold which indicates an obstacle
        if avg_depth < MAX_THRESHOLD:
            # print(f"Depth: {avg_depth}, Angle: {avg_angle}")

            # Checks if it is a new obstacle detected
            if not scanning_obstacle:

                # If it is a new obstacle, the left end of it is recorded as an obstacle
                # And the scanning obstacle is set to true
                obstacles.append([avg_depth, avg_angle])
                scanning_obstacle = True

        # Else if it does not exceed the threshold and it is still currently
        # scanning an obstacle or it reaches the end of the scan and it is still scanning an obstacle
        elif scanning_obstacle or (scanning_obstacle and i == min_length - 1):

            # Mark it as the end of the obstacle and set scanning obstacle to false
            obstacles.append(previous_point)
            scanning_obstacle = False
        
        # Add the average depth and average angle to the array
        depth_angle_array.append((avg_depth, avg_angle))
        previous_point = [avg_depth, avg_angle]
    
    return obstacles

def lidar_3d_callback(msg):
        """Handle incoming lidar messages."""
        global center_row1_points, center_row2_points, detecting_obstacles

        # Initialize arrays for the two center rows
        center_row1_points = []
        center_row2_points = []
        
        # Extract the height and width
        height = msg.height
        width = msg.width
        
        # Define center rows
        center_row1 = height // 2
        center_row2 = center_row1 + 1 if height > 1 else center_row1
        

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

        detecting_obstacles = True

def normalize_yaw(input_yaw):
    if input_yaw > 0:
        if input_yaw > 90:
            return (180 - input_yaw)
        else:
            return input_yaw
    else:
        if input_yaw < -90:
            return -(180 + input_yaw)
        else:
            return input_yaw
        

def main():

    global waypoint_idx

    rclpy.init()
    node = rclpy.create_node('obstacle_avoidance_node')
    node.create_subscription(PointCloud2, "/scan_3D", lidar_3d_callback, 10)
    node.create_subscription(PoseStamped, "/zed/zed_node/pose", zed_pose_callback, 10)

    yaw_pub = node.create_publisher(Float64, "/obstacle_avoidance/yaw", 10)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    try:
        while current_position is None:
            pass

    
        while rclpy.ok():
            
            if math.sqrt((current_position[0] - waypoints[waypoint_idx][0])**2 + (current_position[1] - waypoints[waypoint_idx][1])**2) < RADIUS:
                waypoint_idx += 1
                if waypoint_idx == len(waypoints):
                    print("Mission accomplished!")
                    break
                
            # Gets the direction to the next waypoint
            current_waypoint = waypoints[waypoint_idx]
            
            waypoint_vector = [current_waypoint[0] - current_position[0], current_waypoint[1] - current_position[1]]

            yaw_delta = math.degrees(math.atan(waypoint_vector[1]/waypoint_vector[0])) - current_yaw
            
            if (detecting_obstacles):
                obstacles = detect_obstacles(center_row1_points, center_row2_points)
                if len(obstacles) > 0:
                    
                    # Loop through all the obstacles
                    for i in range(0, len(obstacles), 2):

                        # Get the values of the left and right of the obstacle
                        left = obstacles[i]
                        right = obstacles[i + 1]

                        if abs(left[1] - current_yaw) > abs(right[1] - current_yaw):
                            # Go to the right of the obstacle
                            print(right[0])
                            print("GO RIGHT")
                            yaw_delta = math.degrees(math.atan(DRONE_WIDTH / (2 * right[0])))
                        else:
                            # Go to the left of the obstacle
                            print(left[0])
                            print("GO LEFT")
                            yaw_delta = -math.degrees(math.atan(DRONE_WIDTH / (2 * left[0])))
                
                print("Number of obstacles: ", len(obstacles)/2)
                print("YAW DELTA: ", yaw_delta)
            msg = Float64()
            msg.data = yaw_delta
            yaw_pub.publish(msg)
            

            rate.sleep()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    