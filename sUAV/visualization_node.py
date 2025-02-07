#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ZedLineOverlay(Node):
    def __init__(self):
        super().__init__('zed_line_overlay')
        
        # Create subscription to the Zed camera image topic
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',  # Adjust topic name if needed
            self.image_callback,
            10)
        self.yaw_sub = self.create_subscription(
            Float64,
            '/obstacle_avoidance/yaw',
            self.yaw_callback,
            10
        )
        self.zed_depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.zed_depth_callback,
            10)
        self.lidar_depth_sub = self.create_subscription(
            Image,
            '/scan_image',
            self.lidar_depth_callback,
            10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize angle for the rotating line
        self.current_angle = 0
        self.target_angle = 0
        self.step_size = 2 # degrees
    
    def yaw_callback(self, msg):
        self.target_angle = msg.data

    def lidar_depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            cv_image = cv2.resize(cv_image, (640, 240))
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            
            # Calculate line endpoints
            line_length = height // 2  # Line length is half the image height
            bottom_center = (width // 2, height)  # Bottom center point

            angle_delta = abs(self.current_angle - self.target_angle)

            if angle_delta < self.step_size:
                self.current_angle = self.target_angle
            else:
                if self.current_angle - self.target_angle < 0:
                    self.current_angle += self.step_size
                else:
                    self.current_angle -= self.step_size
            
            # Calculate end point of line based on current angle
            end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
            end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
            
            cv2.line(cv_image, 
                    bottom_center, 
                    (bottom_center[0], bottom_center[1] - line_length),
                    (0, 0, 255),  # Green color
                    2)  # Line thickness

            # Draw the line
            cv2.line(cv_image, 
                    bottom_center, 
                    (end_x, end_y),
                    (0, 255, 0),  # Green color
                    2)  # Line thickness
            
            # Display the image
            cv2.imshow('Lidar Depth with Line Overlay', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing lidar image: {str(e)}')

    def zed_depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            cv_image = cv2.resize(cv_image, (640, 360))

            cv_image = cv2.flip(cv_image, -1)
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Calculate line endpoints
            line_length = height // 2  # Line length is half the image height
            bottom_center = (width // 2, height)  # Bottom center point

            angle_delta = abs(self.current_angle - self.target_angle)

            if angle_delta < self.step_size:
                self.current_angle = self.target_angle
            else:
                if self.current_angle - self.target_angle < 0:
                    self.current_angle += self.step_size
                else:
                    self.current_angle -= self.step_size

            # Calculate end point of line based on current angle
            end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
            end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
            
            cv2.line(cv_image, 
                    bottom_center, 
                    (bottom_center[0], bottom_center[1] - line_length),
                    (0, 0, 255),  # Green color
                    2)  # Line thickness

            # Draw the line
            cv2.line(cv_image, 
                    bottom_center, 
                    (end_x, end_y),
                    (0, 255, 0),  # Green color
                    2)  # Line thickness
            
            # Display the image
            cv2.imshow('Zed Depth with Line Overlay', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            

            cv_image = cv2.flip(cv_image, -1)
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            print(height)
            print(width)
            
            # Calculate line endpoints
            line_length = height // 2  # Line length is half the image height
            bottom_center = (width // 2, height)  # Bottom center point

            angle_delta = abs(self.current_angle - self.target_angle)

            if angle_delta < self.step_size:
                self.current_angle = self.target_angle
            else:
                if self.current_angle - self.target_angle < 0:
                    self.current_angle += self.step_size
                else:
                    self.current_angle -= self.step_size

            print(self.current_angle)
            print(self.target_angle)

            # Calculate end point of line based on current angle
            end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
            end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
            
            cv2.line(cv_image, 
                    bottom_center, 
                    (bottom_center[0], bottom_center[1] - line_length),
                    (0, 0, 255),  # Green color
                    2)  # Line thickness

            # Draw the line
            cv2.line(cv_image, 
                    bottom_center, 
                    (end_x, end_y),
                    (0, 255, 0),  # Green color
                    2)  # Line thickness
            
            # Display the image
            cv2.imshow('Zed Camera with Line Overlay', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    zed_overlay = ZedLineOverlay()
    
    try:
        rclpy.spin(zed_overlay)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        zed_overlay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()