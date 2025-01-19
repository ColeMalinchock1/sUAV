#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque

class ZedLineOverlay(Node):
    def __init__(self):
        super().__init__('zed_line_overlay')
        
        # Parameters for depth smoothing
        self.declare_parameter('buffer_size', 5)
        self.declare_parameter('spatial_kernel', 5)
        
        # Get parameters
        self.buffer_size = self.get_parameter('buffer_size').value
        self.spatial_kernel = self.get_parameter('spatial_kernel').value
        
        # Initialize depth frame buffer for temporal smoothing
        self.depth_buffer = deque(maxlen=self.buffer_size)
        
        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
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
        self.yaw_sub = self.create_subscription(
            Float64,
            '/obstacle_avoidance/yaw',
            self.yaw_callback,
            10
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize angle for the rotating line
        self.current_angle = 0
    
    def yaw_callback(self, msg):
        self.current_angle = msg.data

    def smooth_depth_image(self, depth_image):
        """Apply temporal and spatial smoothing to depth image"""
        # Add to buffer
        self.depth_buffer.append(depth_image)
        
        if len(self.depth_buffer) == self.buffer_size:
            # Temporal smoothing (moving average)
            smoothed = np.mean(self.depth_buffer, axis=0)
            
            # Spatial smoothing with Gaussian blur
            smoothed = cv2.GaussianBlur(
                smoothed,
                (self.spatial_kernel, self.spatial_kernel),
                0
            )
            
            # Additional bilateral filter to preserve edges
            smoothed = cv2.bilateralFilter(
                smoothed.astype(np.float32),
                d=5,  # Diameter of pixel neighborhood
                sigmaColor=50,  # Filter sigma in color space
                sigmaSpace=50  # Filter sigma in coordinate space
            )
            
            return smoothed
        
        return depth_image

    def draw_overlay_lines(self, cv_image):
        """Draw the overlay lines on the image"""
        height, width = cv_image.shape[:2]
        line_length = height // 2
        bottom_center = (width // 2, height)
        
        # Calculate end point of line based on current angle
        end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
        end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
        
        # Draw reference line
        cv2.line(cv_image, 
                bottom_center, 
                (bottom_center[0], bottom_center[1] - line_length),
                (0, 0, 255),  # Red color
                2)
        
        # Draw angle line
        cv2.line(cv_image, 
                bottom_center, 
                (end_x, end_y),
                (0, 255, 0),  # Green color
                2)
        
        return cv_image

    def zed_depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            
            # Flip image
            cv_image = cv2.flip(cv_image, -1)
            
            # Apply smoothing
            smoothed_depth = self.smooth_depth_image(cv_image)
            
            # Draw lines on smoothed depth image
            final_image = self.draw_overlay_lines(smoothed_depth)
            
            # Display the image
            cv2.imshow('Zed Depth with Line Overlay', final_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def lidar_depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            final_image = self.draw_overlay_lines(cv_image)
            cv2.imshow('Lidar Depth with Line Overlay', final_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing lidar image: {str(e)}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.flip(cv_image, -1)
            final_image = self.draw_overlay_lines(cv_image)
            cv2.imshow('Zed Camera with Line Overlay', final_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    zed_overlay = ZedLineOverlay()
    
    try:
        rclpy.spin(zed_overlay)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        zed_overlay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()