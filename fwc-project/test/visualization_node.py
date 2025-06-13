#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
import json
from datetime import datetime

class ZedLineOverlay(Node):
    def __init__(self):
        super().__init__('zed_line_overlay')
        
        # Create subscription to the Zed camera image topic
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.zed_image_callback,
            10)
        self.zed_depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.zed_depth_callback,
            10)

        self.zed_confidence_sub = self.create_subscription(
            Image,
            '/zed/zed_node/confidence/confidence_map',
            self.zed_confidence_callback,
            10
        )

        self.zed_camera_info = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/depth/camera_info',
            self.zed_camera_info_callback,
            10
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Data storage variables
        self.latest_depth = None
        self.latest_confidence = None
        self.latest_camera_info = None
        self.latest_rgb_image = None
        
        # Create data directory
        self.data_dir = f"zed_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(f"{self.data_dir}/depth", exist_ok=True)
        os.makedirs(f"{self.data_dir}/confidence", exist_ok=True)
        os.makedirs(f"{self.data_dir}/rgb", exist_ok=True)
        
        # Timer for saving data every 1 second
        self.save_timer = self.create_timer(1.0, self.save_data_callback)
        self.frame_counter = 0
        
        # Line drawing variables (from your original code)
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.step_size = 1.0

    def zed_image_callback(self, msg):
        """Store the latest RGB image"""
        self.latest_rgb_image = msg
        
        # Your original image processing code
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.flip(cv_image, -1)
            
            height, width = cv_image.shape[:2]
            line_length = height // 2
            bottom_center = (width // 2, height)

            angle_delta = abs(self.current_angle - self.target_angle)
            if angle_delta < self.step_size:
                self.current_angle = self.target_angle
            else:
                if self.current_angle - self.target_angle < 0:
                    self.current_angle += self.step_size
                else:
                    self.current_angle -= self.step_size

            end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
            end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
            
            cv2.line(cv_image, bottom_center, 
                    (bottom_center[0], bottom_center[1] - line_length),
                    (0, 0, 255), 2)
            cv2.line(cv_image, bottom_center, (end_x, end_y), (0, 255, 0), 2)
            
            cv2.imshow('Zed Camera with Line Overlay', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

    def zed_depth_callback(self, msg):
        """Store the latest depth image"""
        self.latest_depth = msg
        
        # Your original depth processing code
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            cv_image = cv2.resize(cv_image, (640, 360))
            cv_image = cv2.flip(cv_image, -1)
            
            height, width = cv_image.shape[:2]
            line_length = height // 2
            bottom_center = (width // 2, height)

            angle_delta = abs(self.current_angle - self.target_angle)
            if angle_delta < self.step_size:
                self.current_angle = self.target_angle
            else:
                if self.current_angle - self.target_angle < 0:
                    self.current_angle += self.step_size
                else:
                    self.current_angle -= self.step_size

            end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
            end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
            
            cv2.line(cv_image, bottom_center,
                    (bottom_center[0], bottom_center[1] - line_length),
                    (0, 0, 255), 2)
            cv2.line(cv_image, bottom_center, (end_x, end_y), (0, 255, 0), 2)
            
            cv2.imshow('Zed Depth with Line Overlay', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def zed_confidence_callback(self, msg):
        """Store the latest confidence map"""
        self.latest_confidence = msg

    def zed_camera_info_callback(self, msg):
        """Store the latest camera info"""
        self.latest_camera_info = msg

    def save_data_callback(self):
        """Save all latest data every 1 second"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # Include milliseconds
        
        try:
            # Save RGB image
            if self.latest_rgb_image is not None:
                rgb_cv = self.bridge.imgmsg_to_cv2(self.latest_rgb_image, "bgr8")
                cv2.imwrite(f"{self.data_dir}/rgb/rgb_{timestamp}.png", rgb_cv)
            
            # Save depth image
            if self.latest_depth is not None:
                depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth)
                # Save as 16-bit PNG to preserve depth values
                if depth_cv.dtype != np.uint16:
                    # Convert to uint16 if needed (assuming depth is in mm)
                    depth_cv = depth_cv.astype(np.uint16)
                cv2.imwrite(f"{self.data_dir}/depth/depth_{timestamp}.png", depth_cv)
            
            # Save confidence map
            if self.latest_confidence is not None:
                confidence_cv = self.bridge.imgmsg_to_cv2(self.latest_confidence, "32FC1")
                # Save as numpy array since it's float32
                np.save(f"{self.data_dir}/confidence/confidence_{timestamp}.npy", confidence_cv)
            
            # Save camera info (only save once or when it changes)
            if self.latest_camera_info is not None:
                camera_info_dict = {
                    'header': {
                        'stamp': {
                            'sec': self.latest_camera_info.header.stamp.sec,
                            'nanosec': self.latest_camera_info.header.stamp.nanosec
                        },
                        'frame_id': self.latest_camera_info.header.frame_id
                    },
                    'height': self.latest_camera_info.height,
                    'width': self.latest_camera_info.width,
                    'distortion_model': self.latest_camera_info.distortion_model,
                    'D': list(self.latest_camera_info.d),
                    'K': list(self.latest_camera_info.k),
                    'R': list(self.latest_camera_info.r),
                    'P': list(self.latest_camera_info.p)
                }
                
                with open(f"{self.data_dir}/camera_info_{timestamp}.json", 'w') as f:
                    json.dump(camera_info_dict, f, indent=2)
            
            self.frame_counter += 1
            self.get_logger().info(f'Saved frame set {self.frame_counter} at {timestamp}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving data: {str(e)}')

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