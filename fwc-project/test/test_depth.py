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

class ZedImaging(Node):
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
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        self.count = 0
        self.depth_image = None
        self.rgb_image = None

        self.create_timer(1.0, self.zed_save_image)

    def zed_image_callback(self, msg):
        """Store the latest RGB image"""
        
        # Your original image processing code
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.flip(cv_image, -1)

            cv2.imshow('Zed Camera RGB', cv_image)
            
            self.rgb_image = cv_image
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

    def zed_depth_callback(self, msg):
        """Store the latest depth image"""

        # Your original depth processing code
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            # cv_image = cv2.resize(cv_image, (640, 360))
            cv_image = cv2.flip(cv_image, -1)
            
            cv2.imshow('Zed Depth', cv_image)
            self.depth_image = cv_image
            
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def zed_save_image(self):
        if self.depth_image is not None and self.rgb_image is not None:
            self.count += 1
            np.save(f'results/depth/depth_image_{self.count}.npy', self.depth_image)
            cv2.imwrite(f'results/rgb/rgb_image_{self.count}.png', self.rgb_image)

def main(args=None):
    rclpy.init(args=args)
    
    zed_imaging = ZedImaging()
    
    try:
        rclpy.spin(zed_imaging)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        zed_imaging.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()