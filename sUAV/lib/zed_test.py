#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ZedLineOverlay(Node):
    def __init__(self):
        super().__init__('zed_line_overlay')
        
        # Create subscription to the Zed camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',  # Adjust topic name if needed
            self.image_callback,
            10)
        self.subscription
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize angle for the rotating line
        self.current_angle = 0
        self.angle_increment = 1  # Degrees to increment per frame
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Calculate line endpoints
            line_length = height // 2  # Line length is half the image height
            bottom_center = (width // 2, height)  # Bottom center point
            
            # Calculate end point of line based on current angle
            end_x = bottom_center[0] + int(line_length * math.sin(math.radians(self.current_angle)))
            end_y = bottom_center[1] - int(line_length * math.cos(math.radians(self.current_angle)))
            
            # Draw the line
            cv2.line(cv_image, 
                    bottom_center, 
                    (end_x, end_y),
                    (0, 255, 0),  # Green color
                    2)  # Line thickness
            
            # Update angle for next frame
            self.current_angle = (self.current_angle + self.angle_increment) % 360
            
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