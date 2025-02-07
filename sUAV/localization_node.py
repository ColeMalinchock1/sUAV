#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pixhawk_commands import PixhawkCommands
import curses
import time
import numpy as np
import os
import csv
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        
        # Initialize data storage
        self.zed_data = {
            'x': None,
            'y': None,
            'z': None,
            'yaw': None
        }
        
        # Initialize curses
        self.stdscr = curses.initscr()
        curses.start_color()
        curses.use_default_colors()
        
        # Initialize CSV logging
        self.filename = f"sensor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_csv()
        
        # Initialize Pixhawk connection
        try:
            self.pixhawk = PixhawkCommands(serial_port='/dev/ttyTHS1', baud_rate=57600)
            self.get_logger().info("Pixhawk connection established")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pixhawk: {e}")
            raise
        
        # Create ZED pose subscription
        self.create_subscription(
            PoseStamped,
            "/zed/zed_node/pose",
            self.zed_pose_callback,
            10
        )
        
        # Create timer for data logging
        self.create_timer(0.1, self.log_data)  # 10Hz logging rate
        
    def init_csv(self):
        """Initialize CSV file with headers"""
        headers = [
            'timestamp',
            'gps_latitude',
            'gps_longitude',
            'gps_altitude',
            'gps_heading',
            'gps_satellites',
            'gps_fix_type',
            'gps_hdop',
            'gps_vdop',
            'gps_ground_speed',
            'zed_x',
            'zed_y',
            'zed_z',
            'zed_yaw'
        ]
        
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            
    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to euler angles"""
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

    def normalize_yaw(self, yaw):
        """Normalize yaw angle"""
        if yaw > 0:
            return 180 - yaw if yaw > 90 else yaw
        else:
            return -(180 + yaw) if yaw < -90 else yaw

    def zed_pose_callback(self, msg):
        """Handle ZED pose messages"""
        try:
            # Extract position
            self.zed_data['x'] = -msg.pose.position.x
            self.zed_data['y'] = msg.pose.position.y
            self.zed_data['z'] = msg.pose.position.z
            
            # Extract orientation
            orientation = msg.pose.orientation
            _, _, yaw = self.quaternion_to_euler(
                orientation.w, 
                orientation.x, 
                orientation.y, 
                orientation.z
            )
            self.zed_data['yaw'] = self.normalize_yaw(yaw)
            
        except Exception as e:
            self.get_logger().error(f"Error processing ZED data: {e}")

    def log_data(self):
        """Log data from both GPS and ZED camera"""
        try:
            # Get GPS data
            gps_data = self.pixhawk.get_gps()
            
            if gps_data and all(v is not None for v in self.zed_data.values()):
                # Prepare data row
                data_row = [
                    time.time(),
                    gps_data['latitude'],
                    gps_data['longitude'],
                    gps_data['altitude'],
                    gps_data['heading'],
                    gps_data['satellites_visible'],
                    gps_data['fix_type'],
                    gps_data['hdop'],
                    gps_data['vdop'],
                    gps_data['ground_speed'],
                    self.zed_data['x'],
                    self.zed_data['y'],
                    self.zed_data['z'],
                    self.zed_data['yaw']
                ]
                
                # Write to CSV
                with open(self.filename, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(data_row)
                
                # Update display
                self.update_display(gps_data)
                
        except Exception as e:
            self.get_logger().error(f"Error logging data: {e}")

    def update_display(self, gps_data):
        """Update the curses display with current data"""
        try:
            self.stdscr.clear()
            self.stdscr.addstr(1, 5, 'DATA LOGGER NODE')
            self.stdscr.addstr(3, 5, f'Satellites: {gps_data["satellites_visible"]}')
            self.stdscr.addstr(4, 5, f'GPS Lat: {gps_data["latitude"]:.6f}')
            self.stdscr.addstr(5, 5, f'GPS Lon: {gps_data["longitude"]:.6f}')
            self.stdscr.addstr(6, 5, f'ZED X: {self.zed_data["x"]:.2f}')
            self.stdscr.addstr(7, 5, f'ZED Y: {self.zed_data["y"]:.2f}')
            self.stdscr.addstr(8, 5, f'ZED Yaw: {self.zed_data["yaw"]:.2f}')
            self.stdscr.addstr(9, 5, f'Logging to: {self.filename}')
            self.stdscr.refresh()
            
        except Exception as e:
            self.get_logger().error(f"Error updating display: {e}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = DataLogger()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally: 
        # Clean up
        curses.endwin()
        rclpy.shutdown()

if __name__ == '__main__':
    main()