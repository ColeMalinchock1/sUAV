
from pymavlink import mavutil
import socket
import math
import time

class PixhawkCommands():

    def __init__(self, serial_port, baud_rate):
        self.pixhawk = mavutil.mavlink_connection(serial_port, baud=baud_rate)
        self.pixhawk.wait_heartbeat()
        
        # Request GPS messages specifically using MESSAGE_INTERVAL
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            1000000,  # 1Hz (interval in microseconds)
            0, 0, 0, 0, 0
        )

    def get_satellite_count(self, timeout=10, max_attempts=3):
        """
        Returns the number of visible GPS satellites with retry mechanism
        
        Parameters:
        timeout (int): Maximum time to wait for a message in seconds
        max_attempts (int): Number of times to retry receiving the message
        
        Returns:
        int: Number of visible satellites, or None if no GPS message received
        """
        for attempt in range(max_attempts):
            # First, flush the receive buffer
            self.pixhawk.recv_match(blocking=False)
            
            # Try to get GPS message
            msg = self.pixhawk.recv_match(
                type='GPS_RAW_INT', 
                blocking=True, 
                timeout=timeout
            )
            
            if msg is not None:
                return msg.satellites_visible
            else:
                print(f"Attempt {attempt + 1}/{max_attempts}: Failed to receive GPS_RAW_INT")
                
                # Request the message again
                self.pixhawk.mav.command_long_send(
                    self.pixhawk.target_system,
                    self.pixhawk.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                    1000000,  # 1Hz
                    0, 0, 0, 0, 0
                )
                
        return None

    def get_gps_status(self):
        """
        Comprehensive GPS status check
        
        Returns:
        dict: GPS status information or None if unavailable
        """
        msg = self.pixhawk.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        sys_status = self.pixhawk.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        
        if msg is None or sys_status is None:
            return None
            
        status = {
            'satellites_visible': msg.satellites_visible,
            'fix_type': msg.fix_type,
            'gps_health': (sys_status.onboard_control_sensors_health & 
                        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS) > 0,
            'hdop': msg.eph / 100.0 if msg.eph != 65535 else None,
            'vdop': msg.epv / 100.0 if msg.epv != 65535 else None
        }
        
        return status

    def get_gps(self, timeout=10, max_attempts=3):
        """
        Comprehensive GPS data retrieval including position, velocity, and status
        
        Parameters:
        timeout (int): Maximum time to wait for messages in seconds
        max_attempts (int): Number of times to retry receiving messages
        
        Returns:
        dict: Complete GPS information including position, velocity, and status
        """
        for attempt in range(max_attempts):
            # Get GPS_RAW_INT for basic GPS info
            gps_raw = self.pixhawk.recv_match(
                type='GPS_RAW_INT',
                blocking=True,
                timeout=timeout
            )
            
            # Get GLOBAL_POSITION_INT for more accurate position and velocity
            global_pos = self.pixhawk.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=timeout
            )
            
            if gps_raw is not None and global_pos is not None:
                gps_data = {
                    # Position data (from GLOBAL_POSITION_INT for better accuracy)
                    'latitude': global_pos.lat / 1e7,  # Convert from int32 to degrees
                    'longitude': global_pos.lon / 1e7,
                    'altitude': global_pos.alt / 1000.0,  # Convert from mm to meters
                    'relative_altitude': global_pos.relative_alt / 1000.0,
                    
                    # Velocity data
                    'vx': global_pos.vx / 100.0,  # Convert to m/s
                    'vy': global_pos.vy / 100.0,
                    'vz': global_pos.vz / 100.0,
                    
                    # GPS status data (from GPS_RAW_INT)
                    'fix_type': gps_raw.fix_type,
                    'satellites_visible': gps_raw.satellites_visible,
                    'hdop': gps_raw.eph / 100.0 if gps_raw.eph != 65535 else None,
                    'vdop': gps_raw.epv / 100.0 if gps_raw.epv != 65535 else None,
                    
                    # Additional GPS info
                    'ground_speed': math.sqrt(
                        (global_pos.vx / 100.0) ** 2 + 
                        (global_pos.vy / 100.0) ** 2
                    ),
                    'heading': global_pos.hdg / 100.0 if global_pos.hdg != 65535 else None,
                }
                
                return gps_data
            else:
                print(f"Attempt {attempt + 1}/{max_attempts}: Failed to receive complete GPS data")
                
                # Request messages again
                self.pixhawk.mav.command_long_send(
                    self.pixhawk.target_system,
                    self.pixhawk.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                    1000000,  # 1Hz
                    0, 0, 0, 0, 0
                )
        
        return None

    def get_fix_type_string(self, fix_type):
        """
        Convert GPS fix type number to human-readable string
        """
        fix_types = {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed",
            7: "Static",
            8: "PPP"
        }
        return fix_types.get(fix_type, "Unknown")