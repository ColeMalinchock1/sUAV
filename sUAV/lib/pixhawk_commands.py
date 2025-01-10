from pymavlink import mavutil
import socket
from lib.coords_to_cartesian import CoordsToCartesian as c2c
import math
import time

class PixhawkCommands:
    """
    A class to handle communication and commands with a Pixhawk flight controller.
    
    This class provides methods for:
    - Getting GPS and position information
    - Managing waypoints and missions
    - Controlling vehicle movement
    - Monitoring vehicle state
    
    Attributes:
        pixhawk: MAVLink connection object
        converter: Coordinate conversion utility object
    """

    def __init__(self, serial_port, baud_rate):
        """
        Initialize connection to Pixhawk and set up data streams.
        
        Args:
            serial_port (str): Serial port for Pixhawk connection
            baud_rate (int): Baud rate for serial communication
        """
        # Establish connection to Pixhawk
        self.pixhawk = mavutil.mavlink_connection(serial_port, baud=baud_rate)
        self.pixhawk.wait_heartbeat()

        # Request position data stream
        # Note: MAV_DATA_STREAM is deprecated, but still widely used
        # See: https://mavlink.io/en/messages/common.html#MESSAGE_INTERVAL
        self._request_data_streams()

        # Initialize coordinate converter with current position
        current_latlon, _ = self.get_current_latlon()
        while current_latlon is None:
            current_latlon, _ = self.get_current_latlon()
        self.converter = c2c(current_latlon[0], current_latlon[1])

    def _request_data_streams(self):
        """Request necessary data streams from Pixhawk."""
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS
        ]
        
        for stream in streams:
            self.pixhawk.mav.request_data_stream_send(
                self.pixhawk.target_system,
                self.pixhawk.target_component,
                stream,
                1,  # Rate in Hz
                1   # Start/Stop (1=start)
            )

    def get_gps_info(self, timeout=10):
        """
        Get current GPS information including satellite count and fix quality.
        
        Args:
            timeout (int): Maximum time to wait for GPS data in seconds
            
        Returns:
            tuple: (dict of GPS info, error string or None)
                GPS info includes:
                - satellites_visible: Number of visible satellites
                - fix_type: GPS fix type (0-1: no fix, 2: 2D fix, 3: 3D fix)
                - hdop: Horizontal dilution of precision
                - vdop: Vertical dilution of precision
        """
        msg = self.pixhawk.recv_match(type='GPS_RAW_INT', blocking=True, timeout=timeout)
        if msg is None:
            return None, "Failed to receive GPS_RAW_INT"
        
        gps_info = {
            'satellites_visible': msg.satellites_visible,
            'fix_type': msg.fix_type,
            'hdop': msg.eph / 100.0,  # Convert from cm to meters
            'vdop': msg.epv / 100.0   # Convert from cm to meters
        }
        
        return gps_info, None

    def get_relative_position(self, timeout=10):
        """
        Get position relative to home location in NED (North-East-Down) frame.
        
        Args:
            timeout (int): Maximum time to wait for position data
            
        Returns:
            tuple: (dict of position info, error string or None)
                Position info includes:
                - x: North position in meters
                - y: East position in meters
                - z: Up position in meters (negative of down)
                - vx, vy, vz: Velocities in respective directions
        """
        msg = self.pixhawk.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
        if msg is None:
            return None, "Failed to receive LOCAL_POSITION_NED"

        position = {
            'x': msg.x,
            'y': msg.y,
            'z': -msg.z,  # Convert Down to Up for intuitive usage
            'vx': msg.vx,
            'vy': msg.vy,
            'vz': -msg.vz  # Convert Down to Up velocity
        }

        return position, None
    
    def set_guided_mode(self):
        """Switch vehicle to GUIDED mode for manual control."""
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_GUIDED_ARMED,  # GUIDED mode
            0, 0, 0, 0, 0, 0  # parameters (unused)
        )

    def set_auto_mode(self):
        """Switch vehicle back to AUTO mode for mission execution."""
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_AUTO_ARMED,  # AUTO mode
            0, 0, 0, 0, 0, 0  # parameters (unused)
        )

    def get_mode(self, timeout=3):
        """
        Get the current flight mode of the vehicle.
        
        Args:
            timeout (int): Maximum time to wait for mode data in seconds
            
        Returns:
            tuple: (str mode_name, error string or None)
                mode_name will be one of:
                - 'STABILIZE'
                - 'GUIDED'
                - 'AUTO'
                - 'RTL'
                - 'LAND'
                - etc.
        """
        msg = self.pixhawk.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg is None:
            return None, "Failed to receive HEARTBEAT message"
        
        # Get the custom mode from heartbeat
        custom_mode = msg.custom_mode
        
        # Map of mode numbers to names for ArduPilot/PX4
        mode_mapping = {
            0: 'STABILIZE',
            4: 'GUIDED',
            3: 'AUTO',
            6: 'RTL',
            9: 'LAND',
            1: 'ACRO',
            2: 'ALT_HOLD',
            5: 'LOITER',
            7: 'CIRCLE',
            11: 'DRIFT',
            13: 'SPORT',
            14: 'FLIP',
            15: 'AUTOTUNE',
            16: 'POSHOLD',
            17: 'BRAKE',
            18: 'THROW',
            19: 'AVOID_ADSB',
            20: 'GUIDED_NOGPS',
            21: 'SMART_RTL',
        }
        
        mode_name = mode_mapping.get(custom_mode, f'UNKNOWN MODE ({custom_mode})')
        return mode_name, None
    

    def store_current_mode(self):
        """
        Store the current flight mode for later restoration.
        
        Returns:
            tuple: (bool success, error string or None)
        """
        mode, error = self.get_mode()
        if mode is None:
            return False, error
            
        self.stored_mode = mode
        return True, None

    def restore_mode(self):
        """
        Restore the previously stored flight mode.
        
        Returns:
            tuple: (bool success, error string or None)
        """
        if self.stored_mode is None:
            return False, "No stored mode available"
            
        # Map of mode names to their MAVLink mode values
        mode_mapping = {
            'STABILIZE': 0,
            'GUIDED': 4,
            'AUTO': 3,
            'RTL': 6,
            'LAND': 9,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'LOITER': 5,
            'CIRCLE': 7,
            'DRIFT': 11,
            'SPORT': 13,
            'FLIP': 14,
            'AUTOTUNE': 15,
            'POSHOLD': 16,
            'BRAKE': 17,
            'THROW': 18,
            'AVOID_ADSB': 19,
            'GUIDED_NOGPS': 20,
            'SMART_RTL': 21
        }
        
        if self.stored_mode not in mode_mapping:
            return False, f"Unknown mode: {self.stored_mode}"
            
        # Set the mode
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[self.stored_mode],
            0, 0, 0, 0, 0  # parameters (unused)
        )
        
        return True, None

    def move_to_relative_position(self, x_offset, y_offset, z_offset, velocity=1):
        """
        Move vehicle relative to its current body frame orientation.
        
        Args:
            x_offset (float): Forward(+)/backward(-) distance in meters
            y_offset (float): Right(+)/left(-) distance in meters
            z_offset (float): Up(+)/down(-) distance in meters
            velocity (float): Desired velocity in m/s
            
        Returns:
            tuple: (bool success, error string or None)
        """
        # Send movement command
        # Bitmask: enable position control only
        type_mask = 0b110111111000  
        self.pixhawk.mav.set_position_target_local_ned_send(
            0,  # timestamp (0 for immediate)
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Changed to body frame
            type_mask,
            x_offset,  # Forward/back
            -y_offset,  # Left/right (negative because NED frame)
            -z_offset,  # Up/down (negative because NED frame)
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0      # yaw, yaw_rate
        )
    
        return True, None

    def get_current_xy(self, timeout=10):
        """
        Get current position in local XY coordinates.
        
        Args:
            timeout (int): Maximum time to wait for position data
            
        Returns:
            tuple: (dict of position info, error string or None)
                Position info includes x, y, z coordinates and yaw
        """
        msg = self.pixhawk.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            return None, "Failed to receive GLOBAL_POSITION_INT"
        
        current_position = self.converter.latlon_to_xy(msg.lat, msg.lon)
        location = {
            'x': current_position[0],
            'y': current_position[1],
            'z': msg.alt,
            'yaw': self.converter.compass_heading_to_yaw(msg.hdg)
        }

        return location, None

    def get_current_latlon(self, timeout=10):
        """
        Get current position in latitude/longitude coordinates.
        
        Args:
            timeout (int): Maximum time to wait for position data
            
        Returns:
            tuple: (list of position info, error string or None)
                Position list contains [latitude, longitude, altitude, heading]
        """
        msg = self.pixhawk.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            return None, "Failed to receive GLOBAL_POSITION_INT"

        position = [
            msg.lat / 1e7,  # Convert to degrees
            msg.lon / 1e7,  # Convert to degrees
            msg.alt / 1000.0,  # Convert mm to meters
            msg.hdg / 100.0 if msg.hdg is not None else None  # Convert centidegrees to degrees
        ]

        return position, None

    def get_waypoints(self, timeout=10):
        """
        Retrieve all waypoints from current mission.
        
        Args:
            timeout (int): Maximum time to wait for waypoint data
            
        Returns:
            tuple: (list of waypoints, error string or None)
        """
        self.pixhawk.waypoint_request_list_send()
        msg = self.pixhawk.recv_match(type='MISSION_COUNT', blocking=True, timeout=timeout)
        
        if msg is None:
            return None, "Failed to receive MISSION_COUNT"

        waypoints = []
        for i in range(msg.count):
            self.pixhawk.waypoint_request_send(i)
            msg = self.pixhawk.recv_match(type='MISSION_ITEM', blocking=True, timeout=timeout)
            
            if msg is None:
                return None, f"Failed to receive MISSION_ITEM for waypoint {i}"
            
            waypoints.append([msg.x, msg.y, msg.z])

        return waypoints, None

    def get_current_waypoint_vector(self):
        """
        Calculate vector between previous and current waypoints.
        
        Returns:
            list: Vector [x, y] between previous and current waypoints
        """
        self.pixhawk.mav.mission_request_list_send()
        current_waypoint_index = None
        current_waypoint = previous_waypoint = None

        while True:
            msg = self.pixhawk.recv_match(type=['MISSION_CURRENT', 'MISSION_ITEM'], blocking=True)
            if msg is None:
                break

            if msg.get_type() == 'MISSION_CURRENT':
                current_waypoint_index = msg.seq
            elif msg.get_type() == 'MISSION_ITEM' and current_waypoint_index is not None:
                if msg.seq == current_waypoint_index - 1:
                    position = self.converter.latlon_to_xy(msg.x, msg.y)
                    previous_waypoint = [position[0], position[1], msg.z]
                if msg.seq == current_waypoint_index:
                    position = self.converter.latlon_to_xy(msg.x, msg.y)
                    current_waypoint = [position[0], position[1], msg.z]
                    break

        return [
            current_waypoint[0] - previous_waypoint[0],
            current_waypoint[1] - previous_waypoint[1]
        ]

    def send_waypoints(self, waypoint_list):
        """
        Upload new mission waypoints to Pixhawk.
        
        Args:
            waypoint_list (list): List of waypoints, each containing [lat, lon, alt]
        """
        waypoints = [
            {'seq': i, 'lat': wp[0], 'lon': wp[1], 'alt': wp[2]}
            for i, wp in enumerate(waypoint_list)
        ]

        self.pixhawk.waypoint_count_send(len(waypoints))
        
        for wp in waypoints:
            self.pixhawk.mav.mission_item_send(
                self.pixhawk.target_system,
                self.pixhawk.target_component,
                wp['seq'],
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,  # current, autocontinue
                0, 0, 0, 0,  # params
                wp['lat'], wp['lon'], wp['alt']
            )

    def move_relative(self, x, y):
        """
        Calculate new position based on current heading and relative movement.
        
        Args:
            x (float): Forward/backward distance
            y (float): Right/left distance
            
        Returns:
            tuple: (target_x, target_y) coordinates
        """
        current_position, _ = self.get_current_xy()
        current_x = current_position['x']
        current_y = current_position['y']
        current_yaw = current_position['yaw']

        theta = math.radians(current_yaw)
        vector = [current_x - x, current_y - y]

        x_goal = current_x + (math.cos(theta) * vector[0] - math.sin(theta) * vector[1])
        y_goal = current_y + (math.sin(theta) * vector[0] + math.cos(theta) * vector[1])

        return x_goal, y_goal

    def hold_position(self):
        """Command vehicle to hold current position using loiter mode."""
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0, 0  # parameters (unused)
        )

    def resume_mission(self):
        """Resume currently loaded mission from next waypoint."""
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_AUTO_ARMED,
            0, 0, 0, 0, 0, 0  # parameters (unused)
        )
