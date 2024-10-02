from pymavlink import mavutil
import socket
from lib.coords_to_cartesian import CoordsToCartesian as c2c
import math
import time

class PixhawkCommands():

    def __init__(self, serial_port, baud_rate):
        self.pixhawk = mavutil.mavlink_connection(serial_port, baud=baud_rate)
        self.pixhawk.wait_heartbeat()

        print("Connected to pixhawk")

        self.pixhawk.mav.request_data_stream_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1,
            1
        )

        current_latlon = self.get_current_latlon()

        while current_latlon is None:
            current_latlon = self.get_current_latlon()

        self.converter = c2c(current_latlon[0], current_latlon[1])

    def get_current_xy(self, timeout=10):
        """
        Returns the current position of the pixhawk as a dictionary with the following format:
        x, y, z, yaw
        """

        # Wait for the GLOBAL_POSITION_INT message
        msg = self.pixhawk.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            print("Failed to receive GLOBAL_POSITION_INT")
            return None
        
        # Extract latitude, longitude, and altitude from the message
        lat = msg.lat
        lon = msg.lon
        alt = msg.alt
        hdg = msg.hdg

        current_position = self.converter.latlon_to_xy(lat, lon)

        location = {
            'x': current_position[0],
            'y': current_position[1],
            'z': alt,
            'yaw': self.converter.compass_heading_to_yaw(hdg)
        }

        return location

    def get_current_latlon(self, timeout=10):
        """
        Returns the current position of the pixhawk as a dictionary with the following format:
        latitude, longitude, altitude, heading
        """
        # Wait for the GLOBAL_POSITION_INT message
        msg = self.pixhawk.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            print("Failed to receive GLOBAL_POSITION_INT")
            return None

        # Extract and convert latitude, longitude, altitude, and heading
        lat = msg.lat / 1e7  # Convert from int to degrees
        lon = msg.lon / 1e7  # Convert from int to degrees
        alt = msg.alt / 1000.0  # Convert from mm to meters
        hdg = msg.hdg / 100.0 if msg.hdg is not None else None  # Convert from centidegrees to degrees

        position = [lat, lon, alt, hdg]

        return position

    
    def get_waypoints(self, timeout=10):
        # Request the total number of waypoints
        self.pixhawk.waypoint_request_list_send()

        # Wait for the MISSION_COUNT message
        msg = self.pixhawk.recv_match(type='MISSION_COUNT', blocking=True, timeout=timeout)
        
        if msg is None:
            print("Failed to receive MISSION_COUNT")
            return []

        waypoint_count = msg.count

        waypoints = []

        for i in range(waypoint_count):
            # Request individual waypoint
            self.pixhawk.waypoint_request_send(i)
            
            # Wait for the corresponding MISSION_ITEM message
            msg = self.pixhawk.recv_match(type='MISSION_ITEM', blocking=True, timeout=timeout)
            
            if msg is None:
                print(f"Failed to receive MISSION_ITEM for waypoint {i}")
                break
            
            waypoint = [msg.x, msg.y, msg.z]

            waypoints.append(waypoint)

        return waypoints
    
    def get_current_waypoint_vector(self):
        """
        Gets the previous and current waypoint to be used to calculate the current vector between the two waypoints
        """

        # Request the current mission state
        self.pixhawk.mav.mission_request_list_send()
        
        current_waypoint_index = None
        current_waypoint = previous_waypoint = None

        while True:
            msg = self.pixhawk.recv_match(type=['MISSION_CURRENT', 'MISSION_ITEM'], blocking=True)

            if msg is None:
                break

            if msg.get_type() == 'MISSION_CURRENT':
                current_waypoint_index = msg.seq
                print(f"Current waypoint index: {current_waypoint_index}")

            elif msg.get_type() == 'MISSION_ITEM' and current_waypoint_index is not None:
                
                if msg.seq == current_waypoint_index - 1:
                    position = self.converter.latlon_to_xy(msg.x, msg.y)
                    previous_waypoint = [position[0], position[1], msg.z]

                if msg.seq == current_waypoint_index:
                    position = self.converter.latlon_to_xy(msg.x, msg.y)
                    current_waypoint = [position[0], position[1], msg.z]

                    print(f"Current waypoint: {current_waypoint}")
                    break
        
        vector = [current_waypoint[0] - previous_waypoint[0], current_waypoint[1] - previous_waypoint[1]]

        return vector
    
    def send_waypoints(self, waypoint_list):

        waypoints = [{'seq': i, 'lat': wp[0], 'lon': wp[1], 'alt': wp[2]} for i, wp in enumerate(waypoint_list)]

        self.pixhawk.waypoint_count_send(len(waypoints))
    
        for wp in waypoints:
            print(f"Sending waypoint: seq={wp['seq']}, lat={wp['lat']}, lon={wp['lon']}, alt={wp['alt']}")
            self.pixhawk.mav.mission_item_send(self.pixhawk.target_system,
                                        self.pixhawk.target_component,
                                        wp['seq'],
                                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                        0, 1, 0, 0, 0, 0, wp['lat'], wp['lon'], wp['alt'])

    def move_relative(self, x, y):
        """
        Based off of the current direction that the sUAV is facing, move relative to the x or y the specified amount
        """

        current_position = self.get_current_xy()

        current_x = current_position[0]
        current_y = current_position[1]
        current_yaw = current_position[3]

        theta = math.radians(current_yaw)

        vector = [current_x - x, current_y - y]

        x_goal = current_x + (math.cos(theta) * vector[0] - math.sin(theta) * vector[1])
        y_goal = current_y + (math.sin(theta) * vector[0] + math.cos(theta) * vector[1])

        return x_goal, y_goal
    
    def hold_position(self):
        """
        Command the Pixhawk to hold its position at the current location
        """
        # Send a loiter command to hold position
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # Command to loiter indefinitely
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0, 0  # Params are not used for this command
        )

    def resume_mission(self):
        """
        Resume the mission by commanding the Pixhawk to continue to the next waypoint
        """
        # Send a command to continue the mission
        self.pixhawk.mav.command_long_send(
            self.pixhawk.target_system,
            self.pixhawk.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command to set mode
            0,  # Confirmation
            mavutil.mavlink.MAV_MODE_AUTO_ARMED,  # Set mode to AUTO
            0, 0, 0, 0, 0, 0  # Params are not used for this command
        )