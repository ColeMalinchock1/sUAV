#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import logging
from typing import Optional, Tuple, List, Dict
import numpy as np
from math import cos, sin, radians, atan2, sqrt, degrees
from filterpy.kalman import ExtendedKalmanFilter as EKF
from scipy.spatial.transform import Rotation
from threading import Thread, Lock

class PixhawkController(Node):
    def __init__(self, connection_string: str = '/dev/ttyTHS1', baud: int = 57600,
                 safety_distance: float = 5.0, avoidance_distance: float = 8.0,
                 lidar_topic: str = '/scan', pose_topic: str = '/zed2i/pose'):
        """
        Initialize the PixhawkController.
        
        Args:
            connection_string: Serial port for Pixhawk connection
            baud: Baud rate for serial connection
            safety_distance: Minimum distance to maintain from obstacles (meters)
            avoidance_distance: Distance at which to begin avoidance maneuvers (meters)
            lidar_topic: ROS2 topic for LIDAR data
            pose_topic: ROS2 topic for ZED2i pose data
        """
        super().__init__('pixhawk_controller')
        
        self.connection_string = connection_string
        self.baud = baud
        self.vehicle = None
        self.logger = self._setup_logger()
        self.safety_distance = safety_distance
        self.avoidance_distance = avoidance_distance
        self.current_waypoint = None
        self.avoiding_obstacle = False
        self.original_mission = None

        # ROS2 subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            5)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10)

        # Data storage with thread safety
        self.data_lock = Lock()
        self.latest_lidar_data = {}
        self.latest_slam_data = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0
        }
        self.last_pose_time = None
        
        # Position fusion attributes
        self.ekf = self._initialize_ekf()
        self.last_slam_update = None
        self.last_gps_position = None
        self.initial_gps_position = None
        self.gps_trust_threshold = 6  # Minimum satellites for high GPS trust
        
    def _initialize_ekf(self) -> EKF:
        """Initialize Extended Kalman Filter for position fusion."""
        ekf = EKF(dim_x=9, dim_z=6)  # State: [x, y, z, vx, vy, vz, ax, ay, az], Measurements: [x, y, z, vx, vy, vz]
        
        # Initial state uncertainty
        ekf.P *= 10
        
        # Process noise
        ekf.Q = np.eye(9) * 0.1
        ekf.Q[6:, 6:] *= 0.5  # Lower process noise for accelerations
        
        # Measurement noise - will be adjusted based on GPS quality
        ekf.R = np.eye(6)
        
        return ekf

    def update_position_estimate(self, slam_data: Dict[str, float], timestamp: float) -> Tuple[float, float, float]:
        """
        Update position estimate using SLAM and GPS data fusion.
        
        Args:
            slam_data: Dictionary containing SLAM position and velocity estimates
            timestamp: Current timestamp
            
        Returns:
            Tuple[float, float, float]: Estimated position (latitude, longitude, altitude)
        """
        if not self.vehicle:
            return None
            
        # Get current GPS data
        gps_position = self.vehicle.location.global_relative_frame
        num_satellites = self.vehicle.gps_0.satellites_visible
        
        # Initialize reference position if needed
        if self.initial_gps_position is None and gps_position is not None:
            self.initial_gps_position = gps_position
            
        # Convert GPS to local coordinates
        if gps_position is not None:
            gps_local = self._gps_to_local(gps_position)
        else:
            gps_local = None
            
        # Predict step
        dt = timestamp - self.last_slam_update if self.last_slam_update else 0.1
        self.ekf.predict(dt=dt)
        
        # Update measurement noise based on GPS quality
        self._adjust_measurement_noise(num_satellites)
        
        # Prepare measurement vector
        z = np.zeros(6)
        z[:3] = [slam_data['x'], slam_data['y'], slam_data['z']]
        z[3:] = [slam_data['vx'], slam_data['vy'], slam_data['vz']]
        
        # If GPS is available, fuse it with SLAM data
        if gps_local is not None:
            z = self._fuse_measurements(z, gps_local, num_satellites)
            
        # Update step
        self.ekf.update(z)
        
        # Convert local position back to GPS coordinates
        estimated_position = self._local_to_gps(self.ekf.x[:3])
        
        self.last_slam_update = timestamp
        return estimated_position
        
    def _adjust_measurement_noise(self, num_satellites: int) -> None:
        """Adjust EKF measurement noise based on GPS quality."""
        if num_satellites >= self.gps_trust_threshold:
            # High GPS trust - lower measurement noise
            self.ekf.R[:3, :3] = np.eye(3) * 2.0  # Position noise
            self.ekf.R[3:, 3:] = np.eye(3) * 0.5   # Velocity noise
        else:
            # Low GPS trust - higher measurement noise
            gps_factor = max(1, (self.gps_trust_threshold - num_satellites))
            self.ekf.R[:3, :3] = np.eye(3) * (2.0 * gps_factor)  # Increase position noise
            self.ekf.R[3:, 3:] = np.eye(3) * 0.5  # Keep velocity noise constant
            
    def _fuse_measurements(self, slam_measurement: np.ndarray, gps_local: np.ndarray, 
                          num_satellites: int) -> np.ndarray:
        """Fuse SLAM and GPS measurements based on GPS quality."""
        if num_satellites >= self.gps_trust_threshold:
            # High GPS trust - equal weighting
            weight_gps = 0.5
        else:
            # Low GPS trust - favor SLAM
            weight_gps = max(0.1, num_satellites / (self.gps_trust_threshold * 2))
            
        weight_slam = 1 - weight_gps
        
        # Fuse position measurements
        fused_measurement = slam_measurement.copy()
        fused_measurement[:3] = (slam_measurement[:3] * weight_slam + 
                               gps_local * weight_gps)
        
        return fused_measurement
        
    def _gps_to_local(self, gps_position) -> np.ndarray:
        """Convert GPS coordinates to local frame relative to initial position."""
        if self.initial_gps_position is None:
            return np.zeros(3)
            
        # Calculate offset in meters
        dlat = (gps_position.lat - self.initial_gps_position.lat) * 111319.5
        dlon = (gps_position.lon - self.initial_gps_position.lon) * \
               (111319.5 * cos(radians(self.initial_gps_position.lat)))
        dalt = gps_position.alt - self.initial_gps_position.alt
        
        return np.array([dlat, dlon, dalt])
        
    def _local_to_gps(self, local_position: np.ndarray) -> LocationGlobalRelative:
        """Convert local frame coordinates back to GPS coordinates."""
        if self.initial_gps_position is None:
            return None
            
        # Convert meters back to degrees
        dlat = local_position[0] / 111319.5
        dlon = local_position[1] / \
               (111319.5 * cos(radians(self.initial_gps_position.lat)))
        
        new_lat = self.initial_gps_position.lat + dlat
        new_lon = self.initial_gps_position.lon + dlon
        new_alt = self.initial_gps_position.alt + local_position[2]
        
        return LocationGlobalRelative(new_lat, new_lon, new_alt)
        
    def lidar_callback(self, msg: LaserScan) -> None:
        """
        Handle incoming LIDAR data from ROS2.
        
        Args:
            msg: LaserScan message containing LIDAR data
        """
        with self.data_lock:
            
            # Convert LaserScan to our angle:distance dictionary format
            angle = msg.angle_min
            self.latest_lidar_data = {}
            
            for distance in msg.ranges:
                if msg.range_min <= distance <= msg.range_max:
                    self.latest_lidar_data[degrees(angle)] = distance
                angle += msg.angle_increment

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Handle incoming pose data from ZED2i.
        
        Args:
            msg: PoseStamped message containing camera pose data
        """
        current_time = time.time()
        
        with self.data_lock:
            # Position data
            self.latest_slam_data['x'] = msg.pose.position.x
            self.latest_slam_data['y'] = msg.pose.position.y
            self.latest_slam_data['z'] = msg.pose.position.z
            
            # Calculate velocity if we have previous data
            if self.last_pose_time is not None:
                dt = current_time - self.last_pose_time
                if dt > 0:
                    self.latest_slam_data['vx'] = (msg.pose.position.x - self.latest_slam_data['x']) / dt
                    self.latest_slam_data['vy'] = (msg.pose.position.y - self.latest_slam_data['y']) / dt
                    self.latest_slam_data['vz'] = (msg.pose.position.z - self.latest_slam_data['z']) / dt
            
            self.last_pose_time = current_time

    def get_latest_lidar_data(self) -> Dict[float, float]:
        """Thread-safe getter for LIDAR data."""
        with self.data_lock:
            return self.latest_lidar_data.copy()

    def get_latest_slam_data(self) -> Tuple[Dict[str, float], float]:
        """Thread-safe getter for SLAM data."""
        with self.data_lock:
            return self.latest_slam_data.copy(), time.time()

    def _setup_logger(self) -> logging.Logger:
        """Configure logging for the controller."""
        logger = logging.getLogger('PixhawkController')
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        return logger

    def connect(self) -> bool:
        try:
            self.logger.info(f"Connecting to vehicle on: {self.connection_string}")
            self.vehicle = connect(self.connection_string, baud=self.baud, wait_ready=True)
            self.logger.info("Vehicle connected successfully")
            time.sleep(2)  # Add delay to allow all systems to initialize
            return True
        except Exception as e:
            self.logger.error(f"Connection failed: {str(e)}")
            return False

    def preflight_check(self) -> bool:
        """
        Perform comprehensive preflight checks.
        """
        if not self.vehicle:
            self.logger.error("Vehicle not connected")
            return False

        # First initialize position estimation
        self.logger.info("Initializing position estimation...")
        retry_count = 0
        while retry_count < 10:  # Try for up to 10 seconds
            try:
                # Get initial SLAM data
                slam_data, timestamp = self.get_latest_slam_data()
                
                # Update position estimate
                estimated_position = self.update_position_estimate(slam_data, timestamp)
                
                if estimated_position:
                    self._send_position_update(estimated_position)
                    self.logger.info("Position estimation initialized")
                    break
                    
            except Exception as e:
                self.logger.warning(f"Position estimation init attempt {retry_count}: {str(e)}")
                
            retry_count += 1
            time.sleep(1)

        # Then perform other checks
        checks = [
            self._check_rc_channels(),
            self._check_battery(),
            self._check_gps(),        # Move GPS check before armable check
            self._check_mode(),
            self._check_armable()    # Move armable check after position init
            
        ]
        
        return all(checks)

    def _check_battery(self) -> bool:
        """Check if battery level is sufficient."""
        if self.vehicle.battery.level < 50:
            self.logger.warning(f"Low battery: {self.vehicle.battery.level}%")
            return False
        return True

    def _check_armable(self) -> bool:
        """Check if vehicle is armable."""
        if not self.vehicle.is_armable:
            self.logger.warning(f"Vehicle not armable. Status: GPS: {self.vehicle.gps_0.fix_type}, EKF: {self.vehicle.ekf_ok}")
            return False
        return True

    def _check_gps(self) -> bool:
        """Check GPS health."""
        if self.vehicle.gps_0.fix_type < 3:
            self.logger.warning("Insufficient GPS fix")
            return False
        return True

    def _check_mode(self) -> bool:
        """Check if vehicle is in correct mode."""
        if self.vehicle.mode.name != "GUIDED":
            self.logger.info("Setting mode to GUIDED")
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
        return self.vehicle.mode.name == "GUIDED"

    def _check_rc_channels(self) -> bool:
        try:
            rc_channels = [self.vehicle.channels[i] for i in range(1, 13)]
            for i, ch in enumerate(rc_channels, 1):
                if ch is None:
                    self.logger.warning(f"RC channel {i} not receiving values")
            if any(ch is None for ch in rc_channels):
                return False
            return True
        except Exception as e:
            self.logger.error(f"RC channel check failed: {str(e)}")
            return False

    def takeoff(self, target_altitude: float) -> bool:
        """
        Arm and takeoff to specified altitude.
        
        Args:
            target_altitude: Target altitude in meters
            
        Returns:
            bool: True if takeoff successful, False otherwise
        """
        if not self.vehicle or not self.preflight_check():
            return False

        try:
            self.logger.info("Arming vehicle...")
            self.vehicle.armed = True
            time.sleep(2)

            if not self.vehicle.armed:
                self.logger.error("Failed to arm vehicle")
                return False

            self.logger.info(f"Taking off to {target_altitude}m...")
            self.vehicle.simple_takeoff(target_altitude)

            # Wait until target altitude reached
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                self.logger.info(f"Altitude: {current_altitude}m")
                
                if current_altitude >= target_altitude * 0.95:
                    self.logger.info("Target altitude reached")
                    break
                    
                time.sleep(1)
                
                # Check if RC override is active
                if self._is_rc_override_active():
                    self.logger.warning("RC override activated during takeoff")
                    return False

            return True

        except Exception as e:
            self.logger.error(f"Takeoff failed: {str(e)}")
            return False

    def _is_rc_override_active(self) -> bool:
        """
        Check if RC override is active based on switch position.
        Assumes channel 8 is used for override switch.
        """
        try:
            override_channel = self.vehicle.channels['8']
            # Typically PWM > 1800 indicates switch in override position
            return override_channel > 1800
        except Exception as e:
            self.logger.error(f"Failed to check RC override: {str(e)}")
            return False

    def hold_position(self) -> None:
        """Hold current position."""
        if self.vehicle and self.vehicle.armed:
            current_location = self.vehicle.location.global_relative_frame
            self.vehicle.simple_goto(current_location)

    def land(self) -> bool:
        """
        Land the vehicle.
        
        Returns:
            bool: True if landing initiated successfully, False otherwise
        """
        try:
            self.logger.info("Initiating landing...")
            self.vehicle.mode = VehicleMode("LAND")
            return True
        except Exception as e:
            self.logger.error(f"Landing failed: {str(e)}")
            return False

    def disconnect(self) -> None:
        """Close vehicle connection."""
        if self.vehicle:
            self.vehicle.close()
            self.logger.info("Vehicle disconnected")

    def handle_obstacle(self, lidar_data: Dict[float, float]) -> None:
        """
        Process LIDAR data and execute avoidance maneuvers if needed.
        
        Args:
            lidar_data: Dictionary of angles (in degrees) to distances (in meters)
        """
        if not self.vehicle or not self.vehicle.armed:
            return

        # Find the closest obstacle
        min_distance = float('inf')
        min_angle = 0
        
        for angle, distance in lidar_data.items():
            if distance < min_distance:
                min_distance = distance
                min_angle = angle

        if min_distance < self.avoidance_distance:
            self.avoiding_obstacle = True
            self._execute_avoidance_maneuver(min_distance, min_angle)
        elif self.avoiding_obstacle:
            self._resume_mission()

    def _execute_avoidance_maneuver(self, distance: float, obstacle_angle: float) -> None:
        """
        Execute an avoidance maneuver based on obstacle position.
        
        Args:
            distance: Distance to obstacle in meters
            obstacle_angle: Angle to obstacle in degrees
        """
        current_pos = self.vehicle.location.global_relative_frame
        vehicle_heading = self.vehicle.heading

        # Convert obstacle angle to global frame
        global_obstacle_angle = (vehicle_heading + obstacle_angle) % 360

        # Calculate avoidance direction (perpendicular to obstacle direction)
        avoidance_angle = (global_obstacle_angle + 90) % 360
        
        # Calculate avoidance distance based on how close we are to the obstacle
        avoidance_magnitude = self.avoidance_distance * (1 - distance / self.avoidance_distance)
        
        # Calculate new position
        delta_north = avoidance_magnitude * cos(radians(avoidance_angle))
        delta_east = avoidance_magnitude * sin(radians(avoidance_angle))
        
        # Create new waypoint
        next_pos = self._get_location_offset_meters(current_pos, delta_north, delta_east)
        
        # Move to avoidance position
        self.vehicle.simple_goto(next_pos)
        self.logger.info(f"Executing avoidance maneuver: {avoidance_angle}°, {avoidance_magnitude}m")

    def _resume_mission(self) -> None:
        """Resume the original mission after avoiding an obstacle."""
        self.avoiding_obstacle = False
        if self.current_waypoint:
            self.vehicle.simple_goto(self.current_waypoint)
            self.logger.info("Resuming mission")

    def _get_location_offset_meters(self, original_location, dNorth, dEast) -> LocationGlobalRelative:
        """
        Calculate new LocationGlobalRelative based on offset from original location.
        
        Args:
            original_location: Original GPS location
            dNorth: Meters north of original location
            dEast: Meters east of original location
            
        Returns:
            New LocationGlobalRelative
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth

        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * cos(radians(original_location.lat)))

        # New position in decimal degrees
        newlat = original_location.lat + degrees(dLat)
        newlon = original_location.lon + degrees(dLon)

        return LocationGlobalRelative(newlat, newlon, original_location.alt)

    def wait_for_mission_complete(self, lidar_callback, slam_callback) -> bool:
        """
        Wait for mission to complete while monitoring RC override, obstacles, and position.
        
        Args:
            lidar_callback: Function that returns current LIDAR data as Dict[angle, distance]
            slam_callback: Function that returns SLAM position data and timestamp
            
        Returns:
            bool: True if mission completed successfully, False if interrupted
        """
        try:
            while True:
                if self._is_rc_override_active():
                    self.logger.warning("RC override activated - ending autonomous control")
                    return False
                
                # Update position estimate with SLAM data
                slam_data, timestamp = slam_callback()
                estimated_position = self.update_position_estimate(slam_data, timestamp)
                
                if estimated_position:
                    # Update vehicle position in Pixhawk
                    self._send_position_update(estimated_position)
                
                # Get current LIDAR data and handle any obstacles
                lidar_data = lidar_callback()
                self.handle_obstacle(lidar_data)
                
                # Store current waypoint for mission resumption
                if not self.avoiding_obstacle:
                    self.current_waypoint = self.vehicle.commands[self.vehicle.commands.next]
                
                # Check if mission is complete
                if self.vehicle.commands.next == self.vehicle.commands.count:
                    self.logger.info("Mission complete")
                    return True
                
                time.sleep(0.1)  # Faster update rate for sensor fusion
                
        except Exception as e:
            self.logger.error(f"Error during mission monitoring: {str(e)}")
            return False
            
    def _send_position_update(self, position: LocationGlobalRelative) -> None:
        """
        Send position update to Pixhawk's EKF.
        
        Args:
            position: Estimated position from sensor fusion
        """
        try:
            # Send vision position estimate message
            msg = self.vehicle.message_factory.vision_position_estimate_encode(
                int(time.time() * 1e6),  # timestamp in microseconds
                position.lat,
                position.lon,
                position.alt,
                0, 0, 0  # rotation in radians (we're not estimating rotation)
            )
            self.vehicle.send_mavlink(msg)
            
        except Exception as e:
            self.logger.error(f"Failed to send position update: {str(e)}")
                
        except Exception as e:
            self.logger.error(f"Error during mission monitoring: {str(e)}")
            return False

def main():
    """Example usage of the PixhawkController class."""
    rclpy.init()
    
    # Initialize controller
    controller = PixhawkController()
    
    # Create a separate thread for ROS2 spin
    ros_thread = Thread(target=lambda: rclpy.spin(controller))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Connect to vehicle
        if not controller.connect():
            return

        # Perform preflight checks
        if not controller.preflight_check():
            controller.disconnect()
            return

        # Take off to 10 meters
        if not controller.takeoff(10):
            controller.land()
            controller.disconnect()
            return

        # Hold position and wait for mission
        controller.hold_position()
        
        # Wait for mission completion or RC override
        # Use the internal data getters for LIDAR and SLAM data
        if controller.wait_for_mission_complete(
            controller.get_latest_lidar_data,
            controller.get_latest_slam_data
        ):
            # Land after successful mission
            controller.land()
        
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        # Ensure we always disconnect and shutdown properly
        controller.disconnect()
        rclpy.shutdown()

if __name__ == "__main__":
    main()