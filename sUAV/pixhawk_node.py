#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import threading
import time
import queue
from typing import List, Optional
import sensor_msgs_py.point_cloud2 as pc2
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Import our custom modules
from lib.obstacle_detector import EnhancedObstacleDetector, Obstacle
from lib.path_planner import PathPlanner, PathSegment

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Initialize components
        self.obstacle_detector = EnhancedObstacleDetector()
        self.path_planner = PathPlanner()
        
        # Threading setup
        self.point_cloud_queue = queue.Queue(maxsize=1)  # Only keep latest scan
        self.current_path: List[PathSegment] = []
        self.path_lock = threading.Lock()

        self.scan_array = None
        self.y_array = None
        self.last_scan_time = time.time()
        
        # Processing threads
        self.obstacle_thread = threading.Thread(target=self._obstacle_processing_loop)
        self.movement_thread = threading.Thread(target=self._movement_execution_loop)
        self.obstacle_thread.daemon = True
        self.movement_thread.daemon = True
        
        # ROS2 subscriber
        self.scan_subscription = self.create_subscription(
            PointCloud2,
            '/scan_3D',  # Using 3D scan topic
            self.scan_3d_callback,
            5
        )
        
        # Drone connection
        print("Connecting to vehicle...")
        self.vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)
        
        # Set up RC channel listener
        self.vehicle.on_message('RC_CHANNELS')(self.rc_listener)
        
        # State variables
        # State variables
        self.mission_count = 0
        self.last_update_time = 0
        self.UPDATE_INTERVAL = 2
        self.in_auto_mode = False
        self.avoiding_obstacle = False
        self.last_channel_press_time = 0
        self.CHANNEL_PRESS_DEBOUNCE = 0.5
        self._last_mode_change_time = 0
        
        # Initial mission download
        self._update_mission_count()
        
        # Start processing threads
        self.obstacle_thread.start()
        self.movement_thread.start()
        
        self.get_logger().info('Drone Controller initialized')

    def _update_mission_count(self):
        """Update mission count with improved timing and state checks"""
        try:
            self.get_logger().info("Starting mission count update...")
            
            # First check if vehicle is ready
            if not self.vehicle or not self.vehicle.is_armable:
                self.get_logger().warn("Vehicle not ready - waiting for vehicle to initialize...")
                # Wait for vehicle to become ready
                start_time = time.time()
                while time.time() - start_time < 10:  # 10 second timeout
                    if self.vehicle and self.vehicle.is_armable:
                        break
                    time.sleep(0.5)
                else:
                    self.get_logger().error("Vehicle failed to become ready!")
                    return False
            
            # Store initial count
            initial_count = self.vehicle.commands.count
            self.get_logger().info(f"Initial command count: {initial_count}")
            
            # Don't clear commands if we already have some
            if initial_count == 0:
                self.get_logger().info("No existing commands, proceeding with download...")
            else:
                self.get_logger().info(f"Found {initial_count} existing commands")
            
            # Download with longer timeout
            self.get_logger().info("Starting command download...")
            self.vehicle.commands.download()
            
            # Wait with periodic status checks
            start_time = time.time()
            max_wait = 15  # 15 second timeout
            check_interval = 1  # Check every second
            
            while time.time() - start_time < max_wait:
                try:
                    if self.vehicle.commands.wait_ready(timeout=check_interval):
                        current_count = self.vehicle.commands.count
                        self.get_logger().info(f"Commands ready! Count: {current_count}")
                        
                        # Verify we didn't lose commands
                        if current_count < initial_count:
                            self.get_logger().warn(f"Command count decreased from {initial_count} to {current_count}")
                            return False
                        
                        self.mission_count = current_count
                        self.last_update_time = time.time()
                        return True
                except TimeoutError:
                    self.get_logger().info("Still waiting for commands...")
                    continue
                    
            self.get_logger().error(f"Command download timed out after {max_wait} seconds")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Exception in _update_mission_count: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def scan_3d_callback(self, msg):
        """Process 3D scan data to get center rows"""
        points_dict = {}
        
        # Read points and organize by height
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            z_key = round(point[2], 2)
            if z_key not in points_dict:
                points_dict[z_key] = []
            points_dict[z_key].append((point[0], point[1]))

        # Get center rows
        z_values = sorted(points_dict.keys())
        if len(z_values) >= 2:
            middle_idx = len(z_values) // 2
            z1, z2 = z_values[middle_idx-1:middle_idx+1]
            center_points = points_dict[z1] + points_dict[z2]
            
            # Extract x and y values
            points = [p[0] for p in center_points]
            y_distances = [p[1] for p in center_points]
            
            self.scan_array = np.array(points)
            self.y_array = np.array(y_distances)
            self.last_scan_time = time.time()
            
            # Process scan data
            self._process_scan_data()

    def _handle_stop_command(self):
        """
        Handle immediate stop command when obstacle is too close.
        Switches to BRAKE mode and hovers in place.
        """
        try:
            # Only proceed if we're in AUTO or GUIDED mode
            if self.vehicle.mode.name in ['AUTO', 'GUIDED']:
                # Switch to BRAKE mode for immediate stop
                self.vehicle.mode = VehicleMode("BRAKE")
                
                # Log the emergency stop
                self.get_logger().warn("Emergency stop initiated - Obstacle too close!")
                
                # Wait for vehicle to come to a complete stop
                time.sleep(2)
                
                # Switch to LOITER mode to maintain position
                self.vehicle.mode = VehicleMode("LOITER")
                
                # Reset path planning state
                with self.path_lock:
                    self.current_path = []
                    self.avoiding_obstacle = False
                
                # Send zero velocity command to ensure stop
                self._send_ned_velocity(0, 0, 0, 1)
                
        except Exception as e:
            self.get_logger().error(f"Error during emergency stop: {str(e)}")
            # Try to force stop even if there's an error
            try:
                self._send_ned_velocity(0, 0, 0, 1)
            except:
                pass

    def _handle_steering(self, steer_angle):
        """
        Handle steering commands for obstacle avoidance.
        
        Args:
            steer_angle (float): Angle to steer in degrees. Positive is right, negative is left.
        """
        try:
            # Only handle steering if we're in AUTO or GUIDED mode
            if self.vehicle.mode.name not in ['AUTO', 'GUIDED']:
                return
                
            # Switch to GUIDED mode for temporary control
            if self.vehicle.mode.name != 'GUIDED':
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(0.5)  # Allow mode switch to complete
                
            # Calculate velocity components based on steering angle
            # Convert angle to radians
            angle_rad = math.radians(steer_angle)
            
            # Current ground speed or default to 5 m/s if not available
            current_speed = self.vehicle.groundspeed if self.vehicle.groundspeed > 0 else 5.0
            
            # Calculate North and East components
            # Using current speed and steering angle to maintain consistent velocity
            velocity_n = current_speed * math.cos(angle_rad)
            velocity_e = current_speed * math.sin(angle_rad)
            
            # Maintain current altitude (zero vertical velocity)
            velocity_d = 0.0
            
            # Send velocity command for a short duration
            self._send_ned_velocity(velocity_n, velocity_e, velocity_d, 0.5)
            
            # Log the steering action
            self.get_logger().info(
                f"Steering adjustment: {steer_angle:.1f}° "
                f"(N:{velocity_n:.1f} m/s, E:{velocity_e:.1f} m/s)"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error during steering adjustment: {str(e)}")
            # Try to maintain stable flight
            try:
                self.vehicle.mode = VehicleMode("LOITER")
            except:
                pass

    def _process_scan_data(self):
        """Process scan data for both immediate obstacle avoidance and path planning"""
        if time.time() - self.last_scan_time < 3.0 and self.scan_array is not None:
            # Get obstacles and steering command
            obstacles, steer_angle = self.obstacle_detector.process_scan_data(
                self.scan_array, self.y_array)
                
            if steer_angle == "STOP":
                # Handle immediate stop
                self._handle_stop_command()
                self.get_logger().warn("STOP COMMAND: Obstacle too close!")
            elif abs(steer_angle) > 0.1:  # Small threshold to avoid minor adjustments
                # Handle steering
                self._handle_steering(steer_angle)
                self.get_logger().info(f"Steering adjustment: {steer_angle:.2f} degrees")
            
            # Update path planning if obstacles detected
            if obstacles and self.in_auto_mode:
                self._update_path_planning(obstacles)
                # Log obstacle positions
                for i, obs in enumerate(obstacles):
                    self.get_logger().info(
                        f"Obstacle {i}: Position (x:{obs.center[0]:.2f}m, y:{obs.center[1]:.2f}m, z:{obs.center[2]:.2f}m), "
                        f"Radius: {obs.radius:.2f}m"
                    )

    def rc_listener(self, _, name, message):
        """Handle RC channel messages with improved debouncing and state checks"""
        current_time = time.time()
        
        # More aggressive debouncing
        if current_time - self.last_channel_press_time < 1.0:  # Increased to 1 second
            return
            
        if message.chan12_raw > 1000:
            self.last_channel_press_time = current_time
            self.get_logger().info("Channel 12 pressed!")
            
            # Prevent rapid mode changes
            if hasattr(self, '_last_mode_change_time'):
                if current_time - self._last_mode_change_time < 2.0:  # 2 second cooldown
                    self.get_logger().warn("Mode change too soon - please wait")
                    return
            
            if not self.in_auto_mode:
                self.get_logger().info("Attempting switch to AUTO mode...")
                
                # Verify vehicle state
                if not self.vehicle.is_armable:
                    self.get_logger().error("Vehicle not armable - cannot switch to AUTO")
                    return
                    
                # Update mission with longer timeout
                update_success = self._update_mission_count()
                self.get_logger().info(f"Mission update success: {update_success}")
                
                if update_success and self.mission_count > 0:
                    try:
                        self._last_mode_change_time = current_time
                        self.vehicle.mode = VehicleMode("AUTO")
                        
                        # Wait for mode change to take effect
                        mode_change_start = time.time()
                        while time.time() - mode_change_start < 3:
                            if self.vehicle.mode.name == "AUTO":
                                self.in_auto_mode = True
                                self.get_logger().info("Successfully switched to AUTO mode")
                                return
                            time.sleep(0.1)
                        
                        self.get_logger().error("Failed to verify AUTO mode change")
                        
                    except Exception as e:
                        self.get_logger().error(f"Error switching to AUTO mode: {str(e)}")
                else:
                    self.get_logger().error("Mission update failed or no waypoints available")
                    
            elif self.in_auto_mode:
                try:
                    self._last_mode_change_time = current_time
                    self.vehicle.mode = VehicleMode("STABILIZE")
                    self.in_auto_mode = False
                    self.avoiding_obstacle = False
                    self.get_logger().info("Switched to STABILIZE mode")
                except Exception as e:
                    self.get_logger().error(f"Error switching to STABILIZE: {str(e)}")

    def _check_mission_available(self):
        """Check if there are any mission waypoints loaded with improved reliability"""
        current_time = time.time()
        
        # Force update if too much time has passed or no mission count
        if (current_time - self.last_update_time > self.UPDATE_INTERVAL) or (self.mission_count == 0):
            success = self._update_mission_count()
            if not success:
                self.get_logger().warning("Mission update failed, using cached count")
                
        return self.mission_count > 0

    def lidar_callback(self, msg: PointCloud2):
        """Process incoming LiDAR data"""
        try:
            # Convert PointCloud2 to numpy array
            points = np.array(list(pc2.read_points(msg, 
                                                 field_names=('x', 'y', 'z'),
                                                 skip_nans=True)))
            
            # Update queue with latest scan (drops old scan if full)
            try:
                self.point_cloud_queue.put_nowait(points)
            except queue.Full:
                try:
                    self.point_cloud_queue.get_nowait()
                    self.point_cloud_queue.put_nowait(points)
                except queue.Empty:
                    pass
                    
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {e}')

    def _obstacle_processing_loop(self):
        """Continuous obstacle detection and path planning"""
        while rclpy.ok():
            try:
                # Get latest pointcloud
                points = self.point_cloud_queue.get(timeout=0.1)
                
                # Detect obstacles
                obstacles = self.obstacle_detector.process_pointcloud(points)
                
                if obstacles and self.in_auto_mode:
                    self.get_logger().info(f"Detected {len(obstacles)} obstacles")
                    # Get current mission waypoint
                    next_waypoint = self._get_next_waypoint()
                    if next_waypoint is None:
                        continue
                    
                    # Log current position and goal
                    current_pos = np.array([
                        self.vehicle.location.global_relative_frame.lat,
                        self.vehicle.location.global_relative_frame.lon,
                        self.vehicle.location.global_relative_frame.alt
                    ])
                    
                    goal_pos = np.array([
                        next_waypoint.lat,
                        next_waypoint.lon,
                        next_waypoint.alt
                    ])
                    
                    self.get_logger().info(
                        f"Current position: lat:{current_pos[0]:.6f}, lon:{current_pos[1]:.6f}, alt:{current_pos[2]:.1f}m"
                        f"\nGoal position: lat:{goal_pos[0]:.6f}, lon:{goal_pos[1]:.6f}, alt:{goal_pos[2]:.1f}m"
                    )
                    
                    with self.path_lock:
                        self.current_path = self.path_planner.find_safe_path(
                            current_pos, goal_pos, obstacles)
                        
                        if len(self.current_path) > 1:  # Need to avoid obstacle
                            self.avoiding_obstacle = True
                            self.get_logger().info(
                                f"Replanning path with {len(self.current_path)} segments to avoid obstacles"
                            )
                            
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in obstacle processing: {e}')
                time.sleep(0.1)

    def _movement_execution_loop(self):
        """Execute the planned path"""
        while rclpy.ok():
            try:
                if self.avoiding_obstacle and self.in_auto_mode:
                    with self.path_lock:
                        if not self.current_path:
                            continue
                            
                        next_segment = self.current_path[0]
                        
                    if self._execute_path_segment(next_segment):
                        # Segment completed
                        with self.path_lock:
                            self.current_path.pop(0)
                            if not self.current_path:
                                self.avoiding_obstacle = False
                                # Resume mission
                                self.vehicle.mode = VehicleMode("AUTO")
                                
            except Exception as e:
                self.get_logger().error(f'Error in movement execution: {e}')
                
            time.sleep(0.1)

    def _execute_path_segment(self, segment: PathSegment) -> bool:
        """Execute a path segment, return True when complete"""
        # Convert segment end to relative movement
        current_pos = np.array([
            self.vehicle.location.global_relative_frame.lat,
            self.vehicle.location.global_relative_frame.lon,
            self.vehicle.location.global_relative_frame.alt
        ])
        
        # Calculate relative movement
        relative_movement = segment.end - current_pos
        
        # Convert to NED frame
        ned_movement = self._geodetic_to_ned(relative_movement)
        
        # Log movement plan
        self.get_logger().info(
            f"Moving to: lat:{segment.end[0]:.6f}, lon:{segment.end[1]:.6f}, alt:{segment.end[2]:.1f}m"
            f"\nRelative movement (NED): N:{ned_movement[0]:.2f}m, E:{ned_movement[1]:.2f}m, D:{ned_movement[2]:.2f}m"
        )
        
        # Send movement command
        self.vehicle.mode = VehicleMode("GUIDED")
        self._send_ned_velocity(
            ned_movement[0],  # North
            ned_movement[1],  # East
            ned_movement[2],  # Down
            1.0  # Duration
        )
        
        # Check if we've reached the end of the segment
        distance = np.linalg.norm(relative_movement)
        if distance < 0.5:  # 0.5m threshold
            self.get_logger().info("Waypoint reached!")
        return distance < 0.5

    def _send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """Send NAV_VELOCITY_NED command to vehicle"""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)
        
        # Send command for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def _geodetic_to_ned(self, geodetic_vector: np.ndarray) -> np.ndarray:
        """Convert geodetic (lat, lon, alt) vector to NED frame"""
        # Simplified conversion - you might want to use a proper geodetic library
        # for more accurate conversion
        EARTH_RADIUS = 6378137.0  # meters
        
        ref_lat = self.vehicle.location.global_relative_frame.lat
        ref_lon = self.vehicle.location.global_relative_frame.lon
        
        north = geodetic_vector[0] * np.pi/180.0 * EARTH_RADIUS
        east = geodetic_vector[1] * np.pi/180.0 * EARTH_RADIUS * np.cos(ref_lat * np.pi/180.0)
        down = -geodetic_vector[2]  # Negative because NED coordinates
        
        return np.array([north, east, down])

    def _get_next_waypoint(self) -> Optional[LocationGlobalRelative]:
        """Get the next waypoint from the mission"""
        if not self.vehicle.commands or self.vehicle.commands.count == 0:
            return None
            
        next_wp_num = self.vehicle.commands.next
        if next_wp_num >= self.vehicle.commands.count:
            return None
            
        # Download the mission if needed
        if time.time() - self.last_update_time > self.UPDATE_INTERVAL:
            self.vehicle.commands.download()
            self.vehicle.commands.wait_ready()
            self.last_update_time = time.time()
            
        # Get the next waypoint
        next_wp = self.vehicle.commands[next_wp_num]
        if next_wp.command == 16:  # MAV_CMD_NAV_WAYPOINT
            return LocationGlobalRelative(
                next_wp.x,  # lat
                next_wp.y,  # lon
                next_wp.z   # altitude (relative)
            )
        return None

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = DroneController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if 'controller' in locals():
            controller.vehicle.close()
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()