from dataclasses import dataclass
import numpy as np
from typing import List, Tuple, Optional
import math

@dataclass
class Obstacle:
    center: np.ndarray  # [x, y, z]
    radius: float
    confidence: float

class EnhancedObstacleDetector:
    def __init__(self, 
                 min_points: int = 10,
                 cluster_threshold: float = 0.5,
                 vehicle_size: float = 5.0,
                 max_threshold: float = 2.0,
                 safety_margin: float = 0.5,
                 steering_sensitivity: float = 1.5):
        """
        Initialize the obstacle detector with improved parameters.
        
        Args:
            min_points: Minimum points to consider a cluster
            cluster_threshold: Distance threshold for clustering points
            vehicle_size: Size of vehicle in meters
            max_threshold: Maximum distance to consider for obstacles
            safety_margin: Additional safety distance buffer
            steering_sensitivity: Multiplier for steering response
        """
        self.min_points = min_points
        self.cluster_threshold = cluster_threshold
        self.VEHICLE_SIZE = vehicle_size
        self.MAX_THRESHOLD = max_threshold
        self.SAFETY_MARGIN = safety_margin
        self.STEERING_SENSITIVITY = steering_sensitivity
        
        # Emergency stop thresholds
        self.CRITICAL_DISTANCE = max_threshold * 0.3  # 30% of max threshold
        self.MIN_OPENING_WIDTH = vehicle_size * 1.2  # 20% wider than vehicle
        
        # Store previous scan data
        self.last_scan = None
        self.last_y_distances = None
        
        # Smoothing parameters
        self.last_steering_angle = 0
        self.steering_smoothing = 0.7  # 70% previous value, 30% new value

    def process_scan_data(self, scan_array: np.ndarray, y_array: np.ndarray) -> Tuple[List[Obstacle], float]:
        """
        Process scan data with improved obstacle detection and smoother steering.
        """
        if scan_array is None or len(scan_array) == 0:
            return [], 0.0

        self.last_scan = scan_array
        self.last_y_distances = y_array

        # Check for critical obstacles
        if self._check_critical_obstacles(scan_array):
            return [], "STOP"

        # Find valid openings
        openings = self._find_openings(scan_array)
        if not openings:
            return [], "STOP"

        # Get best opening and calculate steering
        best_opening = self._select_best_opening(openings, scan_array)
        steer_angle = self._calculate_steering(scan_array, y_array, best_opening)

        # Convert scan to obstacles for path planning
        obstacles = self._convert_scan_to_obstacles(scan_array, y_array)
        
        return obstacles, steer_angle

    def _check_critical_obstacles(self, scan: np.ndarray) -> bool:
        """Check for obstacles that are critically close."""
        # Get the central region of the scan (middle third)
        center_start = len(scan) // 3
        center_end = 2 * len(scan) // 3
        center_scan = scan[center_start:center_end]
        
        # Check if any obstacles are within critical distance
        return np.any(center_scan < self.CRITICAL_DISTANCE)

    def _find_openings(self, scan: np.ndarray) -> List[List[int]]:
        """Find all valid openings in the scan data."""
        openings = []
        current_opening = []
        
        # Use numpy for efficient threshold comparison
        scan_copy = np.minimum(scan, self.MAX_THRESHOLD)
        is_open = scan_copy >= (self.CRITICAL_DISTANCE + self.SAFETY_MARGIN)
        
        for i, open_point in enumerate(is_open):
            if open_point:
                current_opening.append(i)
            elif current_opening:
                if len(current_opening) >= self.MIN_OPENING_WIDTH:
                    openings.append(current_opening)
                current_opening = []
                
        # Don't forget the last opening
        if current_opening and len(current_opening) >= self.MIN_OPENING_WIDTH:
            openings.append(current_opening)
            
        return openings

    def _select_best_opening(self, openings: List[List[int]], scan: np.ndarray) -> List[int]:
        """Select the best opening based on width and position."""
        if not openings:
            return []
            
        mid_scan = len(scan) // 2
        
        # Score each opening based on width and distance from center
        def score_opening(opening):
            width = len(opening)
            center_idx = opening[len(opening)//2]
            distance_from_center = abs(center_idx - mid_scan)
            
            # Prefer wider openings that are closer to center
            return width - (distance_from_center * 0.5)
            
        return max(openings, key=score_opening)

    def _calculate_steering(self, scan: np.ndarray, y_array: np.ndarray, opening: List[int]) -> float:
        """Calculate smooth steering angle based on opening position."""
        if not opening:
            return 0.0
            
        mid_scan = len(scan) // 2
        opening_center = sum(opening) / len(opening)
        
        # Calculate base steering angle
        relative_pos = (opening_center - mid_scan) / mid_scan
        base_angle = relative_pos * 45.0 * self.STEERING_SENSITIVITY
        
        # Apply obstacle-based adjustment
        nearby_obstacles = scan[max(0, int(opening_center)-10):min(len(scan), int(opening_center)+10)]
        if len(nearby_obstacles) > 0:
            min_distance = np.min(nearby_obstacles)
            distance_factor = np.clip(min_distance / self.MAX_THRESHOLD, 0.3, 1.0)
            base_angle *= distance_factor
        
        # Smooth the steering angle
        smoothed_angle = (self.steering_smoothing * self.last_steering_angle + 
                         (1 - self.steering_smoothing) * base_angle)
        self.last_steering_angle = smoothed_angle
        
        return smoothed_angle

    def _convert_scan_to_obstacles(self, scan_array: np.ndarray, y_array: np.ndarray) -> List[Obstacle]:
        """Convert scan data to 3D obstacles with improved clustering."""
        obstacles = []
        points = np.column_stack((scan_array, y_array))
        processed = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if processed[i] or scan_array[i] >= self.MAX_THRESHOLD:
                continue
                
            # Find nearby points
            cluster_points = []
            self._grow_cluster(points, i, processed, cluster_points)
            
            if len(cluster_points) >= self.min_points:
                cluster_points = np.array(cluster_points)
                center = np.array([
                    np.mean(cluster_points[:, 0]),  # x
                    np.mean(cluster_points[:, 1]),  # y
                    0.5  # z (assumed height)
                ])
                
                # Calculate radius as maximum distance from center to any point
                radius = np.max(np.linalg.norm(cluster_points - center[:2], axis=1))
                
                # Calculate confidence based on number of points and consistency
                point_count_confidence = min(1.0, len(cluster_points) / (2 * self.min_points))
                distance_std = np.std(np.linalg.norm(cluster_points - center[:2], axis=1))
                consistency_confidence = 1.0 / (1.0 + distance_std)
                confidence = (point_count_confidence + consistency_confidence) / 2
                
                obstacles.append(Obstacle(center, radius, confidence))
        
        return obstacles

    def _grow_cluster(self, points: np.ndarray, start_idx: int, processed: np.ndarray, cluster_points: List):
        """Helper method for growing point clusters."""
        stack = [start_idx]
        while stack:
            idx = stack.pop()
            if processed[idx]:
                continue
                
            processed[idx] = True
            cluster_points.append(points[idx])
            
            # Check neighbors
            distances = np.linalg.norm(points - points[idx], axis=1)
            neighbors = np.where((distances < self.cluster_threshold) & ~processed)[0]
            stack.extend(neighbors)