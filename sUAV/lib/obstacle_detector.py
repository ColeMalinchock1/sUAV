from dataclasses import dataclass
import numpy as np
from typing import List, Tuple
import math

@dataclass
class Obstacle:
    center: np.ndarray  # [x, y, z]
    radius: float
    confidence: float

class EnhancedObstacleDetector:
    def __init__(self, min_points: int = 10, cluster_threshold: float = 0.5,
                 vehicle_size: float = 5.0, max_threshold: float = 1.0):
        self.min_points = min_points
        self.cluster_threshold = cluster_threshold
        self.VEHICLE_SIZE = vehicle_size
        self.MAX_THRESHOLD = max_threshold
        self.last_scan = None
        self.last_y_distances = None

    def process_scan_data(self, scan_array: np.ndarray, y_array: np.ndarray) -> Tuple[List[Obstacle], float]:
        """Process 2D scan data and return both obstacles and steering angle"""
        self.last_scan = scan_array
        self.last_y_distances = y_array
        
        # First check for immediate obstacles
        opening = self._find_opening(scan_array)
        if opening == -1 or len(opening) < self.VEHICLE_SIZE:
            return [], "STOP"
            
        steer_angle = self._steer_adjuster(scan_array, opening, y_array)
        
        # Convert 2D scan to 3D obstacles for path planning
        obstacles = self._convert_scan_to_obstacles(scan_array, y_array)
        
        return obstacles, steer_angle

    def _find_opening(self, scan):
        """Original find_opening logic from Rerouter"""
        longest_streak = None
        new_streak = [0, 0]
        start_new_streak = True
        
        scan_copy = scan.copy()
        
        for i in range(len(scan_copy)):
            if scan_copy[i] >= self.MAX_THRESHOLD:
                scan_copy[i] = self.MAX_THRESHOLD
                if start_new_streak:
                    start_new_streak = False
                    new_streak = [i, i]
                else:
                    new_streak[1] = i
            else:
                start_new_streak = True
                if longest_streak is not None:
                    if longest_streak[1] - longest_streak[0] < new_streak[1] - new_streak[0]:
                        longest_streak = new_streak
                else:
                    longest_streak = new_streak

        if longest_streak is not None:
            return list(range(longest_streak[0], longest_streak[1] + 1))
        return -1

    def _steer_adjuster(self, scan, opening, y_distances):
        """Original steer_adjuster logic from Rerouter"""
        mid_scan = int(len(scan) / 2)
        vehicle_half = int(self.VEHICLE_SIZE / 2)
        
        if mid_scan - vehicle_half in opening or mid_scan + vehicle_half in opening:
            return 0
            
        k = -1 if opening[int(len(opening) / 2)] < mid_scan else 1
        
        closest_point = None
        if k < 0:
            for i in range(mid_scan, len(scan)):
                if closest_point is None or scan[closest_point] > scan[i]:
                    closest_point = i
        else:
            for i in range(mid_scan):
                if closest_point is None or scan[closest_point] > scan[i]:
                    closest_point = i

        return k * math.degrees(math.atan(y_distances[closest_point]) / self.MAX_THRESHOLD - scan[closest_point])

    def _convert_scan_to_obstacles(self, scan_array: np.ndarray, y_array: np.ndarray) -> List[Obstacle]:
        """Convert 2D scan data to 3D obstacles"""
        obstacles = []
        for i, (depth, y_dist) in enumerate(zip(scan_array, y_array)):
            if depth < self.MAX_THRESHOLD:
                center = np.array([depth, y_dist, 0.5])  # Assume 0.5m height
                radius = 0.3  # Assume 30cm radius for detected points
                confidence = 1.0  # High confidence for direct detections
                obstacles.append(Obstacle(center, radius, confidence))
        return obstacles