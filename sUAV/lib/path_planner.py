#!/usr/bin/env python3
from dataclasses import dataclass
import numpy as np
from typing import List
from .obstacle_detector import Obstacle

@dataclass
class PathSegment:
    start: np.ndarray
    end: np.ndarray
    is_original: bool = True

class PathPlanner:
    def __init__(self, safety_margin: float = 1.0):
        self.safety_margin = safety_margin
        
    def find_safe_path(self, 
                      current_pos: np.ndarray,
                      goal_pos: np.ndarray,
                      obstacles: List[Obstacle]) -> List[PathSegment]:
        """Generate safe path segments avoiding obstacles"""
        if not obstacles:
            return [PathSegment(current_pos, goal_pos)]
            
        # Start with direct path
        path_segments = [PathSegment(current_pos, goal_pos)]
        
        for obstacle in obstacles:
            new_segments = []
            for segment in path_segments:
                # Check if path intersects with obstacle + safety margin
                if self._check_intersection(segment, obstacle):
                    # Generate avoidance waypoints
                    detour = self._generate_detour(segment, obstacle)
                    new_segments.extend(detour)
                else:
                    new_segments.append(segment)
            path_segments = new_segments
            
        return path_segments
        
    def _check_intersection(self, segment: PathSegment, obstacle: Obstacle) -> bool:
        """Check if path segment intersects with obstacle considering safety margin"""
        start = segment.start
        end = segment.end
        
        # Vector from start to end
        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length == 0:
            return False
            
        # Normalized path vector
        path_direction = path_vector / path_length
        
        # Vector from start to obstacle center
        to_obstacle = obstacle.center - start
        
        # Project obstacle onto path
        projection_length = np.dot(to_obstacle, path_direction)
        
        # If projection is outside segment, no intersection
        if projection_length < 0 or projection_length > path_length:
            return False
            
        # Find closest point on path to obstacle center
        closest_point = start + path_direction * projection_length
        
        # Check if distance is less than obstacle radius + safety margin
        distance = np.linalg.norm(obstacle.center - closest_point)
        return distance < (obstacle.radius + self.safety_margin)
        
    def _generate_detour(self, segment: PathSegment, obstacle: Obstacle) -> List[PathSegment]:
        """Generate waypoints to avoid obstacle"""
        # Calculate avoidance direction perpendicular to path
        path_vector = segment.end - segment.start
        path_length = np.linalg.norm(path_vector)
        path_direction = path_vector / path_length
        
        # Calculate perpendicular vector
        perp_vector = np.array([-path_direction[1], path_direction[0], 0])
        
        # Determine which side to avoid based on current position
        to_obstacle = obstacle.center - segment.start
        side = np.sign(np.dot(to_obstacle, perp_vector))
        
        # Calculate avoidance distance
        avoidance_distance = obstacle.radius + self.safety_margin * 1.5
        
        # Generate waypoints
        midpoint = (segment.start + segment.end) / 2
        avoidance_point = midpoint + side * perp_vector * avoidance_distance
        
        return [
            PathSegment(segment.start, avoidance_point, False),
            PathSegment(avoidance_point, segment.end, False)
        ]