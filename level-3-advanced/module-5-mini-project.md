# Module 5: Mini Project — Autonomous Navigation

## 🎯 Project Overview

In this capstone project for Level 3, you will build a complete **autonomous navigation system** using RealSense depth cameras. This project integrates all the concepts from the previous modules: Visual SLAM, sensor fusion, AI perception, and cloud robotics.

### Project Goals

By completing this project, you will:
- Build a working autonomous navigation system
- Integrate SLAM for real-time mapping and localization
- Implement obstacle avoidance using depth data
- Create a path planning and following system
- Deploy and test on a real or simulated robot

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Autonomous Navigation System               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │   RealSense   │───▶│    SLAM      │───▶│     Map      │  │
│  │    Camera     │    │   System     │    │   Manager    │  │
│  └──────────────┘    └──────────────┘    └──────────────┘  │
│         │                   │                    │          │
│         ▼                   ▼                    ▼          │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │   Obstacle    │    │    Pose      │    │     Path     │  │
│  │   Detection   │    │  Estimation  │    │   Planning   │  │
│  └──────────────┘    └──────────────┘    └──────────────┘  │
│         │                   │                    │          │
│         └───────────────────┼────────────────────┘          │
│                             ▼                               │
│                    ┌──────────────┐                         │
│                    │   Motion     │                         │
│                    │  Controller  │                         │
│                    └──────────────┘                         │
│                             │                               │
│                             ▼                               │
│                    ┌──────────────┐                         │
│                    │    Robot     │                         │
│                    │   Platform   │                         │
│                    └──────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

## 🛠️ Prerequisites

### Hardware Requirements
- **RealSense Camera**: D435i, D455, or D457 (with IMU)
- **Robot Platform**: TurtleBot, custom robot, or simulation
- **Computer**: With ROS2 and GPU (recommended)

### Software Requirements
```bash
# ROS2 packages
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-robot-localization

# Python packages
pip install numpy opencv-python open3d scipy
pip install transforms3d
```

## 📋 Project Structure

```
autonomous_navigation/
├── launch/
│   ├── navigation.launch.py
│   ├── slam.launch.py
│   └── full_system.launch.py
├── config/
│   ├── nav2_params.yaml
│   ├── slam_params.yaml
│   └── robot_params.yaml
├── src/
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── depth_processor.py
│   │   ├── obstacle_detector.py
│   │   └── costmap_generator.py
│   ├── localization/
│   │   ├── __init__.py
│   │   ├── visual_odometry.py
│   │   └── pose_estimator.py
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── path_planner.py
│   │   └── trajectory_tracker.py
│   └── control/
│       ├── __init__.py
│       └── motion_controller.py
├── scripts/
│   ├── navigation_node.py
│   └── test_navigation.py
└── README.md
```

## 🔧 Implementation

### Step 1: Perception Module

#### Depth Processor

```python
# src/perception/depth_processor.py
import pyrealsense2 as rs
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Tuple, Optional

@dataclass
class DepthConfig:
    min_distance: float = 0.3
    max_distance: float = 5.0
    decimation_factor: int = 2
    spatial_filter_alpha: float = 0.5
    temporal_filter_alpha: float = 0.4

class DepthProcessor:
    def __init__(self, config: DepthConfig = None):
        self.config = config or DepthConfig()
        
        # Filters
        self.decimation = rs.decimation_filter()
        self.decimation.set_option(rs.option.filter_magnitude, self.config.decimation_factor)
        
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_smooth_alpha, self.config.spatial_filter_alpha)
        
        self.temporal = rs.temporal_filter()
        self.temporal.set_option(rs.option.filter_smooth_alpha, self.config.temporal_filter_alpha)
        
        self.hole_filling = rs.hole_filling_filter()
        
        self.threshold = rs.threshold_filter()
        self.threshold.set_option(rs.option.min_distance, self.config.min_distance)
        self.threshold.set_option(rs.option.max_distance, self.config.max_distance)
        
    def process(self, depth_frame) -> np.ndarray:
        """Process depth frame with filtering pipeline"""
        filtered = depth_frame
        filtered = self.decimation.process(filtered)
        filtered = self.threshold.process(filtered)
        filtered = self.spatial.process(filtered)
        filtered = self.temporal.process(filtered)
        filtered = self.hole_filling.process(filtered)
        
        return np.asanyarray(filtered.get_data())
    
    def compute_ground_plane(self, depth_image: np.ndarray, 
                            camera_height: float,
                            camera_pitch: float) -> Tuple[np.ndarray, float]:
        """Estimate ground plane from depth data"""
        height, width = depth_image.shape
        
        # Sample points from lower portion of image
        sample_region = depth_image[int(height*0.7):, :]
        valid_mask = (sample_region > self.config.min_distance * 1000) & \
                     (sample_region < self.config.max_distance * 1000)
        
        if np.sum(valid_mask) < 100:
            return np.array([0, 1, 0]), camera_height
        
        # Create 3D points
        fx, fy = 525, 525  # Approximate intrinsics
        cx, cy = width // 2, height // 2
        
        points = []
        y_offset = int(height * 0.7)
        
        for v in range(sample_region.shape[0]):
            for u in range(sample_region.shape[1]):
                if valid_mask[v, u]:
                    z = sample_region[v, u] / 1000.0
                    x = (u - cx) * z / fx
                    y = (v + y_offset - cy) * z / fy
                    points.append([x, y, z])
        
        points = np.array(points)
        
        # Fit plane using RANSAC
        from sklearn.linear_model import RANSACRegressor
        
        X = points[:, [0, 2]]  # x, z
        y = points[:, 1]       # y
        
        ransac = RANSACRegressor()
        ransac.fit(X, y)
        
        # Extract plane normal
        normal = np.array([-ransac.estimator_.coef_[0], 1, -ransac.estimator_.coef_[1]])
        normal = normal / np.linalg.norm(normal)
        
        ground_height = ransac.estimator_.intercept_
        
        return normal, ground_height
```

#### Obstacle Detector

```python
# src/perception/obstacle_detector.py
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Obstacle:
    position: np.ndarray  # [x, y, z] in robot frame
    size: np.ndarray      # [width, height, depth]
    confidence: float
    is_dynamic: bool = False

class ObstacleDetector:
    def __init__(self, robot_radius: float = 0.3,
                 safety_margin: float = 0.2,
                 min_obstacle_size: float = 0.05):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.min_obstacle_size = min_obstacle_size
        
        # Tracking for dynamic obstacles
        self.previous_obstacles = []
        self.obstacle_velocities = {}
        
    def detect_obstacles(self, depth_image: np.ndarray,
                        ground_plane: Tuple[np.ndarray, float],
                        intrinsics: dict) -> List[Obstacle]:
        """Detect obstacles from depth image"""
        height, width = depth_image.shape
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']
        
        ground_normal, ground_height = ground_plane
        
        # Convert depth to 3D points
        points = []
        for v in range(0, height, 2):
            for u in range(0, width, 2):
                z = depth_image[v, u] / 1000.0
                if 0.1 < z < 5.0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        
        points = np.array(points)
        
        if len(points) < 10:
            return []
        
        # Filter out ground points
        ground_threshold = 0.1  # 10cm above ground
        obstacle_points = []
        
        for point in points:
            height_above_ground = abs(np.dot(point, ground_normal) - ground_height)
            if height_above_ground > ground_threshold:
                obstacle_points.append(point)
        
        obstacle_points = np.array(obstacle_points)
        
        if len(obstacle_points) < 10:
            return []
        
        # Cluster obstacles
        obstacles = self._cluster_obstacles(obstacle_points)
        
        # Track for dynamic detection
        obstacles = self._track_obstacles(obstacles)
        
        return obstacles
    
    def _cluster_obstacles(self, points: np.ndarray) -> List[Obstacle]:
        """Cluster points into obstacles using DBSCAN"""
        from sklearn.cluster import DBSCAN
        
        clustering = DBSCAN(eps=0.1, min_samples=5).fit(points)
        labels = clustering.labels_
        
        obstacles = []
        unique_labels = set(labels)
        
        for label in unique_labels:
            if label == -1:  # Noise
                continue
            
            cluster_points = points[labels == label]
            
            # Calculate obstacle properties
            center = np.mean(cluster_points, axis=0)
            min_bounds = np.min(cluster_points, axis=0)
            max_bounds = np.max(cluster_points, axis=0)
            size = max_bounds - min_bounds
            
            if np.all(size > self.min_obstacle_size):
                obstacle = Obstacle(
                    position=center,
                    size=size,
                    confidence=min(len(cluster_points) / 100, 1.0)
                )
                obstacles.append(obstacle)
        
        return obstacles
    
    def _track_obstacles(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        """Track obstacles across frames for dynamic detection"""
        if not self.previous_obstacles:
            self.previous_obstacles = obstacles
            return obstacles
        
        # Match current obstacles with previous
        for obs in obstacles:
            best_match = None
            best_distance = float('inf')
            
            for prev_obs in self.previous_obstacles:
                distance = np.linalg.norm(obs.position - prev_obs.position)
                if distance < best_distance and distance < 0.5:
                    best_distance = distance
                    best_match = prev_obs
            
            if best_match is not None:
                # Calculate velocity
                velocity = obs.position - best_match.position
                obs_id = id(obs)
                
                if np.linalg.norm(velocity) > 0.1:  # Moving > 10cm/frame
                    obs.is_dynamic = True
                    self.obstacle_velocities[obs_id] = velocity
        
        self.previous_obstacles = obstacles
        return obstacles
    
    def get_danger_zones(self, obstacles: List[Obstacle]) -> np.ndarray:
        """Get 2D danger zones for costmap"""
        zones = []
        
        for obs in obstacles:
            # Project to 2D (x, z plane)
            center_2d = np.array([obs.position[0], obs.position[2]])
            radius = max(obs.size[0], obs.size[2]) / 2 + self.safety_margin
            
            if obs.is_dynamic:
                radius *= 1.5  # Larger buffer for dynamic obstacles
            
            zones.append({
                'center': center_2d,
                'radius': radius,
                'is_dynamic': obs.is_dynamic
            })
        
        return zones
```

### Step 2: Localization Module

#### Visual Odometry

```python
# src/localization/visual_odometry.py
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class OdometryResult:
    translation: np.ndarray
    rotation: np.ndarray
    confidence: float
    num_inliers: int

class VisualOdometry:
    def __init__(self):
        # Feature detector
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Previous frame data
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_depth = None
        
        # Camera intrinsics
        self.K = None
        
    def set_intrinsics(self, fx, fy, cx, cy):
        """Set camera intrinsics"""
        self.K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
    
    def process_frame(self, rgb_image: np.ndarray, 
                      depth_image: np.ndarray) -> Optional[OdometryResult]:
        """Process new frame and compute odometry"""
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        
        # Detect features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        if self.prev_keypoints is None:
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_depth = depth_image
            return None
        
        if descriptors is None or self.prev_descriptors is None:
            return None
        
        # Match features
        matches = self.bf_matcher.match(self.prev_descriptors, descriptors)
        matches = sorted(matches, key=lambda x: x.distance)[:100]
        
        if len(matches) < 10:
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_depth = depth_image
            return None
        
        # Get 3D-2D correspondences
        pts_3d = []
        pts_2d = []
        
        for match in matches:
            pt1 = self.prev_keypoints[match.queryIdx].pt
            pt2 = keypoints[match.trainIdx].pt
            
            u1, v1 = int(pt1[0]), int(pt1[1])
            
            if 0 <= u1 < self.prev_depth.shape[1] and 0 <= v1 < self.prev_depth.shape[0]:
                depth = self.prev_depth[v1, u1] / 1000.0
                
                if 0.1 < depth < 10.0:
                    # Back-project to 3D
                    x = (u1 - self.K[0, 2]) * depth / self.K[0, 0]
                    y = (v1 - self.K[1, 2]) * depth / self.K[1, 1]
                    z = depth
                    
                    pts_3d.append([x, y, z])
                    pts_2d.append(pt2)
        
        if len(pts_3d) < 6:
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_depth = depth_image
            return None
        
        pts_3d = np.array(pts_3d, dtype=np.float32)
        pts_2d = np.array(pts_2d, dtype=np.float32)
        
        # Solve PnP with RANSAC
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts_3d, pts_2d, self.K, None,
            iterationsCount=100,
            reprojectionError=8.0
        )
        
        if not success:
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_depth = depth_image
            return None
        
        # Convert to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        
        # Store for next frame
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_depth = depth_image
        
        return OdometryResult(
            translation=tvec.flatten(),
            rotation=R,
            confidence=len(inliers) / len(matches),
            num_inliers=len(inliers)
        )
```

### Step 3: Path Planning

#### A* Path Planner

```python
# src/planning/path_planner.py
import numpy as np
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class Node:
    position: Tuple[int, int]
    g_cost: float  # Cost from start
    h_cost: float  # Heuristic cost to goal
    parent: Optional['Node'] = None
    
    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class PathPlanner:
    def __init__(self, resolution: float = 0.05):
        self.resolution = resolution
        self.costmap = None
        self.costmap_origin = None
        
    def update_costmap(self, obstacles: List[dict], 
                       map_size: Tuple[float, float],
                       robot_position: np.ndarray):
        """Update costmap from obstacle data"""
        width = int(map_size[0] / self.resolution)
        height = int(map_size[1] / self.resolution)
        
        self.costmap = np.zeros((height, width), dtype=np.float32)
        self.costmap_origin = robot_position - np.array([map_size[0]/2, map_size[1]/2])
        
        for obs in obstacles:
            center = obs['center']
            radius = obs['radius']
            
            # Convert to grid coordinates
            cx = int((center[0] - self.costmap_origin[0]) / self.resolution)
            cy = int((center[1] - self.costmap_origin[1]) / self.resolution)
            r = int(radius / self.resolution)
            
            # Draw obstacle with gradient
            for y in range(max(0, cy-r-5), min(height, cy+r+5)):
                for x in range(max(0, cx-r-5), min(width, cx+r+5)):
                    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
                    if dist < r:
                        self.costmap[y, x] = 1.0  # Obstacle
                    elif dist < r + 5:
                        # Inflation
                        self.costmap[y, x] = max(self.costmap[y, x], 
                                                  1.0 - (dist - r) / 5)
    
    def world_to_grid(self, position: np.ndarray) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((position[0] - self.costmap_origin[0]) / self.resolution)
        grid_y = int((position[1] - self.costmap_origin[1]) / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_pos: Tuple[int, int]) -> np.ndarray:
        """Convert grid coordinates to world coordinates"""
        world_x = grid_pos[0] * self.resolution + self.costmap_origin[0]
        world_y = grid_pos[1] * self.resolution + self.costmap_origin[1]
        return np.array([world_x, world_y])
    
    def plan_path(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """Plan path using A* algorithm"""
        if self.costmap is None:
            return None
        
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        # Check bounds
        height, width = self.costmap.shape
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
            return None
        if not (0 <= goal_grid[0] < width and 0 <= goal_grid[1] < height):
            return None
        
        # Check if goal is in obstacle
        if self.costmap[goal_grid[1], goal_grid[0]] > 0.5:
            return None
        
        # A* algorithm
        open_list = []
        closed_set = set()
        
        start_node = Node(
            position=start_grid,
            g_cost=0,
            h_cost=self._heuristic(start_grid, goal_grid)
        )
        heapq.heappush(open_list, start_node)
        
        nodes = {start_grid: start_node}
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if current.position == goal_grid:
                # Reconstruct path
                path = []
                node = current
                while node is not None:
                    path.append(self.grid_to_world(node.position))
                    node = node.parent
                return path[::-1]
            
            closed_set.add(current.position)
            
            # Explore neighbors
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), 
                          (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor_pos = (current.position[0] + dx, current.position[1] + dy)
                
                # Check bounds
                if not (0 <= neighbor_pos[0] < width and 0 <= neighbor_pos[1] < height):
                    continue
                
                # Check if already visited
                if neighbor_pos in closed_set:
                    continue
                
                # Check if obstacle
                cost = self.costmap[neighbor_pos[1], neighbor_pos[0]]
                if cost > 0.8:
                    continue
                
                # Calculate cost
                move_cost = np.sqrt(dx**2 + dy**2) * (1 + cost)
                g_cost = current.g_cost + move_cost
                
                # Check if better path
                if neighbor_pos in nodes:
                    if g_cost >= nodes[neighbor_pos].g_cost:
                        continue
                
                # Create/update node
                neighbor_node = Node(
                    position=neighbor_pos,
                    g_cost=g_cost,
                    h_cost=self._heuristic(neighbor_pos, goal_grid),
                    parent=current
                )
                
                nodes[neighbor_pos] = neighbor_node
                heapq.heappush(open_list, neighbor_node)
        
        return None
    
    def _heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def smooth_path(self, path: List[np.ndarray], weight_smooth: float = 0.5) -> List[np.ndarray]:
        """Smooth path using gradient descent"""
        if len(path) < 3:
            return path
        
        path = np.array(path)
        smoothed = path.copy()
        
        for _ in range(100):
            for i in range(1, len(path) - 1):
                smoothed[i] += weight_smooth * (
                    path[i] - smoothed[i] +
                    (smoothed[i-1] + smoothed[i+1] - 2*smoothed[i])
                )
        
        return [p for p in smoothed]
```

### Step 4: Motion Controller

```python
# src/control/motion_controller.py
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, List

@dataclass
class ControlOutput:
    linear_velocity: float
    angular_velocity: float
    is_emergency_stop: bool = False

class PurePursuitController:
    def __init__(self, lookahead_distance: float = 0.5,
                 max_linear_velocity: float = 0.5,
                 max_angular_velocity: float = 1.0,
                 goal_tolerance: float = 0.1):
        self.lookahead_distance = lookahead_distance
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.goal_tolerance = goal_tolerance
        
        self.path = None
        self.current_waypoint_index = 0
        
    def set_path(self, path: List[np.ndarray]):
        """Set path to follow"""
        self.path = path
        self.current_waypoint_index = 0
    
    def compute_control(self, robot_position: np.ndarray,
                        robot_heading: float,
                        obstacles: List[dict]) -> ControlOutput:
        """Compute control commands using Pure Pursuit"""
        if self.path is None or len(self.path) == 0:
            return ControlOutput(0, 0)
        
        # Check for emergency stop
        if self._check_emergency(robot_position, robot_heading, obstacles):
            return ControlOutput(0, 0, is_emergency_stop=True)
        
        # Find lookahead point
        lookahead_point = self._find_lookahead_point(robot_position)
        
        if lookahead_point is None:
            return ControlOutput(0, 0)
        
        # Check if at goal
        if self._at_goal(robot_position):
            return ControlOutput(0, 0)
        
        # Compute control
        dx = lookahead_point[0] - robot_position[0]
        dy = lookahead_point[1] - robot_position[1]
        
        # Angle to lookahead point
        angle_to_point = np.arctan2(dy, dx)
        angle_error = self._normalize_angle(angle_to_point - robot_heading)
        
        # Pure pursuit curvature
        distance = np.sqrt(dx**2 + dy**2)
        curvature = 2 * np.sin(angle_error) / distance
        
        # Compute velocities
        linear_velocity = self.max_linear_velocity
        angular_velocity = curvature * linear_velocity
        
        # Slow down for sharp turns
        if abs(angle_error) > np.pi / 4:
            linear_velocity *= 0.5
        
        # Clip velocities
        linear_velocity = np.clip(linear_velocity, 0, self.max_linear_velocity)
        angular_velocity = np.clip(angular_velocity, 
                                   -self.max_angular_velocity, 
                                   self.max_angular_velocity)
        
        return ControlOutput(linear_velocity, angular_velocity)
    
    def _find_lookahead_point(self, robot_position: np.ndarray) -> Optional[np.ndarray]:
        """Find lookahead point on path"""
        if self.current_waypoint_index >= len(self.path):
            return None
        
        # Find closest point on path
        min_dist = float('inf')
        closest_index = self.current_waypoint_index
        
        for i in range(self.current_waypoint_index, len(self.path)):
            dist = np.linalg.norm(self.path[i] - robot_position)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        
        # Find lookahead point
        for i in range(closest_index, len(self.path)):
            dist = np.linalg.norm(self.path[i] - robot_position)
            if dist >= self.lookahead_distance:
                self.current_waypoint_index = closest_index
                return self.path[i]
        
        # Return last point if no lookahead found
        return self.path[-1]
    
    def _at_goal(self, robot_position: np.ndarray) -> bool:
        """Check if robot is at goal"""
        if self.path is None or len(self.path) == 0:
            return True
        
        goal = self.path[-1]
        return np.linalg.norm(robot_position - goal) < self.goal_tolerance
    
    def _check_emergency(self, robot_position: np.ndarray,
                        robot_heading: float,
                        obstacles: List[dict]) -> bool:
        """Check for emergency stop conditions"""
        for obs in obstacles:
            # Check if obstacle is in front
            obs_vec = obs['center'] - robot_position
            distance = np.linalg.norm(obs_vec)
            angle = np.arctan2(obs_vec[1], obs_vec[0])
            angle_diff = abs(self._normalize_angle(angle - robot_heading))
            
            # Emergency if obstacle is close and in front
            if distance < obs['radius'] + 0.3 and angle_diff < np.pi / 3:
                return True
        
        return False
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
```

### Step 5: Main Navigation Node

```python
#!/usr/bin/env python3
# scripts/navigation_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
import transforms3d

# Import our modules
from perception.depth_processor import DepthProcessor
from perception.obstacle_detector import ObstacleDetector
from localization.visual_odometry import VisualOdometry
from planning.path_planner import PathPlanner
from control.motion_controller import PurePursuitController

class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('map_size', [10.0, 10.0])
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.map_size = self.get_parameter('map_size').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'local_costmap', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize RealSense
        self.setup_realsense()
        
        # Initialize modules
        self.depth_processor = DepthProcessor()
        self.obstacle_detector = ObstacleDetector(robot_radius=self.robot_radius)
        self.visual_odom = VisualOdometry()
        self.path_planner = PathPlanner()
        self.motion_controller = PurePursuitController(
            max_linear_velocity=self.max_velocity
        )
        
        # State
        self.robot_position = np.array([0.0, 0.0])
        self.robot_heading = 0.0
        self.goal_position = None
        self.current_path = None
        
        # Set intrinsics
        depth_stream = self.profile.get_stream(rs.stream.depth)
        intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        self.visual_odom.set_intrinsics(
            intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
        )
        self.intrinsics = {
            'fx': intrinsics.fx, 'fy': intrinsics.fy,
            'cx': intrinsics.ppx, 'cy': intrinsics.ppy
        }
        
        # Main loop timer
        self.timer = self.create_timer(0.033, self.navigation_loop)
        
        self.get_logger().info('Autonomous Navigation Node started')
    
    def setup_realsense(self):
        """Initialize RealSense camera"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
    
    def goal_callback(self, msg):
        """Handle new goal"""
        self.goal_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.get_logger().info(f'New goal: {self.goal_position}')
        
        # Plan path
        self.plan_path()
    
    def plan_path(self):
        """Plan path to goal"""
        if self.goal_position is None:
            return
        
        path = self.path_planner.plan_path(
            self.robot_position,
            self.goal_position
        )
        
        if path is not None:
            path = self.path_planner.smooth_path(path)
            self.current_path = path
            self.motion_controller.set_path(path)
            
            # Publish path
            self.publish_path(path)
            
            self.get_logger().info(f'Path planned with {len(path)} waypoints')
        else:
            self.get_logger().warn('Failed to plan path')
    
    def navigation_loop(self):
        """Main navigation loop"""
        # Get frames
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return
        
        # Process depth
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        processed_depth = self.depth_processor.process(depth_frame)
        
        # Update localization
        odom_result = self.visual_odom.process_frame(color_image, depth_image)
        if odom_result is not None:
            self.update_pose(odom_result)
        
        # Detect obstacles
        ground_plane = self.depth_processor.compute_ground_plane(
            processed_depth, camera_height=0.5, camera_pitch=0.0
        )
        obstacles = self.obstacle_detector.detect_obstacles(
            processed_depth, ground_plane, self.intrinsics
        )
        
        # Update costmap
        danger_zones = self.obstacle_detector.get_danger_zones(obstacles)
        self.path_planner.update_costmap(
            danger_zones, self.map_size, self.robot_position
        )
        
        # Replan if needed
        if self.current_path is not None and len(danger_zones) > 0:
            self.plan_path()
        
        # Compute control
        control = self.motion_controller.compute_control(
            self.robot_position,
            self.robot_heading,
            danger_zones
        )
        
        # Publish command
        twist = Twist()
        twist.linear.x = control.linear_velocity
        twist.angular.z = control.angular_velocity
        
        if control.is_emergency_stop:
            self.get_logger().warn('Emergency stop!')
        
        self.cmd_vel_pub.publish(twist)
        
        # Visualize
        self.visualize(color_image, processed_depth, obstacles, danger_zones)
    
    def update_pose(self, odom_result):
        """Update robot pose from odometry"""
        # Integrate odometry
        rotation_angles = transforms3d.euler.mat2euler(odom_result.rotation)
        yaw = rotation_angles[2]
        
        self.robot_heading += yaw
        
        dx = odom_result.translation[0]
        dz = odom_result.translation[2]
        
        self.robot_position[0] += dx * np.cos(self.robot_heading) - dz * np.sin(self.robot_heading)
        self.robot_position[1] += dx * np.sin(self.robot_heading) + dz * np.cos(self.robot_heading)
    
    def publish_path(self, path):
        """Publish planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def visualize(self, color_image, depth_image, obstacles, danger_zones):
        """Visualize navigation state"""
        vis_image = color_image.copy()
        
        # Draw obstacles
        for obs in obstacles:
            center = (int(320 + obs.position[0] * 100), 
                      int(240 - obs.position[2] * 100))
            radius = int(max(obs.size) * 100)
            color = (0, 0, 255) if obs.is_dynamic else (255, 0, 0)
            cv2.circle(vis_image, center, radius, color, 2)
        
        # Draw path
        if self.current_path is not None:
            for i in range(len(self.current_path) - 1):
                pt1 = (int(320 + self.current_path[i][0] * 100),
                       int(240 - self.current_path[i][1] * 100))
                pt2 = (int(320 + self.current_path[i+1][0] * 100),
                       int(240 - self.current_path[i+1][1] * 100))
                cv2.line(vis_image, pt1, pt2, (0, 255, 0), 2)
        
        # Display
        cv2.imshow('Navigation', vis_image)
        cv2.waitKey(1)
    
    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🧪 Testing & Evaluation

### Test Scenarios

1. **Static Environment**: Navigate through fixed obstacles
2. **Dynamic Obstacles**: Avoid moving obstacles
3. **Narrow Passages**: Navigate through tight spaces
4. **Long Distance**: Navigate across large areas
5. **Recovery**: Handle sensor failures and replanning

### Evaluation Metrics

| Metric | Target | How to Measure |
|--------|--------|----------------|
| **Success Rate** | > 90% | Complete navigation tasks |
| **Path Efficiency** | < 1.2x optimal | Path length vs straight line |
| **Collision Rate** | 0% | Collisions with obstacles |
| **Average Velocity** | > 0.3 m/s | Distance / time |
| **Replanning Rate** | < 5 per run | Number of replans |

## 🎯 Project Deliverables

### Required Components

1. **Working Navigation System**
   - SLAM integration
   - Obstacle detection
   - Path planning
   - Motion control

2. **Documentation**
   - System architecture
   - API documentation
   - Usage instructions

3. **Demo Video**
   - 3-5 minute demonstration
   - Multiple test scenarios
   - Performance metrics

### Submission Checklist

- [ ] Complete source code
- [ ] ROS2 launch files
- [ ] Configuration files
- [ ] README with instructions
- [ ] Demo video
- [ ] Performance report

## 🎉 Completion

Congratulations on completing Level 3! You've built a complete autonomous navigation system using RealSense depth cameras.

**Ready for the next challenge?** → [Level 4: Expert Master Classes](../level-4-expert/)
