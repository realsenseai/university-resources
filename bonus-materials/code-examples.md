# 💻 Code Examples & Templates

## 🎯 Overview

This page provides ready-to-use code examples, project templates, and starter kits for all levels of RealSense University. From simple camera tests to complex SLAM systems, you'll find working code to accelerate your development.

## 📁 Repository Structure

Our code examples are organized by skill level:

```
code-examples/
├── level-1-beginner/          # Basic camera operations
│   ├── basic_camera_test.py   # Camera connection test
│   ├── depth_visualization.py # Depth colormap display
│   ├── point_cloud_generator.py # 3D point cloud creation
│   └── distance_measurement.py  # Distance measurement tool
├── level-2-intermediate/      # ROS2 and applications
│   ├── ros2_basic_node.py     # ROS2 camera node
│   └── obstacle_detector.py   # Obstacle detection system
├── level-3-advanced/          # AI and SLAM
│   └── slam_system.py         # Visual SLAM implementation
├── level-4-expert/            # Advanced systems
│   └── humanoid_perception.py # Humanoid robot perception
└── README.md                  # Documentation
```

## 🚀 Quick Start Examples

### 🧭 Level 1: Beginner Examples

#### Basic Camera Connection
```python
import pyrealsense2 as rs
import numpy as np
import cv2

def basic_camera_test():
    """Test basic RealSense camera connection and streaming"""
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    pipeline.start(config)
    
    try:
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Apply colormap to depth
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # Stack images horizontally
            images = np.hstack((color_image, depth_colormap))
            
            # Display
            cv2.imshow('RealSense', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    basic_camera_test()
```

#### Distance Measurement
```python
import pyrealsense2 as rs
import numpy as np
import cv2

class DistanceMeasurement:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.click_point = None
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
            
    def run(self):
        profile = self.pipeline.start(self.config)
        align = rs.align(rs.stream.color)
        
        cv2.namedWindow('Distance Measurement')
        cv2.setMouseCallback('Distance Measurement', self.mouse_callback)
        
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                
                # Draw click point and distance
                if self.click_point:
                    x, y = self.click_point
                    distance = depth_frame.get_distance(x, y)
                    
                    cv2.circle(color_image, (x, y), 5, (0, 255, 0), -1)
                    cv2.putText(color_image, f"{distance:.2f}m", (x + 10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('Distance Measurement', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    dm = DistanceMeasurement()
    dm.run()
```

### ⚙️ Level 2: Intermediate Examples

#### ROS2 Camera Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        
        # Publishers
        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(self.config)
        
        # Timer for publishing
        self.timer = self.create_timer(0.033, self.publish_frames)
        
        self.get_logger().info('RealSense node started')
        
    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return
        
        # Convert to numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Create ROS messages
        color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')
        
        # Add timestamps
        now = self.get_clock().now().to_msg()
        color_msg.header.stamp = now
        depth_msg.header.stamp = now
        color_msg.header.frame_id = 'camera_link'
        depth_msg.header.frame_id = 'camera_link'
        
        # Publish
        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        
    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    
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

#### Obstacle Detection
```python
import pyrealsense2 as rs
import numpy as np
import cv2

class ObstacleDetector:
    def __init__(self, min_distance=0.3, max_distance=3.0, warning_distance=1.0):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.warning_distance = warning_distance
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Define safety zones (left, center, right)
        self.zones = [
            (0, 213, "LEFT"),
            (213, 427, "CENTER"),
            (427, 640, "RIGHT")
        ]
        
    def detect_obstacles(self, depth_frame):
        """Detect obstacles in each zone"""
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_meters = depth_image * depth_frame.get_units()
        
        obstacles = []
        for start_x, end_x, zone_name in self.zones:
            zone_depth = depth_meters[:, start_x:end_x]
            
            # Filter valid depths
            valid_depths = zone_depth[(zone_depth > self.min_distance) & 
                                      (zone_depth < self.max_distance)]
            
            if len(valid_depths) > 0:
                min_depth = np.min(valid_depths)
                avg_depth = np.mean(valid_depths)
                
                status = "CLEAR"
                if min_depth < self.warning_distance:
                    status = "WARNING"
                if min_depth < self.min_distance * 2:
                    status = "DANGER"
                
                obstacles.append({
                    'zone': zone_name,
                    'min_distance': min_depth,
                    'avg_distance': avg_depth,
                    'status': status
                })
        
        return obstacles
    
    def visualize(self, color_image, depth_frame, obstacles):
        """Visualize obstacles on color image"""
        vis_image = color_image.copy()
        
        colors = {
            'CLEAR': (0, 255, 0),
            'WARNING': (0, 255, 255),
            'DANGER': (0, 0, 255)
        }
        
        for i, obs in enumerate(obstacles):
            start_x, end_x, _ = self.zones[i]
            color = colors[obs['status']]
            
            # Draw zone rectangle
            cv2.rectangle(vis_image, (start_x, 0), (end_x, 480), color, 2)
            
            # Draw status text
            text = f"{obs['zone']}: {obs['min_distance']:.2f}m"
            cv2.putText(vis_image, text, (start_x + 10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(vis_image, obs['status'], (start_x + 10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return vis_image
    
    def run(self):
        self.pipeline.start(self.config)
        
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                obstacles = self.detect_obstacles(depth_frame)
                
                vis_image = self.visualize(color_image, depth_frame, obstacles)
                
                cv2.imshow('Obstacle Detection', vis_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = ObstacleDetector()
    detector.run()
```

### 🤖 Level 3: Advanced Examples

#### Visual SLAM System
```python
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from dataclasses import dataclass
from typing import List, Optional
import threading

@dataclass
class Keyframe:
    id: int
    pose: np.ndarray
    rgb_image: np.ndarray
    depth_image: np.ndarray
    features: dict
    point_cloud: Optional[o3d.geometry.PointCloud]

class VisualSLAM:
    def __init__(self):
        self.keyframes: List[Keyframe] = []
        self.current_pose = np.eye(4)
        self.trajectory = [np.eye(4)]
        self.global_map = o3d.geometry.PointCloud()
        
        # Feature detector
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Camera intrinsics (will be set from RealSense)
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0
        
        self.keyframe_threshold = 0.1  # meters
        self.last_keyframe_pose = np.eye(4)
        
    def set_intrinsics(self, intrinsics):
        """Set camera intrinsics from RealSense"""
        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.cx = intrinsics.ppx
        self.cy = intrinsics.ppy
        
    def extract_features(self, rgb_image):
        """Extract ORB features from image"""
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return {'keypoints': keypoints, 'descriptors': descriptors}
    
    def match_features(self, features1, features2):
        """Match features between two frames"""
        if features1['descriptors'] is None or features2['descriptors'] is None:
            return []
        
        matches = self.bf_matcher.match(features1['descriptors'], 
                                         features2['descriptors'])
        matches = sorted(matches, key=lambda x: x.distance)
        return matches[:100]  # Top 100 matches
    
    def estimate_pose(self, matches, features1, features2, depth_image):
        """Estimate relative pose from feature matches"""
        if len(matches) < 10:
            return None
        
        # Get matched points
        pts1 = np.float32([features1['keypoints'][m.queryIdx].pt for m in matches])
        pts2 = np.float32([features2['keypoints'][m.trainIdx].pt for m in matches])
        
        # Get 3D points from depth
        pts3d = []
        pts2d = []
        
        for i, (pt1, pt2) in enumerate(zip(pts1, pts2)):
            x, y = int(pt1[0]), int(pt1[1])
            if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                depth = depth_image[y, x] / 1000.0  # Convert to meters
                if 0.1 < depth < 10.0:
                    X = (x - self.cx) * depth / self.fx
                    Y = (y - self.cy) * depth / self.fy
                    pts3d.append([X, Y, depth])
                    pts2d.append(pt2)
        
        if len(pts3d) < 6:
            return None
        
        pts3d = np.array(pts3d, dtype=np.float32)
        pts2d = np.array(pts2d, dtype=np.float32)
        
        # Camera matrix
        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]], dtype=np.float32)
        
        # Solve PnP
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts3d, pts2d, K, None,
            iterationsCount=100,
            reprojectionError=8.0
        )
        
        if success:
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()
            return T
        
        return None
    
    def create_point_cloud(self, rgb_image, depth_image):
        """Create colored point cloud from RGB-D data"""
        height, width = depth_image.shape
        
        # Create point cloud
        points = []
        colors = []
        
        for v in range(0, height, 2):  # Subsample for performance
            for u in range(0, width, 2):
                depth = depth_image[v, u] / 1000.0
                if 0.1 < depth < 10.0:
                    x = (u - self.cx) * depth / self.fx
                    y = (v - self.cy) * depth / self.fy
                    z = depth
                    points.append([x, y, z])
                    colors.append(rgb_image[v, u][::-1] / 255.0)  # BGR to RGB
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
        
        return pcd
    
    def should_add_keyframe(self):
        """Determine if current frame should be a keyframe"""
        translation = np.linalg.norm(
            self.current_pose[:3, 3] - self.last_keyframe_pose[:3, 3]
        )
        return translation > self.keyframe_threshold
    
    def add_keyframe(self, rgb_image, depth_image, features):
        """Add new keyframe to the map"""
        pcd = self.create_point_cloud(rgb_image, depth_image)
        
        # Transform point cloud to global frame
        pcd.transform(self.current_pose)
        
        keyframe = Keyframe(
            id=len(self.keyframes),
            pose=self.current_pose.copy(),
            rgb_image=rgb_image.copy(),
            depth_image=depth_image.copy(),
            features=features,
            point_cloud=pcd
        )
        
        self.keyframes.append(keyframe)
        self.last_keyframe_pose = self.current_pose.copy()
        
        # Add to global map
        self.global_map += pcd
        
        # Downsample global map
        if len(self.keyframes) % 10 == 0:
            self.global_map = self.global_map.voxel_down_sample(0.02)
    
    def process_frame(self, rgb_image, depth_image):
        """Process new frame"""
        # Extract features
        features = self.extract_features(rgb_image)
        
        if len(self.keyframes) == 0:
            # First frame - add as keyframe
            self.add_keyframe(rgb_image, depth_image, features)
            return self.current_pose
        
        # Match with last keyframe
        last_kf = self.keyframes[-1]
        matches = self.match_features(last_kf.features, features)
        
        # Estimate relative pose
        relative_pose = self.estimate_pose(
            matches, last_kf.features, features, last_kf.depth_image
        )
        
        if relative_pose is not None:
            self.current_pose = last_kf.pose @ relative_pose
            self.trajectory.append(self.current_pose.copy())
        
        # Add keyframe if needed
        if self.should_add_keyframe():
            self.add_keyframe(rgb_image, depth_image, features)
        
        return self.current_pose
    
    def get_trajectory(self):
        """Get camera trajectory"""
        return np.array([pose[:3, 3] for pose in self.trajectory])
    
    def save_map(self, filename):
        """Save global map to file"""
        o3d.io.write_point_cloud(filename, self.global_map)

def main():
    # Initialize RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    
    # Get intrinsics
    depth_stream = profile.get_stream(rs.stream.depth)
    intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    
    # Initialize SLAM
    slam = VisualSLAM()
    slam.set_intrinsics(intrinsics)
    
    print("SLAM started. Press 'q' to quit, 's' to save map.")
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Process frame
            pose = slam.process_frame(color_image, depth_image)
            
            # Visualize
            vis_image = color_image.copy()
            cv2.putText(vis_image, f"Keyframes: {len(slam.keyframes)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(vis_image, f"Pose: {pose[:3, 3]}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow('Visual SLAM', vis_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                slam.save_map('slam_map.ply')
                print("Map saved to slam_map.ply")
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 🧑‍🏫 Level 4: Expert Examples

#### Humanoid Perception System
See the full implementation in [humanoid_perception.py](../code-examples/level-4-expert/humanoid_perception.py)

## 📦 Project Templates

### 🚀 Starter Templates

| Template | Description | Level |
|----------|-------------|-------|
| [Basic Camera App](https://github.com/realsenseai/templates/basic-camera) | Simple camera streaming application | Beginner |
| [ROS2 Package](https://github.com/realsenseai/templates/ros2-package) | ROS2 package template with RealSense | Intermediate |
| [AI Detection App](https://github.com/realsenseai/templates/ai-detection) | YOLO + RealSense object detection | Advanced |
| [SLAM Robot](https://github.com/realsenseai/templates/slam-robot) | Complete SLAM robot package | Advanced |
| [Edge AI System](https://github.com/realsenseai/templates/edge-ai) | OpenVINO + RealSense edge system | Expert |

### 📋 Template Usage

```bash
# Clone a template
git clone https://github.com/realsenseai/templates/basic-camera
cd basic-camera

# Install dependencies
pip install -r requirements.txt

# Run the application
python main.py
```

## 🛠️ Development Tools

### 🔧 Utility Scripts

| Script | Purpose | Usage |
|--------|---------|-------|
| `camera_info.py` | Display camera information | `python camera_info.py` |
| `record_bag.py` | Record ROS2 bag file | `python record_bag.py output.bag` |
| `calibration.py` | Camera calibration tool | `python calibration.py` |
| `benchmark.py` | Performance benchmarking | `python benchmark.py` |

### 📊 Debugging Tools

```python
# Debug utility for RealSense
import pyrealsense2 as rs

def print_device_info():
    """Print detailed device information"""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    for i, dev in enumerate(devices):
        print(f"\nDevice {i}: {dev.get_info(rs.camera_info.name)}")
        print(f"  Serial: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
        print(f"  USB Type: {dev.get_info(rs.camera_info.usb_type_descriptor)}")
        
        for sensor in dev.query_sensors():
            print(f"\n  Sensor: {sensor.get_info(rs.camera_info.name)}")
            for profile in sensor.get_stream_profiles():
                if profile.is_video_stream_profile():
                    vp = profile.as_video_stream_profile()
                    print(f"    {vp.stream_type()} {vp.width()}x{vp.height()} "
                          f"@ {vp.fps()}fps {vp.format()}")

if __name__ == '__main__':
    print_device_info()
```

## 📚 Documentation

### 📖 Code Documentation Standards

All code examples follow these documentation standards:

```python
def example_function(param1: int, param2: str = "default") -> dict:
    """
    Brief description of the function.
    
    Longer description with more details about what the function does,
    any important considerations, and usage notes.
    
    Args:
        param1: Description of param1
        param2: Description of param2 (default: "default")
        
    Returns:
        Description of the return value
        
    Raises:
        ValueError: When param1 is negative
        
    Example:
        >>> result = example_function(42, "test")
        >>> print(result)
        {'value': 42, 'name': 'test'}
    """
    pass
```

## 🤝 Contributing

### 📝 How to Contribute

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/new-example`
3. **Add your code example** following our standards
4. **Write tests** for your code
5. **Submit a pull request**

### ✅ Contribution Checklist

- [ ] Code follows PEP 8 style guidelines
- [ ] Docstrings included for all functions
- [ ] Type hints added where applicable
- [ ] Unit tests written and passing
- [ ] README updated with new example
- [ ] Example tested with actual RealSense camera

## 📞 Support

### 🆘 Getting Help
- **GitHub Issues**: [Report bugs](https://github.com/realsenseai/university-resources/issues)
- **Discord**: [Join our community](https://discord.gg/SQdtSH4J)
- **Email**: support@realsenseai.com

---

**Ready to start coding?** Clone the repository and run your first example!

```bash
git clone https://github.com/realsenseai/university-resources.git
cd university-resources/code-examples
python level-1-beginner/basic_camera_test.py
```
