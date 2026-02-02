# Module 5: Mini Project: Obstacle Detection for AMR

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Build a complete ROS2-based obstacle detection system
- Implement real-time depth processing for robotics
- Create safety zone definitions and alerts
- Integrate with robot navigation systems
- Optimize performance for real-time operation

## 🎯 Project Overview

**Goal**: Create a ROS2-based obstacle detection node for Autonomous Mobile Robots (AMR) that can:
- Detect obstacles in real-time using RealSense depth data
- Define safety zones around the robot
- Publish obstacle information to navigation systems
- Provide visual feedback in RViz
- Handle multiple obstacle types and scenarios

## 🛠️ Project Requirements

### Functional Requirements
- [ ] Real-time obstacle detection from depth data
- [ ] Safety zone definition (critical, warning, safe)
- [ ] Obstacle classification (static, dynamic, unknown)
- [ ] ROS2 topic publishing for navigation
- [ ] RViz visualization
- [ ] Performance optimization for real-time operation

### Technical Requirements
- [ ] ROS2 Humble or Iron
- [ ] RealSense D435/D455 camera
- [ ] Python 3.8+ with required libraries
- [ ] RViz for visualization
- [ ] Performance monitoring and logging

## 🏗️ Project Structure

```
obstacle_detection_amr/
├── src/
│   ├── obstacle_detector/
│   │   ├── __init__.py
│   │   ├── obstacle_detector_node.py
│   │   ├── obstacle_processor.py
│   │   ├── safety_zone_manager.py
│   │   └── visualization.py
│   └── obstacle_detection_amr/
│       ├── package.xml
│       └── setup.py
├── launch/
│   └── obstacle_detection.launch.py
├── config/
│   └── obstacle_detection.yaml
├── rviz/
│   └── obstacle_detection.rviz
└── README.md
```

## 💻 Implementation

### Step 1: ROS2 Package Setup

Create `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>obstacle_detection_amr</name>
  <version>1.0.0</version>
  <description>RealSense-based obstacle detection for AMR</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create `setup.py`:

```python
from setuptools import setup

package_name = 'obstacle_detection_amr'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/obstacle_detection.launch.py']),
        ('share/' + package_name + '/config', ['config/obstacle_detection.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/obstacle_detection.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='RealSense-based obstacle detection for AMR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = obstacle_detection_amr.obstacle_detector_node:main',
        ],
    },
)
```

### Step 2: Obstacle Processor

Create `obstacle_processor.py`:

```python
import numpy as np
import cv2
from typing import List, Dict, Tuple

class ObstacleProcessor:
    def __init__(self, config):
        self.config = config
        self.min_obstacle_area = config.get('min_obstacle_area', 1000)
        self.max_obstacle_distance = config.get('max_obstacle_distance', 3000)
        self.min_obstacle_distance = config.get('min_obstacle_distance', 200)
        
    def process_depth_image(self, depth_image: np.ndarray) -> Dict:
        """Process depth image to detect obstacles"""
        # Create obstacle mask
        obstacle_mask = self.create_obstacle_mask(depth_image)
        
        # Find obstacle contours
        obstacles = self.find_obstacles(obstacle_mask)
        
        # Classify obstacles
        classified_obstacles = self.classify_obstacles(obstacles, depth_image)
        
        # Calculate safety zones
        safety_zones = self.calculate_safety_zones(depth_image)
        
        return {
            'obstacles': classified_obstacles,
            'safety_zones': safety_zones,
            'obstacle_mask': obstacle_mask
        }
    
    def create_obstacle_mask(self, depth_image: np.ndarray) -> np.ndarray:
        """Create mask for potential obstacles"""
        # Filter valid depth range
        valid_depth = (depth_image > self.min_obstacle_distance) & \
                     (depth_image < self.max_obstacle_distance) & \
                     (depth_image > 0)
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        obstacle_mask = cv2.morphologyEx(valid_depth.astype(np.uint8), 
                                       cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        return obstacle_mask.astype(bool)
    
    def find_obstacles(self, obstacle_mask: np.ndarray) -> List[Dict]:
        """Find obstacle contours and properties"""
        contours, _ = cv2.findContours(obstacle_mask.astype(np.uint8), 
                                      cv2.RETR_EXTERNAL, 
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_obstacle_area:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Calculate distance (simplified)
                distance = self.estimate_distance(contour, obstacle_mask)
                
                obstacles.append({
                    'id': len(obstacles),
                    'contour': contour,
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'area': area,
                    'distance': distance,
                    'type': 'unknown'
                })
        
        return obstacles
    
    def classify_obstacles(self, obstacles: List[Dict], depth_image: np.ndarray) -> List[Dict]:
        """Classify obstacles based on properties"""
        for obstacle in obstacles:
            # Simple classification based on size and distance
            if obstacle['area'] > 5000 and obstacle['distance'] < 1000:
                obstacle['type'] = 'large_close'
            elif obstacle['area'] < 2000:
                obstacle['type'] = 'small'
            else:
                obstacle['type'] = 'medium'
        
        return obstacles
    
    def calculate_safety_zones(self, depth_image: np.ndarray) -> Dict:
        """Calculate safety zones based on depth"""
        zones = {
            'critical': depth_image < 500,  # < 50cm
            'warning': (depth_image >= 500) & (depth_image < 1000),  # 50cm - 1m
            'safe': depth_image >= 1000  # > 1m
        }
        
        return zones
    
    def estimate_distance(self, contour: np.ndarray, depth_image: np.ndarray) -> float:
        """Estimate distance to obstacle"""
        # Get points within contour
        mask = np.zeros(depth_image.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [contour], 255)
        
        # Calculate mean distance
        distances = depth_image[mask > 0]
        valid_distances = distances[distances > 0]
        
        if len(valid_distances) > 0:
            return np.mean(valid_distances)
        else:
            return 0.0
```

### Step 3: Safety Zone Manager

Create `safety_zone_manager.py`:

```python
import numpy as np
from typing import Dict, List

class SafetyZoneManager:
    def __init__(self, config):
        self.config = config
        self.critical_distance = config.get('critical_distance', 500)
        self.warning_distance = config.get('warning_distance', 1000)
        self.safe_distance = config.get('safe_distance', 1500)
        
    def analyze_safety_zones(self, depth_image: np.ndarray) -> Dict:
        """Analyze safety zones and generate alerts"""
        zones = self.calculate_zones(depth_image)
        alerts = self.generate_alerts(zones)
        
        return {
            'zones': zones,
            'alerts': alerts,
            'safety_status': self.determine_safety_status(alerts)
        }
    
    def calculate_zones(self, depth_image: np.ndarray) -> Dict:
        """Calculate safety zones"""
        zones = {
            'critical': depth_image < self.critical_distance,
            'warning': (depth_image >= self.critical_distance) & 
                      (depth_image < self.warning_distance),
            'safe': depth_image >= self.warning_distance
        }
        
        return zones
    
    def generate_alerts(self, zones: Dict) -> List[Dict]:
        """Generate safety alerts"""
        alerts = []
        
        # Check critical zone
        if np.any(zones['critical']):
            alerts.append({
                'level': 'critical',
                'message': 'Obstacle in critical zone!',
                'action': 'STOP'
            })
        
        # Check warning zone
        elif np.any(zones['warning']):
            alerts.append({
                'level': 'warning',
                'message': 'Obstacle approaching',
                'action': 'SLOW_DOWN'
            })
        
        return alerts
    
    def determine_safety_status(self, alerts: List[Dict]) -> str:
        """Determine overall safety status"""
        if any(alert['level'] == 'critical' for alert in alerts):
            return 'CRITICAL'
        elif any(alert['level'] == 'warning' for alert in alerts):
            return 'WARNING'
        else:
            return 'SAFE'
```

### Step 4: Main ROS2 Node

Create `obstacle_detector_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

from obstacle_processor import ObstacleProcessor
from safety_zone_manager import SafetyZoneManager

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        
        # Load configuration
        self.config = self.load_config()
        
        # Initialize components
        self.obstacle_processor = ObstacleProcessor(self.config)
        self.safety_zone_manager = SafetyZoneManager(self.config)
        self.bridge = CvBridge()
        
        # Create publishers
        self.obstacle_pub = self.create_publisher(String, 'obstacle_detection/obstacles', 10)
        self.safety_pub = self.create_publisher(String, 'obstacle_detection/safety', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_detection/markers', 10)
        self.status_pub = self.create_publisher(String, 'obstacle_detection/status', 10)
        
        # Create subscribers
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/depth/image_rect_raw', 
            self.depth_callback, 
            10
        )
        
        self.get_logger().info('Obstacle detector node started')
    
    def load_config(self) -> dict:
        """Load configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(__file__), 
            '..', 'config', 'obstacle_detection.yaml'
        )
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            self.get_logger().info('Configuration loaded successfully')
            return config
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}, using defaults')
            return self.get_default_config()
    
    def get_default_config(self) -> dict:
        """Get default configuration"""
        return {
            'min_obstacle_area': 1000,
            'max_obstacle_distance': 3000,
            'min_obstacle_distance': 200,
            'critical_distance': 500,
            'warning_distance': 1000,
            'safe_distance': 1500
        }
    
    def depth_callback(self, msg):
        """Process depth image and detect obstacles"""
        try:
            # Convert ROS image to OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            
            # Process obstacles
            obstacle_data = self.obstacle_processor.process_depth_image(depth_image)
            
            # Analyze safety zones
            safety_data = self.safety_zone_manager.analyze_safety_zones(depth_image)
            
            # Publish results
            self.publish_obstacles(obstacle_data['obstacles'])
            self.publish_safety_status(safety_data)
            self.publish_markers(obstacle_data['obstacles'], safety_data['zones'])
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def publish_obstacles(self, obstacles):
        """Publish obstacle information"""
        obstacle_msg = String()
        obstacle_msg.data = f"Detected {len(obstacles)} obstacles"
        self.obstacle_pub.publish(obstacle_msg)
    
    def publish_safety_status(self, safety_data):
        """Publish safety status"""
        safety_msg = String()
        safety_msg.data = safety_data['safety_status']
        self.safety_pub.publish(safety_msg)
        
        # Publish individual alerts
        for alert in safety_data['alerts']:
            self.get_logger().warn(f"{alert['level']}: {alert['message']}")
    
    def publish_markers(self, obstacles, safety_zones):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Create obstacle markers
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "camera_depth_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position (simplified)
            marker.pose.position.x = obstacle['distance'] / 1000.0  # Convert to meters
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            
            # Set scale
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Set color based on type
            if obstacle['type'] == 'large_close':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.7
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    
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

### Step 5: Launch File

Create `launch/obstacle_detection.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value='config/obstacle_detection.yaml',
            description='Path to configuration file'
        ),
        
        # RealSense camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            parameters=[{
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30
            }]
        ),
        
        # Obstacle detector node
        Node(
            package='obstacle_detection_amr',
            executable='obstacle_detector_node',
            name='obstacle_detector',
            parameters=[LaunchConfiguration('config_file')]
        ),
        
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'rviz/obstacle_detection.rviz']
        )
    ])
```

### Step 6: Configuration File

Create `config/obstacle_detection.yaml`:

```yaml
# Obstacle Detection Configuration

# Obstacle detection parameters
min_obstacle_area: 1000          # Minimum obstacle area in pixels
max_obstacle_distance: 3000      # Maximum detection distance in mm
min_obstacle_distance: 200       # Minimum detection distance in mm

# Safety zone parameters
critical_distance: 500           # Critical zone distance in mm
warning_distance: 1000           # Warning zone distance in mm
safe_distance: 1500              # Safe zone distance in mm

# Performance parameters
processing_fps: 10               # Target processing FPS
visualization_enabled: true      # Enable RViz visualization
logging_enabled: true            # Enable logging
```

## 🧪 Testing Your Application

### Test Cases

1. **Basic Obstacle Detection**:
   - Place objects at different distances
   - Verify obstacle detection accuracy
   - Test with different object sizes

2. **Safety Zone Testing**:
   - Test critical zone alerts
   - Verify warning zone detection
   - Check safe zone behavior

3. **Performance Testing**:
   - Monitor CPU usage
   - Check memory consumption
   - Verify real-time performance

4. **Integration Testing**:
   - Test with navigation system
   - Verify topic publishing
   - Check RViz visualization

### Running the Application

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select obstacle_detection_amr

# Source the workspace
source install/setup.bash

# Launch the application
ros2 launch obstacle_detection_amr obstacle_detection.launch.py

# Monitor topics
ros2 topic list
ros2 topic echo /obstacle_detection/obstacles
ros2 topic echo /obstacle_detection/safety
```

## 🎯 Project Extensions

### Advanced Features

1. **Dynamic Obstacle Tracking**:
   - Track moving obstacles
   - Predict obstacle trajectories
   - Update safety zones dynamically

2. **Multi-Camera Support**:
   - Fuse data from multiple cameras
   - Handle camera synchronization
   - Improve detection accuracy

3. **Machine Learning Integration**:
   - Object classification
   - Obstacle prediction
   - Adaptive thresholding

4. **Navigation Integration**:
   - Cost map generation
   - Path planning integration
   - Dynamic obstacle avoidance

## 🎉 Congratulations!

You've successfully completed Level 2 of RealSense University! You now have:
- Advanced point cloud processing skills
- ROS2 integration experience
- Depth-based application development
- Cross-platform optimization knowledge
- A complete obstacle detection system

## 🎯 Next Steps

Ready to advance to the next level? Check out [Level 3: Advanced — AI + Robotics with RealSense](../level-3-advanced/) to learn about:
- Visual SLAM and mapping
- Sensor fusion techniques
- AI perception pipelines
- Cloud robotics integration

## 📚 Additional Resources

- [ROS2 Navigation Stack](https://navigation.ros.org/)
- [RealSense ROS2 Package](https://github.com/realsenseai/realsense-ros)
- [RViz Visualization](http://wiki.ros.org/rviz)
- [ROS2 Best Practices](https://docs.ros.org/en/humble/Contributing/Developer-Guide.html)
