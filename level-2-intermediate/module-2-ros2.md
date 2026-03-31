# Module 2: Using RealSense in ROS2

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Install and configure realsense-ros package
- Publish RGB, depth, and IMU topics
- Visualize RealSense data in RViz
- Use TF frames for coordinate transforms
- Create custom ROS2 nodes for RealSense

## 🚀 ROS2 Setup and Installation

### Installing ROS2 Humble

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### Installing RealSense ROS2 Package

```bash
# Install realsense-ros package
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Or build from source
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/realsenseai/realsense-ros.git
cd ~/ros2_ws
colcon build --packages-select realsense2_camera realsense2_description
```

## 🔧 Basic ROS2 RealSense Usage

### Launching RealSense Camera

```bash
# Basic launch
ros2 launch realsense2_camera rs_launch.py

# With specific parameters
ros2 launch realsense2_camera rs_launch.py \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=30 \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30
```

### Available Topics

```bash
# List all topics
ros2 topic list

# Common RealSense topics
/camera/color/image_raw          # RGB images
/camera/depth/image_rect_raw     # Depth images
/camera/depth/color/points       # Point clouds
/camera/accel/sample             # Accelerometer data
/camera/gyro/sample              # Gyroscope data
/camera/imu                      # Combined IMU data
```

### Viewing Data in RViz

```bash
# Launch RViz
rviz2

# Add displays:
# - Image: /camera/color/image_raw
# - Image: /camera/depth/image_rect_raw
# - PointCloud2: /camera/depth/color/points
```

## 🐍 Creating Custom ROS2 Nodes

### Basic RealSense Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        
        # Create publishers
        self.color_pub = self.create_publisher(Image, 'custom/color', 10)
        self.depth_pub = self.create_publisher(Image, 'custom/depth', 10)
        
        # Create subscribers
        self.color_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.color_callback, 
            10
        )
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/depth/image_rect_raw', 
            self.depth_callback, 
            10
        )
        
        # CV bridge
        self.bridge = CvBridge()
        
        self.get_logger().info('RealSense node started')

    def color_callback(self, msg):
        """Process color images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process image (example: add timestamp)
            cv2.putText(cv_image, f'Time: {msg.header.stamp.sec}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Convert back to ROS image
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            ros_image.header = msg.header
            
            # Publish processed image
            self.color_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing color image: {e}')

    def depth_callback(self, msg):
        """Process depth images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            
            # Process depth image (example: apply colormap)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(cv_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # Convert back to ROS image
            ros_image = self.bridge.cv2_to_imgmsg(depth_colormap, 'bgr8')
            ros_image.header = msg.header
            
            # Publish processed image
            self.depth_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

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

## 🧪 Hands-On Exercises

### Exercise 1: Basic ROS2 Setup
1. Install ROS2 Humble and realsense-ros
2. Launch RealSense camera
3. List available topics
4. View data in RViz

### Exercise 2: Custom Node Development
1. Create a custom ROS2 node
2. Subscribe to RealSense topics
3. Process and republish data
4. Test with RViz

### Exercise 3: TF Frames
1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Understand camera coordinate frames
3. Create custom TF transforms

## 🎯 Next Steps

Ready to continue? → [Module 3: Depth-Based Applications](./module-3-depth-applications.md)
