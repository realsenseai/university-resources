# Track 3: RealSense Developer SDK Extensions

## 🎯 Learning Objectives

By the end of this track, you will be able to:
- Understand the RealSense SDK architecture and internals
- Create custom processing blocks and filters
- Develop ROS2 nodes and packages for RealSense
- Extend pyrealsense2 with custom functionality
- Contribute to open-source RealSense projects

## 🏗️ RealSense SDK Architecture

### SDK Overview

The **librealsense** SDK is organized into several layers:

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│            (Your Code, ROS2, OpenCV, etc.)                  │
├─────────────────────────────────────────────────────────────┤
│                    Language Bindings                         │
│         (Python, C#, Java, Node.js, Rust)                   │
├─────────────────────────────────────────────────────────────┤
│                      C++ API                                 │
│        (rs2::pipeline, rs2::frame, rs2::sensor)             │
├─────────────────────────────────────────────────────────────┤
│                    Processing Blocks                         │
│      (Filters, Align, Pointcloud, Colorizer)                │
├─────────────────────────────────────────────────────────────┤
│                      Core Library                            │
│           (Device management, Streaming)                    │
├─────────────────────────────────────────────────────────────┤
│                    Backend Layer                             │
│         (USB, UVC, HID, Platform-specific)                  │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

```python
import pyrealsense2 as rs

# Context: Manages device discovery
ctx = rs.context()

# Device: Represents physical camera
devices = ctx.query_devices()
device = devices[0]

# Sensor: Camera sensor (depth, color, IMU)
sensors = device.query_sensors()
depth_sensor = device.first_depth_sensor()

# Pipeline: High-level streaming interface
pipeline = rs.pipeline()
config = rs.config()

# Processing Block: Frame processing
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()

# Frameset: Collection of synchronized frames
frames = pipeline.wait_for_frames()
```

## 🔧 Creating Custom Processing Blocks

### Processing Block Interface

```python
import pyrealsense2 as rs
import numpy as np
from typing import Callable, Optional

class CustomProcessingBlock:
    """Base class for custom processing blocks"""
    
    def __init__(self, name: str):
        self.name = name
        self._callback = None
        
    def process(self, frame: rs.frame) -> rs.frame:
        """Process a single frame - override in subclass"""
        raise NotImplementedError
    
    def set_callback(self, callback: Callable):
        """Set callback for processed frames"""
        self._callback = callback
    
    def invoke(self, frame: rs.frame):
        """Invoke processing and call callback"""
        processed = self.process(frame)
        if self._callback and processed:
            self._callback(processed)
        return processed


class AdaptiveThresholdFilter(CustomProcessingBlock):
    """Custom filter that applies adaptive thresholding to depth"""
    
    def __init__(self, block_size: int = 11, c: float = 2):
        super().__init__("AdaptiveThreshold")
        self.block_size = block_size
        self.c = c
        
    def process(self, frame: rs.frame) -> rs.frame:
        if not frame.is_depth_frame():
            return frame
        
        depth_frame = frame.as_depth_frame()
        depth_data = np.asanyarray(depth_frame.get_data())
        
        # Normalize depth to 8-bit
        depth_normalized = cv2.normalize(
            depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        
        # Apply adaptive threshold
        thresholded = cv2.adaptiveThreshold(
            depth_normalized,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self.block_size,
            self.c
        )
        
        # Convert back to depth format
        result = depth_data.copy()
        result[thresholded == 0] = 0
        
        # Create new frame with processed data
        return self._create_depth_frame(frame, result)
    
    def _create_depth_frame(self, original: rs.frame, 
                            new_data: np.ndarray) -> rs.frame:
        """Create a new depth frame from processed data"""
        # Note: In practice, you'd use the SDK's frame allocation
        # This is a simplified example
        return original  # Return original for now


class MedianDepthFilter(CustomProcessingBlock):
    """Median filter for depth noise reduction"""
    
    def __init__(self, kernel_size: int = 5):
        super().__init__("MedianFilter")
        self.kernel_size = kernel_size
        
    def process(self, frame: rs.frame) -> rs.frame:
        if not frame.is_depth_frame():
            return frame
        
        depth_frame = frame.as_depth_frame()
        depth_data = np.asanyarray(depth_frame.get_data())
        
        # Apply median filter
        filtered = cv2.medianBlur(depth_data.astype(np.float32), self.kernel_size)
        
        return frame  # Return with processed data


class DepthEdgePreserver(CustomProcessingBlock):
    """Edge-preserving smoothing for depth images"""
    
    def __init__(self, sigma_s: float = 60, sigma_r: float = 0.4):
        super().__init__("EdgePreserver")
        self.sigma_s = sigma_s
        self.sigma_r = sigma_r
        
    def process(self, frame: rs.frame) -> rs.frame:
        if not frame.is_depth_frame():
            return frame
        
        depth_frame = frame.as_depth_frame()
        depth_data = np.asanyarray(depth_frame.get_data())
        
        # Normalize for processing
        depth_normalized = depth_data.astype(np.float32) / 65535.0
        
        # Apply edge-preserving filter
        filtered = cv2.edgePreservingFilter(
            cv2.cvtColor(
                (depth_normalized * 255).astype(np.uint8),
                cv2.COLOR_GRAY2BGR
            ),
            flags=cv2.RECURS_FILTER,
            sigma_s=self.sigma_s,
            sigma_r=self.sigma_r
        )
        
        # Convert back
        filtered_gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        result = (filtered_gray.astype(np.float32) / 255.0 * 65535).astype(np.uint16)
        
        return frame
```

### Filter Pipeline Builder

```python
class FilterPipeline:
    """Build and manage custom filter pipelines"""
    
    def __init__(self):
        self.filters = []
        
    def add_filter(self, filter_block):
        """Add filter to pipeline"""
        self.filters.append(filter_block)
        return self
    
    def add_decimation(self, magnitude: int = 2):
        """Add decimation filter"""
        f = rs.decimation_filter()
        f.set_option(rs.option.filter_magnitude, magnitude)
        self.filters.append(f)
        return self
    
    def add_threshold(self, min_dist: float = 0.15, max_dist: float = 4.0):
        """Add threshold filter"""
        f = rs.threshold_filter()
        f.set_option(rs.option.min_distance, min_dist)
        f.set_option(rs.option.max_distance, max_dist)
        self.filters.append(f)
        return self
    
    def add_spatial(self, magnitude: int = 2, smooth_alpha: float = 0.5,
                   smooth_delta: int = 20, holes_fill: int = 0):
        """Add spatial filter"""
        f = rs.spatial_filter()
        f.set_option(rs.option.filter_magnitude, magnitude)
        f.set_option(rs.option.filter_smooth_alpha, smooth_alpha)
        f.set_option(rs.option.filter_smooth_delta, smooth_delta)
        f.set_option(rs.option.holes_fill, holes_fill)
        self.filters.append(f)
        return self
    
    def add_temporal(self, smooth_alpha: float = 0.4, smooth_delta: int = 20,
                    persistence: int = 3):
        """Add temporal filter"""
        f = rs.temporal_filter()
        f.set_option(rs.option.filter_smooth_alpha, smooth_alpha)
        f.set_option(rs.option.filter_smooth_delta, smooth_delta)
        f.set_option(rs.option.holes_fill, persistence)
        self.filters.append(f)
        return self
    
    def add_hole_filling(self, mode: int = 1):
        """Add hole filling filter"""
        f = rs.hole_filling_filter()
        f.set_option(rs.option.holes_fill, mode)
        self.filters.append(f)
        return self
    
    def add_custom(self, custom_filter: CustomProcessingBlock):
        """Add custom processing block"""
        self.filters.append(custom_filter)
        return self
    
    def process(self, frame: rs.frame) -> rs.frame:
        """Process frame through pipeline"""
        result = frame
        for f in self.filters:
            if isinstance(f, CustomProcessingBlock):
                result = f.process(result)
            else:
                result = f.process(result)
        return result
    
    def get_config(self) -> dict:
        """Get pipeline configuration"""
        config = {'filters': []}
        for f in self.filters:
            if isinstance(f, CustomProcessingBlock):
                config['filters'].append({
                    'type': 'custom',
                    'name': f.name
                })
            else:
                config['filters'].append({
                    'type': str(type(f).__name__),
                    'options': self._get_filter_options(f)
                })
        return config
    
    def _get_filter_options(self, f) -> dict:
        """Extract filter options"""
        options = {}
        try:
            for opt in f.get_supported_options():
                options[str(opt)] = f.get_option(opt)
        except:
            pass
        return options


# Usage example
def create_high_quality_pipeline():
    return (FilterPipeline()
            .add_decimation(2)
            .add_threshold(0.15, 4.0)
            .add_spatial(2, 0.5, 20, 1)
            .add_temporal(0.4, 20, 3)
            .add_hole_filling(1))
```

## 🤖 ROS2 Node Development

### Custom RealSense ROS2 Node

```python
#!/usr/bin/env python3
"""
Advanced RealSense ROS2 Node with custom features
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import pyrealsense2 as rs
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Dict, Any
import json

@dataclass
class StreamConfig:
    stream_type: rs.stream
    width: int
    height: int
    format: rs.format
    fps: int
    enabled: bool = True

class AdvancedRealSenseNode(Node):
    def __init__(self):
        super().__init__('advanced_realsense_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize components
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Stream configurations
        self.stream_configs = self._load_stream_configs()
        
        # Initialize RealSense
        self._init_realsense()
        
        # Create publishers
        self._create_publishers()
        
        # Processing pipeline
        self.filter_pipeline = self._create_filter_pipeline()
        
        # Timer for main loop
        self.timer = self.create_timer(1.0/30.0, self.publish_frames)
        
        self.get_logger().info('Advanced RealSense node initialized')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('serial_number', '')
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('depth_fps', 30)
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('filter_config', '{}')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_tf', True)
    
    def _load_stream_configs(self) -> Dict[str, StreamConfig]:
        """Load stream configurations from parameters"""
        configs = {}
        
        if self.get_parameter('enable_depth').value:
            configs['depth'] = StreamConfig(
                stream_type=rs.stream.depth,
                width=self.get_parameter('depth_width').value,
                height=self.get_parameter('depth_height').value,
                format=rs.format.z16,
                fps=self.get_parameter('depth_fps').value
            )
        
        if self.get_parameter('enable_color').value:
            configs['color'] = StreamConfig(
                stream_type=rs.stream.color,
                width=self.get_parameter('color_width').value,
                height=self.get_parameter('color_height').value,
                format=rs.format.bgr8,
                fps=self.get_parameter('color_fps').value
            )
        
        if self.get_parameter('enable_imu').value:
            configs['accel'] = StreamConfig(
                stream_type=rs.stream.accel,
                width=0, height=0,
                format=rs.format.motion_xyz32f,
                fps=250
            )
            configs['gyro'] = StreamConfig(
                stream_type=rs.stream.gyro,
                width=0, height=0,
                format=rs.format.motion_xyz32f,
                fps=400
            )
        
        return configs
    
    def _init_realsense(self):
        """Initialize RealSense pipeline"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable specific device if serial number provided
        serial = self.get_parameter('serial_number').value
        if serial:
            self.config.enable_device(serial)
        
        # Configure streams
        for name, stream_config in self.stream_configs.items():
            if stream_config.stream_type in [rs.stream.accel, rs.stream.gyro]:
                self.config.enable_stream(
                    stream_config.stream_type,
                    stream_config.format,
                    stream_config.fps
                )
            else:
                self.config.enable_stream(
                    stream_config.stream_type,
                    stream_config.width,
                    stream_config.height,
                    stream_config.format,
                    stream_config.fps
                )
        
        # Start pipeline
        self.profile = self.pipeline.start(self.config)
        
        # Get device info
        device = self.profile.get_device()
        self.device_name = device.get_info(rs.camera_info.name)
        self.device_serial = device.get_info(rs.camera_info.serial_number)
        self.get_logger().info(f'Connected to: {self.device_name} ({self.device_serial})')
        
        # Configure depth sensor
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(
                rs.option.visual_preset, 
                rs.rs400_visual_preset.high_accuracy
            )
        
        # Create align object
        self.align = rs.align(rs.stream.color)
        
        # Get intrinsics
        self._load_intrinsics()
    
    def _load_intrinsics(self):
        """Load camera intrinsics"""
        self.intrinsics = {}
        
        for stream_type in [rs.stream.depth, rs.stream.color]:
            try:
                stream_profile = self.profile.get_stream(stream_type)
                video_profile = stream_profile.as_video_stream_profile()
                self.intrinsics[stream_type] = video_profile.get_intrinsics()
            except:
                pass
    
    def _create_publishers(self):
        """Create ROS2 publishers"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publishers = {}
        
        if 'depth' in self.stream_configs:
            self.publishers['depth'] = self.create_publisher(
                Image, 'camera/depth/image_raw', qos)
            self.publishers['depth_info'] = self.create_publisher(
                CameraInfo, 'camera/depth/camera_info', qos)
        
        if 'color' in self.stream_configs:
            self.publishers['color'] = self.create_publisher(
                Image, 'camera/color/image_raw', qos)
            self.publishers['color_info'] = self.create_publisher(
                CameraInfo, 'camera/color/camera_info', qos)
        
        if self.get_parameter('enable_pointcloud').value:
            self.publishers['pointcloud'] = self.create_publisher(
                PointCloud2, 'camera/pointcloud', qos)
        
        if 'accel' in self.stream_configs:
            self.publishers['imu'] = self.create_publisher(
                Imu, 'camera/imu', qos)
    
    def _create_filter_pipeline(self) -> FilterPipeline:
        """Create depth filter pipeline"""
        filter_config = self.get_parameter('filter_config').value
        
        try:
            config = json.loads(filter_config)
        except:
            config = {}
        
        pipeline = FilterPipeline()
        
        # Default high-quality pipeline
        pipeline.add_decimation(config.get('decimation', 2))
        pipeline.add_threshold(
            config.get('min_distance', 0.15),
            config.get('max_distance', 4.0)
        )
        pipeline.add_spatial(
            config.get('spatial_magnitude', 2),
            config.get('spatial_alpha', 0.5),
            config.get('spatial_delta', 20)
        )
        pipeline.add_temporal(
            config.get('temporal_alpha', 0.4),
            config.get('temporal_delta', 20)
        )
        pipeline.add_hole_filling(config.get('holes_fill', 1))
        
        return pipeline
    
    def publish_frames(self):
        """Publish camera frames"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            self.get_logger().warn(f'Frame timeout: {e}')
            return
        
        # Align frames
        aligned_frames = self.align.process(frames)
        
        timestamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').value
        
        # Publish depth
        depth_frame = aligned_frames.get_depth_frame()
        if depth_frame and 'depth' in self.publishers:
            # Apply filters
            filtered_depth = self.filter_pipeline.process(depth_frame)
            
            depth_image = np.asanyarray(filtered_depth.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = f'{frame_id}_depth_optical_frame'
            self.publishers['depth'].publish(depth_msg)
            
            # Publish camera info
            self._publish_camera_info('depth', rs.stream.depth, timestamp)
        
        # Publish color
        color_frame = aligned_frames.get_color_frame()
        if color_frame and 'color' in self.publishers:
            color_image = np.asanyarray(color_frame.get_data())
            color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = f'{frame_id}_color_optical_frame'
            self.publishers['color'].publish(color_msg)
            
            # Publish camera info
            self._publish_camera_info('color', rs.stream.color, timestamp)
        
        # Publish pointcloud
        if 'pointcloud' in self.publishers and depth_frame and color_frame:
            self._publish_pointcloud(depth_frame, color_frame, timestamp, frame_id)
        
        # Publish IMU
        self._publish_imu(frames, timestamp, frame_id)
        
        # Publish TF
        if self.get_parameter('publish_tf').value:
            self._publish_tf(timestamp, frame_id)
    
    def _publish_camera_info(self, name: str, stream_type: rs.stream, timestamp):
        """Publish camera info message"""
        if stream_type not in self.intrinsics:
            return
        
        intrinsics = self.intrinsics[stream_type]
        
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = f"{self.get_parameter('frame_id').value}_{name}_optical_frame"
        
        info.width = intrinsics.width
        info.height = intrinsics.height
        
        info.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        info.d = list(intrinsics.coeffs)
        
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        info.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.publishers[f'{name}_info'].publish(info)
    
    def _publish_pointcloud(self, depth_frame, color_frame, timestamp, frame_id):
        """Publish pointcloud message"""
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        texcoords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        
        # Filter invalid points
        valid = ~np.all(vertices == 0, axis=1)
        vertices = vertices[valid]
        
        # Create PointCloud2 message
        from sensor_msgs_py import point_cloud2
        
        cloud_msg = point_cloud2.create_cloud_xyz32(
            Header(stamp=timestamp, frame_id=f'{frame_id}_depth_optical_frame'),
            vertices
        )
        
        self.publishers['pointcloud'].publish(cloud_msg)
    
    def _publish_imu(self, frames, timestamp, frame_id):
        """Publish IMU message"""
        if 'imu' not in self.publishers:
            return
        
        accel_frame = None
        gyro_frame = None
        
        for frame in frames:
            if frame.is_motion_frame():
                if frame.get_profile().stream_type() == rs.stream.accel:
                    accel_frame = frame.as_motion_frame()
                elif frame.get_profile().stream_type() == rs.stream.gyro:
                    gyro_frame = frame.as_motion_frame()
        
        if accel_frame and gyro_frame:
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = f'{frame_id}_imu_frame'
            
            accel_data = accel_frame.get_motion_data()
            imu_msg.linear_acceleration.x = accel_data.x
            imu_msg.linear_acceleration.y = accel_data.y
            imu_msg.linear_acceleration.z = accel_data.z
            
            gyro_data = gyro_frame.get_motion_data()
            imu_msg.angular_velocity.x = gyro_data.x
            imu_msg.angular_velocity.y = gyro_data.y
            imu_msg.angular_velocity.z = gyro_data.z
            
            self.publishers['imu'].publish(imu_msg)
    
    def _publish_tf(self, timestamp, frame_id):
        """Publish TF transforms"""
        # Camera link to optical frames
        transforms = [
            (f'{frame_id}', f'{frame_id}_depth_optical_frame', [0, 0, 0], [-0.5, 0.5, -0.5, 0.5]),
            (f'{frame_id}', f'{frame_id}_color_optical_frame', [0, 0, 0], [-0.5, 0.5, -0.5, 0.5]),
            (f'{frame_id}', f'{frame_id}_imu_frame', [0, 0, 0], [0, 0, 0, 1]),
        ]
        
        for parent, child, translation, rotation in transforms:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = rotation[0]
            t.transform.rotation.y = rotation[1]
            t.transform.rotation.z = rotation[2]
            t.transform.rotation.w = rotation[3]
            
            self.tf_broadcaster.sendTransform(t)
    
    def destroy_node(self):
        """Cleanup"""
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedRealSenseNode()
    
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

## 🐍 Extending pyrealsense2

### Custom Python Module

```python
"""
Extended pyrealsense2 functionality
"""
import pyrealsense2 as rs
import numpy as np
import cv2
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass
import json

@dataclass
class CameraConfig:
    """Camera configuration"""
    serial_number: Optional[str] = None
    depth_resolution: Tuple[int, int] = (640, 480)
    color_resolution: Tuple[int, int] = (640, 480)
    depth_fps: int = 30
    color_fps: int = 30
    enable_imu: bool = True
    visual_preset: str = "high_accuracy"

class RealSenseDevice:
    """Extended RealSense device wrapper"""
    
    VISUAL_PRESETS = {
        'high_accuracy': rs.rs400_visual_preset.high_accuracy,
        'high_density': rs.rs400_visual_preset.high_density,
        'medium_density': rs.rs400_visual_preset.medium_density,
        'default': rs.rs400_visual_preset.default
    }
    
    def __init__(self, config: CameraConfig = None):
        self.config = config or CameraConfig()
        self.pipeline = None
        self.profile = None
        self.align = None
        self.intrinsics = {}
        self.extrinsics = {}
        
    def connect(self) -> bool:
        """Connect to RealSense device"""
        try:
            self.pipeline = rs.pipeline()
            rs_config = rs.config()
            
            if self.config.serial_number:
                rs_config.enable_device(self.config.serial_number)
            
            # Configure streams
            rs_config.enable_stream(
                rs.stream.depth,
                self.config.depth_resolution[0],
                self.config.depth_resolution[1],
                rs.format.z16,
                self.config.depth_fps
            )
            
            rs_config.enable_stream(
                rs.stream.color,
                self.config.color_resolution[0],
                self.config.color_resolution[1],
                rs.format.bgr8,
                self.config.color_fps
            )
            
            if self.config.enable_imu:
                rs_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
                rs_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
            
            self.profile = self.pipeline.start(rs_config)
            self.align = rs.align(rs.stream.color)
            
            # Apply visual preset
            device = self.profile.get_device()
            depth_sensor = device.first_depth_sensor()
            if self.config.visual_preset in self.VISUAL_PRESETS:
                depth_sensor.set_option(
                    rs.option.visual_preset,
                    self.VISUAL_PRESETS[self.config.visual_preset]
                )
            
            # Load calibration
            self._load_calibration()
            
            return True
            
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def _load_calibration(self):
        """Load camera calibration data"""
        for stream_type in [rs.stream.depth, rs.stream.color]:
            try:
                stream = self.profile.get_stream(stream_type)
                video_stream = stream.as_video_stream_profile()
                self.intrinsics[stream_type] = video_stream.get_intrinsics()
            except:
                pass
        
        # Load extrinsics
        try:
            depth_stream = self.profile.get_stream(rs.stream.depth)
            color_stream = self.profile.get_stream(rs.stream.color)
            self.extrinsics['depth_to_color'] = depth_stream.get_extrinsics_to(color_stream)
            self.extrinsics['color_to_depth'] = color_stream.get_extrinsics_to(depth_stream)
        except:
            pass
    
    def get_frames(self, align: bool = True, timeout_ms: int = 1000) -> Optional[Dict[str, Any]]:
        """Get aligned frames"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=timeout_ms)
            
            if align:
                frames = self.align.process(frames)
            
            result = {
                'timestamp': frames.get_timestamp(),
                'frame_number': frames.get_frame_number()
            }
            
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                result['depth'] = {
                    'frame': depth_frame,
                    'data': np.asanyarray(depth_frame.get_data()),
                    'timestamp': depth_frame.get_timestamp()
                }
            
            color_frame = frames.get_color_frame()
            if color_frame:
                result['color'] = {
                    'frame': color_frame,
                    'data': np.asanyarray(color_frame.get_data()),
                    'timestamp': color_frame.get_timestamp()
                }
            
            # IMU data
            if self.config.enable_imu:
                for frame in frames:
                    if frame.is_motion_frame():
                        motion = frame.as_motion_frame()
                        data = motion.get_motion_data()
                        stream_name = 'accel' if frame.get_profile().stream_type() == rs.stream.accel else 'gyro'
                        result[stream_name] = {
                            'data': np.array([data.x, data.y, data.z]),
                            'timestamp': motion.get_timestamp()
                        }
            
            return result
            
        except RuntimeError:
            return None
    
    def get_pointcloud(self, frames: Dict[str, Any], colored: bool = True) -> np.ndarray:
        """Generate point cloud from frames"""
        if 'depth' not in frames:
            return np.array([])
        
        pc = rs.pointcloud()
        
        if colored and 'color' in frames:
            pc.map_to(frames['color']['frame'])
        
        points = pc.calculate(frames['depth']['frame'])
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        
        if colored and 'color' in frames:
            texcoords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
            color_image = frames['color']['data']
            
            # Sample colors
            h, w = color_image.shape[:2]
            u = (texcoords[:, 0] * w).astype(int).clip(0, w-1)
            v = (texcoords[:, 1] * h).astype(int).clip(0, h-1)
            colors = color_image[v, u] / 255.0
            
            return np.hstack([vertices, colors])
        
        return vertices
    
    def project_to_3d(self, pixel: Tuple[int, int], depth: float) -> np.ndarray:
        """Project 2D pixel to 3D point"""
        if rs.stream.depth not in self.intrinsics:
            raise ValueError("Depth intrinsics not loaded")
        
        intrinsics = self.intrinsics[rs.stream.depth]
        point = rs.rs2_deproject_pixel_to_point(intrinsics, list(pixel), depth)
        return np.array(point)
    
    def project_to_2d(self, point: np.ndarray) -> Tuple[int, int]:
        """Project 3D point to 2D pixel"""
        if rs.stream.depth not in self.intrinsics:
            raise ValueError("Depth intrinsics not loaded")
        
        intrinsics = self.intrinsics[rs.stream.depth]
        pixel = rs.rs2_project_point_to_pixel(intrinsics, list(point))
        return (int(pixel[0]), int(pixel[1]))
    
    def get_device_info(self) -> Dict[str, str]:
        """Get device information"""
        device = self.profile.get_device()
        
        info = {}
        for info_type in [
            rs.camera_info.name,
            rs.camera_info.serial_number,
            rs.camera_info.firmware_version,
            rs.camera_info.usb_type_descriptor,
            rs.camera_info.product_line
        ]:
            try:
                info[str(info_type)] = device.get_info(info_type)
            except:
                pass
        
        return info
    
    def export_calibration(self, filepath: str):
        """Export calibration to JSON file"""
        calibration = {
            'intrinsics': {},
            'extrinsics': {}
        }
        
        for stream_type, intr in self.intrinsics.items():
            calibration['intrinsics'][str(stream_type)] = {
                'width': intr.width,
                'height': intr.height,
                'fx': intr.fx,
                'fy': intr.fy,
                'ppx': intr.ppx,
                'ppy': intr.ppy,
                'model': str(intr.model),
                'coeffs': list(intr.coeffs)
            }
        
        for name, extr in self.extrinsics.items():
            calibration['extrinsics'][name] = {
                'rotation': list(extr.rotation),
                'translation': list(extr.translation)
            }
        
        with open(filepath, 'w') as f:
            json.dump(calibration, f, indent=2)
    
    def disconnect(self):
        """Disconnect from device"""
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None


# Utility functions
def list_devices() -> List[Dict[str, str]]:
    """List all connected RealSense devices"""
    ctx = rs.context()
    devices = []
    
    for dev in ctx.query_devices():
        info = {}
        for info_type in [rs.camera_info.name, rs.camera_info.serial_number]:
            try:
                info[str(info_type)] = dev.get_info(info_type)
            except:
                pass
        devices.append(info)
    
    return devices


def hardware_reset(serial_number: str = None):
    """Perform hardware reset on device"""
    ctx = rs.context()
    
    for dev in ctx.query_devices():
        if serial_number is None or dev.get_info(rs.camera_info.serial_number) == serial_number:
            dev.hardware_reset()
            return True
    
    return False
```

## 🤝 Contributing to Open Source

### Contribution Workflow

```bash
# 1. Fork the repository on GitHub

# 2. Clone your fork
git clone https://github.com/YOUR_USERNAME/librealsense.git
cd librealsense

# 3. Add upstream remote
git remote add upstream https://github.com/IntelRealSense/librealsense.git

# 4. Create feature branch
git checkout -b feature/my-new-feature

# 5. Make changes and commit
git add .
git commit -m "Add new feature: description"

# 6. Push to your fork
git push origin feature/my-new-feature

# 7. Create Pull Request on GitHub
```

### Code Standards

```cpp
// C++ code style for librealsense contributions

// Use snake_case for functions and variables
void process_depth_frame(rs2::depth_frame frame);

// Use PascalCase for classes
class DepthProcessor {
public:
    // Public methods first
    void process(rs2::frame frame);
    
private:
    // Private members with underscore prefix
    float _threshold;
};

// Document public APIs
/**
 * @brief Process a depth frame with custom filtering
 * @param frame Input depth frame
 * @return Processed depth frame
 */
rs2::frame process_depth_frame(rs2::depth_frame frame);
```

## 🧪 Hands-On Exercises

### Exercise 1: Custom Filter
1. Create a bilateral filter for depth
2. Implement as CustomProcessingBlock
3. Compare with built-in filters
4. Measure performance impact

### Exercise 2: ROS2 Node
1. Create custom ROS2 node
2. Add parameter configuration
3. Implement pointcloud publishing
4. Test with RViz

### Exercise 3: Python Extension
1. Extend RealSenseDevice class
2. Add recording functionality
3. Implement playback
4. Create configuration system

### Exercise 4: Open Source Contribution
1. Find an issue to fix
2. Implement solution
3. Write tests
4. Submit pull request

## 🎯 Next Steps

Ready to continue? → [Track 4: Capstone Project](./track-4-capstone.md)
