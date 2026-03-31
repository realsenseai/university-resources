# Module 4: Remote and Cloud Robotics

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Stream RealSense data over networks efficiently
- Integrate ROS2 with Zenoh for remote robotics
- Deploy cloud inference services for perception
- Build edge-cloud collaborative systems
- Implement distributed robotics architectures

## ☁️ Cloud Robotics Overview

### What is Cloud Robotics?

**Cloud robotics** extends robot capabilities by leveraging remote computing resources:

- **Offloaded Computation**: Complex AI inference in the cloud
- **Shared Knowledge**: Robots learn from collective experience
- **Remote Operation**: Control robots from anywhere
- **Scalable Processing**: Elastic resources for demanding tasks

### Architecture Patterns

```
┌─────────────────────────────────────────────────────────────┐
│                        Cloud Layer                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  AI Inference │  │  Data Store │  │  Fleet Mgmt │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
└───────────────────────────┬─────────────────────────────────┘
                            │
                    ┌───────▼───────┐
                    │   Edge Layer   │
                    │  (Low Latency) │
                    └───────┬───────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                       Robot Layer                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │   RealSense  │  │   RealSense  │  │   RealSense  │         │
│  │   Robot 1    │  │   Robot 2    │  │   Robot 3    │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

## 🌐 ROS2 + Zenoh Integration

### Why Zenoh?

**Zenoh** is a modern pub/sub protocol designed for robotics and IoT:

- **Low latency**: Sub-millisecond delivery
- **Scalable**: From edge to cloud
- **Flexible**: Pub/sub, query, and storage
- **ROS2 compatible**: Native DDS bridge

### Zenoh Bridge Setup

```bash
# Install Zenoh bridge for ROS2
sudo apt install ros-humble-rmw-zenoh-cpp

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Start Zenoh router
zenohd --config zenoh-config.json
```

### Zenoh Configuration

```json
{
  "mode": "router",
  "listen": {
    "endpoints": ["tcp/0.0.0.0:7447", "udp/0.0.0.0:7447"]
  },
  "connect": {
    "endpoints": []
  },
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446"
    }
  },
  "plugins": {
    "ros2dds": {
      "ros_namespace": "/",
      "ros_domain_id": 0
    }
  }
}
```

### Remote RealSense Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class RemoteRealSenseNode(Node):
    def __init__(self):
        super().__init__('remote_realsense_node')
        
        # Parameters
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('compression_quality', 80)
        self.declare_parameter('stream_depth', True)
        self.declare_parameter('stream_color', True)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.compression_quality = self.get_parameter('compression_quality').value
        
        # Publishers with namespaced topics
        namespace = f'/{self.robot_id}/camera'
        
        self.color_pub = self.create_publisher(
            CompressedImage, f'{namespace}/color/compressed', 10)
        self.depth_pub = self.create_publisher(
            CompressedImage, f'{namespace}/depth/compressed', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, f'{namespace}/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        
        # Get intrinsics for camera info
        depth_stream = self.profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        
        # Timer for streaming
        self.timer = self.create_timer(0.033, self.stream_frames)
        
        self.get_logger().info(f'Remote RealSense node started for {self.robot_id}')
    
    def compress_depth(self, depth_image):
        """Compress depth image efficiently"""
        # Normalize to 8-bit for compression
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_8bit = depth_normalized.astype(np.uint8)
        
        # Encode as PNG (lossless for depth)
        _, encoded = cv2.imencode('.png', depth_8bit)
        return encoded.tobytes()
    
    def compress_color(self, color_image):
        """Compress color image"""
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.compression_quality]
        _, encoded = cv2.imencode('.jpg', color_image, encode_param)
        return encoded.tobytes()
    
    def create_camera_info(self):
        """Create CameraInfo message"""
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = f'{self.robot_id}_camera_link'
        
        info.width = self.intrinsics.width
        info.height = self.intrinsics.height
        
        info.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        info.d = list(self.intrinsics.coeffs)
        
        return info
    
    def stream_frames(self):
        """Stream compressed frames"""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return
            
            now = self.get_clock().now().to_msg()
            
            # Compress and publish color
            color_image = np.asanyarray(color_frame.get_data())
            color_compressed = CompressedImage()
            color_compressed.header.stamp = now
            color_compressed.header.frame_id = f'{self.robot_id}_camera_link'
            color_compressed.format = 'jpeg'
            color_compressed.data = self.compress_color(color_image)
            self.color_pub.publish(color_compressed)
            
            # Compress and publish depth
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_compressed = CompressedImage()
            depth_compressed.header.stamp = now
            depth_compressed.header.frame_id = f'{self.robot_id}_camera_link'
            depth_compressed.format = 'png'
            depth_compressed.data = self.compress_depth(depth_image)
            self.depth_pub.publish(depth_compressed)
            
            # Publish camera info
            info = self.create_camera_info()
            self.info_pub.publish(info)
            
        except Exception as e:
            self.get_logger().error(f'Error streaming: {e}')
    
    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RemoteRealSenseNode()
    
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

### Cloud Subscriber Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CloudReceiverNode(Node):
    def __init__(self):
        super().__init__('cloud_receiver_node')
        
        # Subscribe to multiple robots
        self.robot_ids = ['robot_1', 'robot_2', 'robot_3']
        
        self.subscribers = {}
        self.latest_frames = {}
        
        for robot_id in self.robot_ids:
            namespace = f'/{robot_id}/camera'
            
            self.subscribers[f'{robot_id}_color'] = self.create_subscription(
                CompressedImage,
                f'{namespace}/color/compressed',
                lambda msg, rid=robot_id: self.color_callback(msg, rid),
                10
            )
            
            self.subscribers[f'{robot_id}_depth'] = self.create_subscription(
                CompressedImage,
                f'{namespace}/depth/compressed',
                lambda msg, rid=robot_id: self.depth_callback(msg, rid),
                10
            )
            
            self.latest_frames[robot_id] = {'color': None, 'depth': None}
        
        self.bridge = CvBridge()
        
        # Display timer
        self.timer = self.create_timer(0.033, self.display_frames)
        
        self.get_logger().info('Cloud receiver node started')
    
    def decompress_color(self, data):
        """Decompress JPEG color image"""
        np_arr = np.frombuffer(data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def decompress_depth(self, data):
        """Decompress PNG depth image"""
        np_arr = np.frombuffer(data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    
    def color_callback(self, msg, robot_id):
        """Handle color image"""
        try:
            color_image = self.decompress_color(msg.data)
            self.latest_frames[robot_id]['color'] = color_image
        except Exception as e:
            self.get_logger().error(f'Error decompressing color: {e}')
    
    def depth_callback(self, msg, robot_id):
        """Handle depth image"""
        try:
            depth_image = self.decompress_depth(msg.data)
            self.latest_frames[robot_id]['depth'] = depth_image
        except Exception as e:
            self.get_logger().error(f'Error decompressing depth: {e}')
    
    def display_frames(self):
        """Display frames from all robots"""
        display_images = []
        
        for robot_id in self.robot_ids:
            color = self.latest_frames[robot_id].get('color')
            depth = self.latest_frames[robot_id].get('depth')
            
            if color is not None and depth is not None:
                # Apply colormap to depth
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=1.0),
                    cv2.COLORMAP_JET
                )
                
                # Stack horizontally
                combined = np.hstack([color, depth_colormap])
                
                # Add robot label
                cv2.putText(combined, robot_id, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                display_images.append(combined)
        
        if display_images:
            # Stack all robots vertically
            full_display = np.vstack(display_images)
            cv2.imshow('Cloud Robot Monitor', full_display)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CloudReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🧠 Cloud Inference Services

### REST API for AI Inference

```python
from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.responses import JSONResponse
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import io
from PIL import Image
import base64

app = FastAPI(title="RealSense Cloud Inference API")

# Load models
object_detector = YOLO('yolov8n.pt')

@app.post("/detect/objects")
async def detect_objects(
    color_image: UploadFile = File(...),
    depth_image: UploadFile = File(None)
):
    """Detect objects in RGB image with optional depth"""
    try:
        # Read color image
        color_bytes = await color_image.read()
        color_np = np.frombuffer(color_bytes, np.uint8)
        color = cv2.imdecode(color_np, cv2.IMREAD_COLOR)
        
        # Read depth if provided
        depth = None
        if depth_image:
            depth_bytes = await depth_image.read()
            depth_np = np.frombuffer(depth_bytes, np.uint8)
            depth = cv2.imdecode(depth_np, cv2.IMREAD_UNCHANGED)
        
        # Run detection
        results = object_detector(color)
        
        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = object_detector.names[class_id]
                    
                    detection = {
                        'class_name': class_name,
                        'class_id': class_id,
                        'confidence': confidence,
                        'bbox': {
                            'x1': int(x1), 'y1': int(y1),
                            'x2': int(x2), 'y2': int(y2)
                        }
                    }
                    
                    # Add depth if available
                    if depth is not None:
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        if 0 <= center_x < depth.shape[1] and 0 <= center_y < depth.shape[0]:
                            detection['depth_mm'] = int(depth[center_y, center_x])
                    
                    detections.append(detection)
        
        return JSONResponse({
            'success': True,
            'detections': detections,
            'count': len(detections)
        })
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/segment/semantic")
async def semantic_segmentation(
    color_image: UploadFile = File(...),
    depth_image: UploadFile = File(None)
):
    """Perform semantic segmentation"""
    try:
        # Read images
        color_bytes = await color_image.read()
        color_np = np.frombuffer(color_bytes, np.uint8)
        color = cv2.imdecode(color_np, cv2.IMREAD_COLOR)
        
        # Run segmentation
        results = object_detector(color)
        
        # Get segmentation masks if available
        masks = []
        if results[0].masks is not None:
            for mask in results[0].masks.data:
                mask_np = mask.cpu().numpy()
                masks.append(mask_np.tolist())
        
        return JSONResponse({
            'success': True,
            'masks': masks,
            'num_segments': len(masks)
        })
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/pointcloud/process")
async def process_pointcloud(
    depth_image: UploadFile = File(...),
    color_image: UploadFile = File(None),
    fx: float = 525.0,
    fy: float = 525.0,
    cx: float = 320.0,
    cy: float = 240.0
):
    """Process depth image into point cloud statistics"""
    try:
        # Read depth
        depth_bytes = await depth_image.read()
        depth_np = np.frombuffer(depth_bytes, np.uint8)
        depth = cv2.imdecode(depth_np, cv2.IMREAD_UNCHANGED)
        
        # Calculate 3D points
        height, width = depth.shape
        points = []
        
        for v in range(0, height, 4):  # Subsample
            for u in range(0, width, 4):
                z = depth[v, u] / 1000.0
                if 0.1 < z < 10.0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        
        points = np.array(points)
        
        # Calculate statistics
        stats = {
            'num_points': len(points),
            'centroid': points.mean(axis=0).tolist() if len(points) > 0 else [0, 0, 0],
            'min_bounds': points.min(axis=0).tolist() if len(points) > 0 else [0, 0, 0],
            'max_bounds': points.max(axis=0).tolist() if len(points) > 0 else [0, 0, 0],
            'std_dev': points.std(axis=0).tolist() if len(points) > 0 else [0, 0, 0]
        }
        
        return JSONResponse({
            'success': True,
            'statistics': stats
        })
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='0.0.0.0', port=8000)
```

### Client for Cloud Inference

```python
import requests
import cv2
import numpy as np
import pyrealsense2 as rs

class CloudInferenceClient:
    def __init__(self, server_url):
        self.server_url = server_url
        
    def detect_objects(self, color_image, depth_image=None):
        """Send images to cloud for object detection"""
        # Encode images
        _, color_encoded = cv2.imencode('.jpg', color_image)
        files = {'color_image': ('color.jpg', color_encoded.tobytes())}
        
        if depth_image is not None:
            _, depth_encoded = cv2.imencode('.png', depth_image)
            files['depth_image'] = ('depth.png', depth_encoded.tobytes())
        
        # Send request
        response = requests.post(
            f'{self.server_url}/detect/objects',
            files=files
        )
        
        return response.json()
    
    def get_pointcloud_stats(self, depth_image, intrinsics):
        """Get point cloud statistics from cloud"""
        _, depth_encoded = cv2.imencode('.png', depth_image)
        
        files = {'depth_image': ('depth.png', depth_encoded.tobytes())}
        params = {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'cx': intrinsics.ppx,
            'cy': intrinsics.ppy
        }
        
        response = requests.post(
            f'{self.server_url}/pointcloud/process',
            files=files,
            params=params
        )
        
        return response.json()

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
    
    # Initialize client
    client = CloudInferenceClient('http://cloud-server:8000')
    
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
            
            # Call cloud inference
            result = client.detect_objects(color_image, depth_image)
            
            # Visualize results
            if result.get('success'):
                for detection in result.get('detections', []):
                    bbox = detection['bbox']
                    cv2.rectangle(
                        color_image,
                        (bbox['x1'], bbox['y1']),
                        (bbox['x2'], bbox['y2']),
                        (0, 255, 0), 2
                    )
                    
                    label = f"{detection['class_name']}"
                    if 'depth_mm' in detection:
                        label += f" {detection['depth_mm']/1000:.2f}m"
                    
                    cv2.putText(
                        color_image, label,
                        (bbox['x1'], bbox['y1'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                    )
            
            cv2.imshow('Cloud Inference', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

## 🔄 Edge-Cloud Collaboration

### Adaptive Offloading System

```python
import time
import threading
import queue
import numpy as np

class AdaptiveOffloader:
    def __init__(self, local_model, cloud_client):
        self.local_model = local_model
        self.cloud_client = cloud_client
        
        # Performance tracking
        self.local_latencies = []
        self.cloud_latencies = []
        self.max_latency_ms = 100
        
        # Offloading decision
        self.offload_threshold = 0.7  # Complexity threshold
        self.use_cloud = False
        
        # Async cloud queue
        self.cloud_queue = queue.Queue(maxsize=5)
        self.cloud_results = {}
        self.cloud_thread = threading.Thread(target=self._cloud_worker)
        self.cloud_thread.daemon = True
        self.cloud_thread.start()
        
    def _cloud_worker(self):
        """Background thread for cloud inference"""
        while True:
            try:
                frame_id, color_image, depth_image = self.cloud_queue.get(timeout=1)
                
                start_time = time.time()
                result = self.cloud_client.detect_objects(color_image, depth_image)
                latency = (time.time() - start_time) * 1000
                
                self.cloud_latencies.append(latency)
                if len(self.cloud_latencies) > 100:
                    self.cloud_latencies.pop(0)
                
                self.cloud_results[frame_id] = result
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Cloud error: {e}")
    
    def estimate_complexity(self, color_image, depth_image):
        """Estimate frame complexity"""
        # Edge density as complexity metric
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.sum(edges > 0) / edges.size
        
        # Depth variance
        valid_depth = depth_image[depth_image > 0]
        if len(valid_depth) > 0:
            depth_variance = np.std(valid_depth) / np.mean(valid_depth)
        else:
            depth_variance = 0
        
        # Combined complexity score
        complexity = 0.6 * edge_density + 0.4 * min(depth_variance, 1.0)
        
        return complexity
    
    def should_offload(self, complexity):
        """Decide whether to offload to cloud"""
        # Check cloud availability
        avg_cloud_latency = np.mean(self.cloud_latencies) if self.cloud_latencies else 1000
        
        if avg_cloud_latency > self.max_latency_ms:
            return False
        
        # Check local performance
        avg_local_latency = np.mean(self.local_latencies) if self.local_latencies else 50
        
        # Offload if complex and cloud is faster
        if complexity > self.offload_threshold and avg_cloud_latency < avg_local_latency:
            return True
        
        return False
    
    def process(self, frame_id, color_image, depth_image):
        """Process frame with adaptive offloading"""
        complexity = self.estimate_complexity(color_image, depth_image)
        
        if self.should_offload(complexity):
            # Offload to cloud (async)
            if not self.cloud_queue.full():
                self.cloud_queue.put((frame_id, color_image.copy(), depth_image.copy()))
            
            # Use cached result if available
            if frame_id - 1 in self.cloud_results:
                return self.cloud_results[frame_id - 1]
            
            # Fall back to local
            return self.process_local(color_image, depth_image)
        else:
            return self.process_local(color_image, depth_image)
    
    def process_local(self, color_image, depth_image):
        """Process locally"""
        start_time = time.time()
        result = self.local_model(color_image)
        latency = (time.time() - start_time) * 1000
        
        self.local_latencies.append(latency)
        if len(self.local_latencies) > 100:
            self.local_latencies.pop(0)
        
        return result
    
    def get_stats(self):
        """Get performance statistics"""
        return {
            'avg_local_latency': np.mean(self.local_latencies) if self.local_latencies else 0,
            'avg_cloud_latency': np.mean(self.cloud_latencies) if self.cloud_latencies else 0,
            'cloud_queue_size': self.cloud_queue.qsize(),
            'offload_rate': len(self.cloud_latencies) / (len(self.local_latencies) + len(self.cloud_latencies) + 1)
        }
```

## 🌍 Distributed Fleet Management

### Fleet Coordinator

```python
import asyncio
import json
from dataclasses import dataclass
from typing import Dict, List, Optional
import aiohttp
from datetime import datetime

@dataclass
class RobotStatus:
    robot_id: str
    last_seen: datetime
    position: List[float]
    battery_level: float
    task_status: str
    camera_status: str

class FleetCoordinator:
    def __init__(self):
        self.robots: Dict[str, RobotStatus] = {}
        self.tasks: Dict[str, dict] = {}
        self.websockets: Dict[str, aiohttp.ClientWebSocketResponse] = {}
        
    async def register_robot(self, robot_id: str, endpoint: str):
        """Register a new robot"""
        self.robots[robot_id] = RobotStatus(
            robot_id=robot_id,
            last_seen=datetime.now(),
            position=[0, 0, 0],
            battery_level=100.0,
            task_status='idle',
            camera_status='active'
        )
        
        # Connect WebSocket
        session = aiohttp.ClientSession()
        ws = await session.ws_connect(f'{endpoint}/ws')
        self.websockets[robot_id] = ws
        
        # Start listening
        asyncio.create_task(self._listen_robot(robot_id, ws))
        
    async def _listen_robot(self, robot_id: str, ws):
        """Listen for updates from robot"""
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    await self._handle_robot_update(robot_id, data)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    break
        except Exception as e:
            print(f"Error listening to {robot_id}: {e}")
        finally:
            if robot_id in self.websockets:
                del self.websockets[robot_id]
    
    async def _handle_robot_update(self, robot_id: str, data: dict):
        """Handle update from robot"""
        if robot_id in self.robots:
            robot = self.robots[robot_id]
            robot.last_seen = datetime.now()
            
            if 'position' in data:
                robot.position = data['position']
            if 'battery' in data:
                robot.battery_level = data['battery']
            if 'task_status' in data:
                robot.task_status = data['task_status']
            if 'camera_status' in data:
                robot.camera_status = data['camera_status']
    
    async def assign_task(self, robot_id: str, task: dict):
        """Assign task to robot"""
        if robot_id not in self.websockets:
            raise ValueError(f"Robot {robot_id} not connected")
        
        task_id = f"task_{len(self.tasks)}"
        self.tasks[task_id] = {
            'robot_id': robot_id,
            'task': task,
            'status': 'assigned',
            'created_at': datetime.now().isoformat()
        }
        
        # Send task to robot
        ws = self.websockets[robot_id]
        await ws.send_json({
            'type': 'task',
            'task_id': task_id,
            'task': task
        })
        
        return task_id
    
    async def broadcast_command(self, command: dict):
        """Broadcast command to all robots"""
        for robot_id, ws in self.websockets.items():
            try:
                await ws.send_json(command)
            except Exception as e:
                print(f"Error sending to {robot_id}: {e}")
    
    def get_fleet_status(self):
        """Get status of all robots"""
        return {
            'robots': [
                {
                    'robot_id': robot.robot_id,
                    'last_seen': robot.last_seen.isoformat(),
                    'position': robot.position,
                    'battery_level': robot.battery_level,
                    'task_status': robot.task_status,
                    'camera_status': robot.camera_status
                }
                for robot in self.robots.values()
            ],
            'active_tasks': len([t for t in self.tasks.values() if t['status'] == 'in_progress']),
            'connected_robots': len(self.websockets)
        }
```

## 🧪 Hands-On Exercises

### Exercise 1: Remote Streaming
1. Set up Zenoh bridge between two machines
2. Stream RealSense data over network
3. Measure latency and bandwidth
4. Optimize compression parameters

### Exercise 2: Cloud Inference
1. Deploy FastAPI inference server
2. Send RealSense frames for detection
3. Compare cloud vs local inference
4. Implement result caching

### Exercise 3: Edge-Cloud Hybrid
1. Implement adaptive offloading
2. Test with varying network conditions
3. Measure performance improvement
4. Tune offloading thresholds

### Exercise 4: Fleet Management
1. Set up multi-robot simulation
2. Implement fleet coordinator
3. Assign and monitor tasks
4. Handle robot disconnections

## 🎯 Next Steps

Ready to continue? → [Module 5: Mini Project - Autonomous Navigation](./module-5-mini-project.md)
