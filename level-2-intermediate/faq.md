# ❓ Level 2 Frequently Asked Questions

## Point Cloud Questions

### What's the difference between organized and unorganized point clouds?

| Type | Description | Use Case |
|------|-------------|----------|
| **Organized** | 2D grid structure (like an image), may have invalid points | Fast neighbor search, image-like processing |
| **Unorganized** | 1D list of valid points only | General 3D processing, smaller file size |

```python
# RealSense produces organized point clouds
# Convert to unorganized by removing invalid points
valid_mask = ~np.all(vertices == 0, axis=1)
unorganized = vertices[valid_mask]
```

### How do I save point clouds to file?

```python
import open3d as o3d

# Create point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(vertices)
pcd.colors = o3d.utility.Vector3dVector(colors)

# Save in different formats
o3d.io.write_point_cloud("output.ply", pcd)  # PLY format
o3d.io.write_point_cloud("output.pcd", pcd)  # PCD format
o3d.io.write_point_cloud("output.xyz", pcd)  # XYZ format
```

### How do I filter noise from point clouds?

```python
import open3d as o3d

# Statistical outlier removal
pcd_clean, ind = pcd.remove_statistical_outlier(
    nb_neighbors=20,
    std_ratio=2.0
)

# Radius outlier removal
pcd_clean, ind = pcd.remove_radius_outlier(
    nb_points=16,
    radius=0.05
)
```

### How do I downsample a point cloud?

```python
# Voxel downsampling (best for uniform reduction)
pcd_down = pcd.voxel_down_sample(voxel_size=0.01)  # 1cm voxels

# Uniform downsampling (every Nth point)
pcd_down = pcd.uniform_down_sample(every_k_points=5)
```

### How do I compute point cloud normals?

```python
# Estimate normals
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30
    )
)

# Orient normals consistently
pcd.orient_normals_consistent_tangent_plane(k=15)
```

## ROS2 Questions

### What ROS2 topics does the RealSense node publish?

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | Color image |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | Depth image |
| `/camera/depth/color/points` | sensor_msgs/PointCloud2 | Colored point cloud |
| `/camera/imu` | sensor_msgs/Imu | IMU data (if enabled) |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Camera calibration |

### How do I change RealSense ROS2 parameters?

```bash
# At launch time
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x30 \
    enable_color:=true \
    pointcloud.enable:=true

# Runtime parameter change
ros2 param set /camera/camera depth_module.enable_auto_exposure true
```

### How do I create a custom ROS2 node that uses RealSense data?

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MyRealSenseNode(Node):
    def __init__(self):
        super().__init__('my_realsense_node')
        self.bridge = CvBridge()
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
    
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        # Process depth_image

def main():
    rclpy.init()
    node = MyRealSenseNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### How do I record and playback ROS2 data?

```bash
# Record all camera topics
ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw

# Playback
ros2 bag play rosbag2_2024_01_01-00_00_00

# Record specific duration
ros2 bag record -d 30 /camera/depth/image_rect_raw  # 30 seconds
```

### How do I visualize RealSense data in RViz2?

1. Launch camera: `ros2 launch realsense2_camera rs_launch.py`
2. Open RViz2: `rviz2`
3. Add displays:
   - Add → By topic → /camera/color/image_raw → Image
   - Add → By topic → /camera/depth/color/points → PointCloud2
4. Set Fixed Frame to `camera_link`

## Depth Application Questions

### How do I detect obstacles in front of the camera?

```python
def detect_obstacles(depth_image, min_dist=0.3, max_dist=2.0):
    """Detect obstacles in specified range"""
    # Convert to meters
    depth_meters = depth_image.astype(float) / 1000.0
    
    # Find obstacles in range
    obstacles = (depth_meters > min_dist) & (depth_meters < max_dist)
    
    # Calculate obstacle statistics
    if np.any(obstacles):
        min_obstacle_dist = np.min(depth_meters[obstacles])
        obstacle_percentage = np.sum(obstacles) / obstacles.size * 100
        return min_obstacle_dist, obstacle_percentage
    
    return None, 0
```

### How do I segment the background?

```python
def segment_background(depth_image, threshold=2.0):
    """Remove background beyond threshold distance"""
    depth_meters = depth_image.astype(float) / 1000.0
    
    # Create foreground mask
    foreground_mask = (depth_meters > 0) & (depth_meters < threshold)
    
    return foreground_mask
```

### How do I track objects using depth?

```python
import cv2

def track_objects_depth(depth_image, prev_depth=None):
    """Simple object tracking using depth changes"""
    if prev_depth is None:
        return None
    
    # Calculate depth difference
    diff = cv2.absdiff(depth_image, prev_depth)
    
    # Threshold to find moving objects
    _, motion_mask = cv2.threshold(diff, 500, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(
        motion_mask.astype(np.uint8),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    
    return contours
```

### How do I implement gesture recognition?

```python
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

def recognize_gesture(color_image, depth_image):
    """Recognize hand gestures with depth information"""
    rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Get wrist position
            wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            x, y = int(wrist.x * 640), int(wrist.y * 480)
            
            # Get depth at wrist
            if 0 <= x < 640 and 0 <= y < 480:
                depth = depth_image[y, x] / 1000.0
                return {'position': (x, y), 'depth': depth}
    
    return None
```

## Cross-Platform Questions

### What are the minimum specs for different platforms?

| Platform | RAM | CPU | Notes |
|----------|-----|-----|-------|
| **Desktop** | 8GB | Modern quad-core CPU | Full performance |
| **Jetson Nano** | 4GB | ARM Cortex-A57 | Reduce resolution |
| **Jetson Xavier** | 16GB | ARM v8.2 | Good performance |
| **Raspberry Pi 4** | 4GB | ARM Cortex-A72 | Limited, reduce resolution |

### How do I optimize for embedded systems?

1. **Reduce Resolution**
   ```python
   config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)
   ```

2. **Skip Frames**
   ```python
   if frame_count % 2 == 0:  # Process every other frame
       continue
   ```

3. **Use Hardware Acceleration**
   - Jetson: Use CUDA/TensorRT
   - CPU: Use OpenVINO

4. **Minimize Memory Copies**
   ```python
   # Use numpy views instead of copies
   depth_view = np.asarray(depth_frame.get_data())  # View, not copy
   ```

### How do I build librealsense from source?

```bash
# Clone repository
git clone https://github.com/realsenseai/librealsense.git
cd librealsense

# Install dependencies (Ubuntu)
sudo apt install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
make -j$(nproc)
sudo make install
```

## Performance Questions

### How do I benchmark my application?

```python
import time

class FPSCounter:
    def __init__(self, window_size=30):
        self.times = []
        self.window_size = window_size
    
    def update(self):
        self.times.append(time.time())
        if len(self.times) > self.window_size:
            self.times.pop(0)
    
    def get_fps(self):
        if len(self.times) < 2:
            return 0
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])

fps = FPSCounter()
while True:
    frames = pipeline.wait_for_frames()
    fps.update()
    print(f"FPS: {fps.get_fps():.1f}")
```

### What's the maximum frame rate I can achieve?

| Resolution | Depth FPS | Color FPS |
|------------|-----------|-----------|
| 1280x720 | 30 | 30 |
| 848x480 | 60 | 60 |
| 640x480 | 90 | 60 |
| 424x240 | 90 | 90 |

### How do I reduce CPU usage?

1. Use hardware decoding when available
2. Process in separate thread
3. Use efficient data structures
4. Minimize copies and conversions

```python
import threading
import queue

frame_queue = queue.Queue(maxsize=2)

def capture_thread():
    while running:
        frames = pipeline.wait_for_frames()
        if not frame_queue.full():
            frame_queue.put(frames)

def process_thread():
    while running:
        frames = frame_queue.get()
        # Process frames
```

## Integration Questions

### Can I use RealSense with OpenCV DNN?

```python
import cv2

net = cv2.dnn.readNet("model.weights", "model.cfg")

def detect_with_dnn(color_image, depth_image):
    blob = cv2.dnn.blobFromImage(color_image, 1/255.0, (416, 416))
    net.setInput(blob)
    outputs = net.forward(net.getUnconnectedOutLayersNames())
    
    # Add depth information to detections
    for detection in outputs:
        # Get center of detection
        x, y = int(detection[0] * 640), int(detection[1] * 480)
        depth = depth_image[y, x] / 1000.0
        detection['depth'] = depth
    
    return outputs
```

### Can I use RealSense with TensorFlow/PyTorch?

Yes! Convert frames to tensors:

```python
import torch

depth_image = np.asanyarray(depth_frame.get_data())
depth_tensor = torch.from_numpy(depth_image).float() / 1000.0
depth_tensor = depth_tensor.unsqueeze(0).unsqueeze(0)  # Add batch and channel dims
```

---

**Still have questions?** Check the [Troubleshooting Guide](./troubleshooting.md) or ask in our [Discord community](https://discord.gg/SQdtSH4J).
