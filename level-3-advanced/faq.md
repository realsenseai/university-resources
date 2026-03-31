# ❓ Level 3 Frequently Asked Questions

## Visual SLAM Questions

### Which SLAM system should I use?

| System | Best For | Complexity | Performance |
|--------|----------|------------|-------------|
| **RTAB-Map** | RGB-D SLAM, loop closure | Medium | Good |
| **ORB-SLAM2/3** | Visual odometry, research | High | Excellent |
| **Cartographer** | 2D/3D LiDAR + camera | High | Excellent |
| **VINS-Fusion** | Visual-inertial, drones | High | Excellent |
| **OpenVSLAM** | General purpose, modular | Medium | Good |

**Recommendation:** Start with RTAB-Map for RealSense as it has excellent RGB-D support.

### How do I choose between RGB-D SLAM and Visual-Inertial SLAM?

| Approach | Pros | Cons |
|----------|------|------|
| **RGB-D SLAM** | Dense maps, simple setup | Limited range, drift |
| **Visual-Inertial** | Better accuracy, works with motion blur | Sparse maps, needs IMU |

Use Visual-Inertial (with D435i/D455/D457) when:
- Fast motion is expected
- High accuracy is critical
- Operating in challenging lighting

### How accurate is RealSense-based SLAM?

Typical accuracy with RealSense D455:
- **Position accuracy**: 1-2% of traveled distance
- **Orientation accuracy**: < 1 degree per meter
- **With loop closure**: < 0.5% drift

Factors affecting accuracy:
- Camera model (D455 > D435 > D415)
- IMU usage
- Environment texture
- Loop closure frequency

### How do I save and load SLAM maps?

```python
# RTAB-Map
# Maps are automatically saved to ~/.ros/rtabmap.db

# Load existing map
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--Mem/IncrementalMemory false" \
    database_path:="/path/to/rtabmap.db"

# Export to point cloud
ros2 run rtabmap_ros rtabmap_export_cloud \
    database_path:="/path/to/rtabmap.db"
```

## Sensor Fusion Questions

### What sensors can I fuse with RealSense?

| Sensor | Fusion Benefit | Complexity |
|--------|----------------|------------|
| **IMU** | Better orientation, reduce drift | Low |
| **LiDAR** | Long range, outdoor | Medium |
| **Wheel Encoders** | Ground truth velocity | Low |
| **GPS** | Global localization | Medium |
| **Multiple RealSense** | Wider FOV, redundancy | Medium |

### How do I synchronize multiple sensors?

1. **Hardware Sync** (best)
   ```python
   # Enable global timestamp
   sensor.set_option(rs.option.global_time_enabled, True)
   ```

2. **Software Sync**
   ```python
   from message_filters import ApproximateTimeSynchronizer
   
   sync = ApproximateTimeSynchronizer(
       [depth_sub, lidar_sub, imu_sub],
       queue_size=10,
       slop=0.05  # 50ms tolerance
   )
   sync.registerCallback(synchronized_callback)
   ```

### What Kalman filter variant should I use?

| Filter | Use Case | Complexity |
|--------|----------|------------|
| **KF** | Linear systems only | Low |
| **EKF** | Nonlinear, moderate | Medium |
| **UKF** | Highly nonlinear | Medium |
| **Particle Filter** | Multi-modal, complex | High |

**Recommendation:** EKF for most robotics applications.

### How do I calibrate IMU bias?

```python
def calibrate_imu(samples, duration=5.0):
    """Keep camera stationary during calibration"""
    accel_samples = []
    gyro_samples = []
    
    start = time.time()
    while time.time() - start < duration:
        frames = pipeline.wait_for_frames()
        accel = frames.first(rs.stream.accel).as_motion_frame().get_motion_data()
        gyro = frames.first(rs.stream.gyro).as_motion_frame().get_motion_data()
        
        accel_samples.append([accel.x, accel.y, accel.z])
        gyro_samples.append([gyro.x, gyro.y, gyro.z])
    
    accel_bias = np.mean(accel_samples, axis=0)
    gyro_bias = np.mean(gyro_samples, axis=0)
    
    # Subtract gravity from accel bias (assuming Z-up)
    accel_bias[2] -= 9.81
    
    return accel_bias, gyro_bias
```

## AI Perception Questions

### Which object detector works best with RealSense?

| Detector | Speed | Accuracy | RGB-D Support |
|----------|-------|----------|---------------|
| **YOLOv8** | Fast | High | Add depth manually |
| **Detectron2** | Medium | Very High | Good |
| **MMDetection** | Medium | Very High | Good |
| **MediaPipe** | Very Fast | Medium | Limited |

**Recommendation:** YOLOv8 for real-time, Detectron2 for accuracy.

### How do I add depth information to 2D detections?

```python
def add_depth_to_detection(detection, depth_image):
    x1, y1, x2, y2 = detection['bbox']
    
    # Get depth in bounding box center
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    
    # Use median of region for robustness
    region = depth_image[y1:y2, x1:x2]
    valid = region[region > 0]
    
    if len(valid) > 0:
        detection['depth'] = np.median(valid) / 1000.0  # meters
    else:
        detection['depth'] = None
    
    return detection
```

### How do I train a custom RGB-D model?

1. **Collect Dataset**
   ```python
   # Record synchronized RGB-D pairs
   color_path = f"data/color/{frame_id:06d}.png"
   depth_path = f"data/depth/{frame_id:06d}.png"
   cv2.imwrite(color_path, color_image)
   cv2.imwrite(depth_path, depth_image)
   ```

2. **Use 4-Channel Input**
   ```python
   # Modify model input
   class RGBD_Backbone(nn.Module):
       def __init__(self):
           super().__init__()
           self.conv1 = nn.Conv2d(4, 64, 7)  # 4 channels: RGB + D
   ```

3. **Train with Depth Augmentation**
   ```python
   def augment_depth(depth):
       # Add noise
       noise = np.random.normal(0, 0.02, depth.shape)
       depth = depth + noise
       
       # Random dropout
       mask = np.random.random(depth.shape) > 0.1
       depth = depth * mask
       
       return depth
   ```

### How do I optimize models for edge deployment?

1. **Quantization**
   ```python
   from openvino.tools import mo
   
   mo.convert_model(
       "model.onnx",
       compress_to_fp16=True,
       output_dir="./optimized"
   )
   ```

2. **Pruning**
   ```python
   import torch.nn.utils.prune as prune
   
   prune.l1_unstructured(model.conv1, name='weight', amount=0.3)
   ```

3. **Knowledge Distillation**
   - Train small model to mimic large model

## Cloud Robotics Questions

### How much bandwidth do I need for video streaming?

| Quality | Resolution | FPS | Bandwidth |
|---------|------------|-----|-----------|
| Low | 320x240 | 15 | 0.5 Mbps |
| Medium | 640x480 | 30 | 2 Mbps |
| High | 1280x720 | 30 | 5 Mbps |
| Depth Only | 640x480 | 30 | 1 Mbps |

### Should I process on edge or cloud?

| Process On Edge | Process On Cloud |
|-----------------|------------------|
| Real-time control | Complex AI models |
| Safety-critical | Large-scale analytics |
| Low latency needed | Training/learning |
| Limited connectivity | Multi-robot coordination |

### How do I handle network disconnections?

```python
class RobustConnection:
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.connected = False
        self.retry_count = 0
        self.max_retries = 5
    
    def send(self, data):
        while self.retry_count < self.max_retries:
            try:
                response = requests.post(self.endpoint, data=data, timeout=5)
                self.retry_count = 0
                return response
            except requests.exceptions.RequestException:
                self.retry_count += 1
                time.sleep(2 ** self.retry_count)  # Exponential backoff
        
        # Fall back to local processing
        return self.process_locally(data)
```

### How do I secure robot-cloud communication?

1. **Use TLS/SSL**
   ```python
   import ssl
   
   context = ssl.create_default_context()
   context.verify_mode = ssl.CERT_REQUIRED
   ```

2. **Authenticate Requests**
   ```python
   headers = {
       'Authorization': f'Bearer {api_token}',
       'X-Robot-ID': robot_id
   }
   ```

3. **Encrypt Sensitive Data**
   ```python
   from cryptography.fernet import Fernet
   
   key = Fernet.generate_key()
   f = Fernet(key)
   encrypted = f.encrypt(data)
   ```

## Performance Questions

### How do I achieve real-time performance?

1. **Profile First**
   ```python
   import cProfile
   cProfile.run('main()', 'output.prof')
   ```

2. **Optimize Bottlenecks**
   - Use GPU acceleration
   - Reduce resolution
   - Skip frames if needed

3. **Parallel Processing**
   ```python
   from concurrent.futures import ThreadPoolExecutor
   
   with ThreadPoolExecutor(max_workers=4) as executor:
       futures = [executor.submit(process, frame) for frame in frames]
   ```

### What FPS should I target?

| Application | Minimum FPS | Recommended FPS |
|-------------|-------------|-----------------|
| SLAM | 15 | 30 |
| Object Detection | 10 | 15-30 |
| Obstacle Avoidance | 20 | 30 |
| Manipulation | 10 | 15 |
| Navigation | 10 | 20 |

### How do I reduce memory usage?

1. **Process in Batches**
   ```python
   for batch in chunk_data(data, batch_size=100):
       process(batch)
       del batch
       gc.collect()
   ```

2. **Use Memory-Mapped Files**
   ```python
   import numpy as np
   
   mmap = np.memmap('data.bin', dtype='float32', mode='r', shape=(1000, 1000))
   ```

3. **Release Unused Resources**
   ```python
   del large_array
   gc.collect()
   torch.cuda.empty_cache()  # If using PyTorch
   ```

---

**Still have questions?** Check the [Troubleshooting Guide](./troubleshooting.md) or ask in our [Discord community](https://discord.gg/SQdtSH4J).
