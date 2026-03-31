# 🔧 Level 3 Troubleshooting Guide

This guide covers common issues encountered when working with Visual SLAM, sensor fusion, AI perception, and cloud robotics.

## 🗺️ Visual SLAM Issues

### SLAM Loses Tracking

**Symptoms:**
- Pose estimation jumps erratically
- Map quality degrades
- "Lost tracking" warnings

**Solutions:**

1. **Improve Feature Detection**
   ```python
   # Use more features
   orb = cv2.ORB_create(nfeatures=2000)  # Increase from default 500
   
   # Or try SIFT for better quality (slower)
   sift = cv2.SIFT_create()
   ```

2. **Slow Down Movement**
   - Move camera more slowly
   - Ensure good overlap between frames

3. **Improve Lighting**
   - Add diffuse lighting
   - Avoid direct sunlight
   - Remove flickering lights

4. **Adjust Camera Settings**
   ```python
   depth_sensor.set_option(rs.option.exposure, 8500)
   depth_sensor.set_option(rs.option.gain, 16)
   ```

### Loop Closure Not Detected

**Solutions:**

1. **Revisit Same Location**
   - Ensure visual similarity to previous visit
   - Approach from similar angle

2. **Tune Loop Closure Parameters**
   ```python
   # For RTAB-Map
   rtabmap_args = [
       '--Kp/MaxFeatures', '400',
       '--Rtabmap/LoopThr', '0.11',
       '--Rtabmap/LoopRatio', '0.9'
   ]
   ```

3. **Increase Vocabulary Size**
   ```bash
   # For ORB-SLAM2
   # Use larger vocabulary file
   ```

### Map Drift Over Time

**Solutions:**

1. **Enable IMU Fusion**
   ```python
   # Use D435i/D455/D457 with IMU
   config.enable_stream(rs.stream.accel)
   config.enable_stream(rs.stream.gyro)
   ```

2. **Add Loop Closures**
   - Create planned paths that revisit locations

3. **Use Bundle Adjustment**
   ```python
   # Enable global optimization
   slam.enable_bundle_adjustment(True)
   ```

### RTAB-Map Database Issues

**Error:** `Database error` or slow performance

**Solutions:**

```bash
# Clean database
rm ~/.ros/rtabmap.db

# Limit database size
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start --Rtabmap/MemoryThr 500"

# Use optimized settings
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--Mem/IncrementalMemory true --Mem/RehearsalSimilarity 0.6"
```

## 🔗 Sensor Fusion Issues

### IMU Data Not Available

**Check IMU Availability:**
```python
import pyrealsense2 as rs

ctx = rs.context()
for dev in ctx.query_devices():
    sensors = dev.query_sensors()
    for sensor in sensors:
        if sensor.is_motion_sensor():
            print(f"IMU found: {sensor.get_info(rs.camera_info.name)}")
```

**Solutions:**

1. **Enable IMU Streams**
   ```python
   config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
   config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
   ```

2. **Update Firmware**
   - Some older firmware versions have IMU issues
   - Update via RealSense Viewer

### Sensor Timestamps Not Synchronized

**Solutions:**

1. **Enable Hardware Sync**
   ```python
   sensor.set_option(rs.option.global_time_enabled, True)
   ```

2. **Use Timestamp Interpolation**
   ```python
   def interpolate_imu(imu_buffer, target_timestamp):
       # Find bracketing samples
       before = None
       after = None
       for sample in imu_buffer:
           if sample.timestamp <= target_timestamp:
               before = sample
           elif after is None:
               after = sample
       
       if before and after:
           alpha = (target_timestamp - before.timestamp) / (after.timestamp - before.timestamp)
           return (1 - alpha) * before.data + alpha * after.data
       return None
   ```

### Multi-Camera Calibration Fails

**Solutions:**

1. **Use Larger Checkerboard**
   - At least 9x6 squares
   - Square size > 20mm

2. **Capture More Images**
   - Minimum 15-20 image pairs
   - Various angles and distances

3. **Check Overlap**
   - Ensure cameras see same checkerboard region
   - Minimize motion blur

```python
# Manual extrinsics if calibration fails
T_cam1_to_cam2 = np.array([
    [1, 0, 0, 0.1],   # 10cm offset in X
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
```

### Kalman Filter Diverges

**Symptoms:**
- State estimates grow unbounded
- Covariance matrix becomes singular

**Solutions:**

1. **Tune Process Noise**
   ```python
   # Increase process noise for unstable systems
   Q = np.eye(state_dim) * 0.1  # Larger values = less trust in model
   ```

2. **Check Measurement Updates**
   ```python
   # Ensure measurements are valid
   if np.any(np.isnan(measurement)) or np.any(np.isinf(measurement)):
       return  # Skip bad measurement
   ```

3. **Add Covariance Bounds**
   ```python
   # Prevent covariance from becoming too small or too large
   P = np.clip(P, 1e-6, 1e6)
   ```

## 🧠 AI Perception Issues

### Model Inference Too Slow

**Benchmarking:**
```python
import time

times = []
for _ in range(100):
    start = time.time()
    result = model(input_tensor)
    times.append(time.time() - start)

print(f"Avg: {np.mean(times)*1000:.1f}ms, Std: {np.std(times)*1000:.1f}ms")
```

**Solutions:**

1. **Use TensorRT/OpenVINO**
   ```python
   # OpenVINO
   from openvino.runtime import Core
   core = Core()
   model = core.compile_model("model.xml", "GPU")
   ```

2. **Reduce Input Size**
   ```python
   input_image = cv2.resize(image, (320, 240))
   ```

3. **Enable Mixed Precision**
   ```python
   # PyTorch
   with torch.cuda.amp.autocast():
       output = model(input)
   ```

4. **Batch Processing**
   ```python
   # Process multiple frames at once
   batch = torch.stack([frame1, frame2, frame3, frame4])
   outputs = model(batch)
   ```

### Object Detection Misses Objects

**Solutions:**

1. **Lower Confidence Threshold**
   ```python
   results = model(image, conf=0.25)  # Default is often 0.5
   ```

2. **Use Depth for Filtering**
   ```python
   # Filter detections by depth range
   valid_detections = []
   for det in detections:
       depth = get_depth_at_bbox(det.bbox)
       if 0.3 < depth < 5.0:
           valid_detections.append(det)
   ```

3. **Retrain on RGB-D Data**
   - Include depth channel in training
   - Use domain-specific dataset

### Segmentation Boundaries Inaccurate

**Solutions:**

1. **Use Depth-Aware Refinement**
   ```python
   # Use depth edges to refine segmentation
   depth_edges = cv2.Canny(depth_normalized, 50, 150)
   refined_mask = cv2.bitwise_and(segmentation_mask, ~depth_edges)
   ```

2. **Apply CRF Post-Processing**
   ```python
   import pydensecrf.densecrf as dcrf
   
   d = dcrf.DenseCRF2D(width, height, num_classes)
   d.setUnaryEnergy(unary)
   d.addPairwiseGaussian(sxy=3, compat=3)
   refined = d.inference(5)
   ```

## ☁️ Cloud Robotics Issues

### High Network Latency

**Measure Latency:**
```python
import time
import requests

start = time.time()
response = requests.post(cloud_endpoint, data=data)
latency = (time.time() - start) * 1000
print(f"Round-trip latency: {latency:.1f}ms")
```

**Solutions:**

1. **Use Edge Computing**
   - Process locally when possible
   - Only send summaries to cloud

2. **Compress Data**
   ```python
   import zlib
   
   compressed = zlib.compress(data, level=6)
   # Send compressed data
   ```

3. **Use UDP for Real-Time Data**
   ```python
   import socket
   
   sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   sock.sendto(frame_data, (server_ip, port))
   ```

### ROS2 Zenoh Connection Issues

**Error:** `Unable to connect to Zenoh router`

**Solutions:**

```bash
# Start Zenoh router
zenohd --config zenoh-config.json

# Check connectivity
zenoh-test --mode peer

# Use TCP instead of UDP
# In zenoh-config.json:
{
  "connect": {
    "endpoints": ["tcp/cloud-server:7447"]
  }
}
```

### Video Stream Stuttering

**Solutions:**

1. **Adaptive Bitrate**
   ```python
   def adjust_quality(bandwidth_mbps):
       if bandwidth_mbps < 1:
           return {'resolution': (320, 240), 'quality': 50}
       elif bandwidth_mbps < 5:
           return {'resolution': (640, 480), 'quality': 70}
       else:
           return {'resolution': (1280, 720), 'quality': 85}
   ```

2. **Frame Skipping**
   ```python
   if network_congested:
       if frame_count % 2 == 0:
           continue  # Skip every other frame
   ```

3. **Use Hardware Encoding**
   ```python
   # NVIDIA GPU encoding
   import cv2
   fourcc = cv2.VideoWriter_fourcc(*'H264')
   writer = cv2.VideoWriter('output.mp4', fourcc, 30, (640, 480))
   ```

## 🐛 Debugging Tips

### Enable Verbose Logging

```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# RealSense logging
rs.log_to_console(rs.log_severity.debug)
```

### Profile Performance

```python
import cProfile
import pstats

profiler = cProfile.Profile()
profiler.enable()

# Your code here
main_loop()

profiler.disable()
stats = pstats.Stats(profiler).sort_stats('cumtime')
stats.print_stats(20)  # Top 20 functions
```

### Memory Profiling

```python
from memory_profiler import profile

@profile
def process_frame(frame):
    # Your processing code
    pass
```

## 🆘 Getting More Help

1. **Generate Debug Report**
   ```bash
   rs-enumerate-devices -s > debug_report.txt
   nvidia-smi >> debug_report.txt
   ros2 doctor --report >> debug_report.txt
   ```

2. **Check GPU Status**
   ```bash
   # NVIDIA
   nvidia-smi
   
   # Intel
   intel_gpu_top
   ```

3. **Contact Support**
   - [Discord Community](https://discord.gg/SQdtSH4J)
   - [ROS Answers](https://answers.ros.org/)
   - [GitHub Issues](https://github.com/IntelRealSense/librealsense/issues)

---

**Still stuck?** → Check the [FAQ](./faq.md) for more common questions.
