# 🔧 Level 2 Troubleshooting Guide

This guide covers common issues encountered when working with point clouds, ROS2, and cross-platform development.

## 📦 Point Cloud Issues

### Point Cloud is Empty or Sparse

**Symptoms:**
- `points.get_vertices()` returns empty array
- Very few points in visualization

**Solutions:**

1. **Check Depth Frame**
   ```python
   depth_frame = frames.get_depth_frame()
   if not depth_frame:
       print("No depth frame received")
   
   depth_image = np.asanyarray(depth_frame.get_data())
   valid_points = np.count_nonzero(depth_image)
   print(f"Valid depth pixels: {valid_points}")
   ```

2. **Verify Camera Range**
   ```python
   # Check for objects in valid range
   depth_sensor = profile.get_device().first_depth_sensor()
   depth_scale = depth_sensor.get_depth_scale()
   print(f"Depth scale: {depth_scale}")
   ```

3. **Remove Filtering Issues**
   ```python
   # Disable filters temporarily to debug
   # Process raw depth frame instead
   ```

### Point Cloud Visualization Crashes

**Solutions:**

1. **Reduce Point Count**
   ```python
   import open3d as o3d
   
   # Downsample before visualization
   pcd = pcd.voxel_down_sample(voxel_size=0.01)
   ```

2. **Update Open3D**
   ```bash
   pip install --upgrade open3d
   ```

3. **Use Non-Blocking Visualization**
   ```python
   vis = o3d.visualization.Visualizer()
   vis.create_window()
   vis.add_geometry(pcd)
   vis.poll_events()
   vis.update_renderer()
   ```

### Point Cloud Colors Wrong

**Solutions:**

1. **Align Frames**
   ```python
   align = rs.align(rs.stream.color)
   aligned_frames = align.process(frames)
   ```

2. **Check Color Format**
   ```python
   # RealSense uses BGR, Open3D expects RGB
   colors = color_image[:, :, ::-1] / 255.0  # BGR to RGB
   ```

## 🤖 ROS2 Issues

### realsense2_camera Package Not Found

**Error:** `Package 'realsense2_camera' not found`

**Solutions:**

```bash
# Install ROS2 RealSense package
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Or build from source
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ..
colcon build
source install/setup.bash
```

### Camera Topics Not Publishing

**Check with:**
```bash
# List all topics
ros2 topic list

# Check if camera node is running
ros2 node list

# View camera topic info
ros2 topic info /camera/depth/image_rect_raw
```

**Solutions:**

1. **Launch Camera Node**
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

2. **Check Parameters**
   ```bash
   # View all parameters
   ros2 param list /camera/camera
   
   # Enable specific streams
   ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true
   ```

### TF Frames Not Published

**Solutions:**

1. **Enable TF Publishing**
   ```bash
   ros2 launch realsense2_camera rs_launch.py publish_tf:=true
   ```

2. **Check TF Tree**
   ```bash
   ros2 run tf2_tools view_frames
   # This creates frames.pdf
   ```

3. **Verify Frame IDs**
   ```bash
   ros2 topic echo /camera/depth/image_rect_raw --field header.frame_id
   ```

### RViz Not Showing Images

**Solutions:**

1. **Check Topic Type**
   ```bash
   ros2 topic type /camera/color/image_raw
   # Should be: sensor_msgs/msg/Image
   ```

2. **Use Correct Display Type**
   - Add → By topic → Select your topic
   - Or Add → By display type → Image

3. **Check QoS Settings**
   ```bash
   # In RViz, set QoS to "Sensor Data" or "Best Effort"
   ```

### Node Crashes with Segfault

**Solutions:**

1. **Update ROS2 and Packages**
   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. **Check Memory Usage**
   ```bash
   ros2 launch realsense2_camera rs_launch.py \
       depth_module.profile:=640x480x15
   ```

3. **Run with Debug**
   ```bash
   ros2 run --prefix 'gdb -ex run --args' realsense2_camera realsense2_camera_node
   ```

## 🔄 Cross-Platform Issues

### NVIDIA Jetson Issues

#### Camera Not Detected on Jetson

```bash
# Check USB connection
lsusb | grep Intel

# Install librealsense for Jetson
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true
make -j$(nproc)
sudo make install
```

#### Low Frame Rate on Jetson

**Solutions:**

1. **Enable Max Performance Mode**
   ```bash
   sudo nvpmodel -m 0  # Max performance
   sudo jetson_clocks
   ```

2. **Reduce Resolution**
   ```python
   config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
   ```

3. **Use Hardware Acceleration**
   ```python
   # Use CUDA for image processing
   import cv2
   cv2.cuda.setDevice(0)
   ```

### Raspberry Pi Issues

#### pyrealsense2 Installation Fails

```bash
# Install dependencies
sudo apt install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config

# Build from source
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=true -DPYTHON_EXECUTABLE=/usr/bin/python3
make -j4
sudo make install
```

#### Out of Memory Errors

**Solutions:**

1. **Increase Swap Space**
   ```bash
   sudo dphys-swapfile swapoff
   sudo nano /etc/dphys-swapfile
   # Set CONF_SWAPSIZE=2048
   sudo dphys-swapfile setup
   sudo dphys-swapfile swapon
   ```

2. **Reduce Processing**
   ```python
   # Use lower resolution
   config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)
   
   # Skip frames
   if frame_count % 2 == 0:
       continue
   ```

## 📊 Performance Issues

### High Latency

**Measure Latency:**
```python
import time

start = time.time()
frames = pipeline.wait_for_frames()
capture_time = time.time() - start

start = time.time()
depth_image = np.asanyarray(frames.get_depth_frame().get_data())
process_time = time.time() - start

print(f"Capture: {capture_time*1000:.1f}ms, Process: {process_time*1000:.1f}ms")
```

**Solutions:**

1. **Use Frame Queue**
   ```python
   # Reduce queue size for lower latency
   queue = rs.frame_queue(1)  # Only keep latest frame
   ```

2. **Use Async Processing**
   ```python
   def callback(frame):
       # Process frame in background
       pass
   
   pipeline.start(config, callback)
   ```

### Memory Leak

**Symptoms:**
- Memory usage increases over time
- Application slows down

**Solutions:**

1. **Release Frames Properly**
   ```python
   # Frames are automatically released when going out of scope
   # But explicitly release large data
   del depth_image
   del color_image
   ```

2. **Use Context Manager**
   ```python
   with rs.pipeline() as pipeline:
       pipeline.start(config)
       # Process frames
   # Pipeline automatically stopped
   ```

3. **Monitor Memory**
   ```python
   import psutil
   
   process = psutil.Process()
   print(f"Memory: {process.memory_info().rss / 1024 / 1024:.1f} MB")
   ```

## 🐍 Python-Specific Issues

### NumPy Array Issues

**Error:** `ValueError: cannot reshape array`

**Solutions:**
```python
# Check array shape
print(f"Shape: {depth_image.shape}, dtype: {depth_image.dtype}")

# Correct reshaping
vertices = np.asanyarray(points.get_vertices())
vertices = vertices.view(np.float32).reshape(-1, 3)
```

### OpenCV Display Issues

**Error:** `cv2.imshow() causes crash`

**Solutions:**

1. **Install OpenCV with GUI**
   ```bash
   pip install opencv-python  # Not opencv-python-headless
   ```

2. **Check Display Environment**
   ```bash
   echo $DISPLAY  # Should show :0 or similar
   ```

3. **Use Alternative Display**
   ```python
   import matplotlib.pyplot as plt
   plt.imshow(depth_colormap)
   plt.show()
   ```

## 🌐 Network/Remote Issues

### Camera Access Over Network

**Solution using ROS2:**
```bash
# On robot (with camera)
ros2 launch realsense2_camera rs_launch.py

# On remote computer
export ROS_DOMAIN_ID=0  # Same as robot
ros2 topic list  # Should see camera topics
```

### SSH Display Issues

```bash
# Enable X forwarding
ssh -X user@robot

# Or use headless processing
export DISPLAY=:0
```

## 🆘 Getting More Help

1. **Check ROS2 Logs**
   ```bash
   ros2 run realsense2_camera realsense2_camera_node --ros-args --log-level debug
   ```

2. **Generate System Report**
   ```bash
   rs-enumerate-devices -s > system_info.txt
   ros2 doctor --report >> system_info.txt
   ```

3. **Contact Support**
   - [Discord Community](https://discord.gg/SQdtSH4J)
   - [ROS Answers](https://answers.ros.org/questions/tagged/realsense/)
   - [GitHub Issues](https://github.com/IntelRealSense/realsense-ros/issues)

---

**Still stuck?** → Check the [FAQ](./faq.md) for more common questions.
