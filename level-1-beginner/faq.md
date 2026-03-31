# ❓ Level 1 Frequently Asked Questions

## General Questions

### What is a RealSense camera?

A RealSense camera is a stereo depth camera that uses two infrared cameras and an IR projector to calculate the distance to objects in the scene. It also includes an RGB color camera for regular video capture.

### Which RealSense camera should I buy?

| Camera | Best For | Price Range |
|--------|----------|-------------|
| **D415** | Close-range, indoor, detailed scanning | $$ |
| **D435** | General purpose, most popular | $$ |
| **D455** | Long-range, outdoor, robotics | $$$ |
| **D457** | AI applications, high accuracy | $$$ |
| **D555** | AI/Robotics, ROS/Holoscan built-in | $$$$ |

**Recommendation for beginners:** D435 offers the best balance of features and price.

### What's the difference between stereo depth and LiDAR?

| Feature | Stereo Depth (RealSense) | LiDAR |
|---------|--------------------------|-------|
| **Range** | 0.1m - 20m | Up to 200m+ |
| **Resolution** | High (up to 1280x720) | Low (sparse points) |
| **Color** | Yes (RGB camera) | No |
| **Cost** | $100-400 | $1000-10000+ |
| **Best For** | Indoor, robotics, AR | Outdoor, automotive |

### Can I use RealSense outdoors?

Yes, but with limitations:
- **D455** and **D457** are best for outdoor use
- Avoid direct sunlight on the camera
- Performance degrades in bright conditions
- IR projector helps in low-light conditions

## Hardware Questions

### What USB port do I need?

**USB 3.0 (blue port) is required.** The camera will not function properly with USB 2.0. Check your computer's specifications if you're unsure.

### Can I use a USB extension cable?

Active USB 3.0 extension cables (with signal boosters) work best. Passive cables may cause connection issues beyond 2 meters.

### How do I mount the camera?

RealSense cameras have a standard 1/4"-20 tripod mount on the bottom. Options include:
- Standard camera tripods
- Articulated arms
- Custom 3D printed mounts
- Robot mounting brackets

### What's the power consumption?

| Camera | Typical Power | Max Power |
|--------|---------------|-----------|
| D415 | 700mA @ 5V | 1.5A |
| D435 | 700mA @ 5V | 1.5A |
| D455 | 1.5A @ 5V | 2A |
| D457 | 2A @ 5V | 2.5A |

## Software Questions

### What operating systems are supported?

- **Windows 10/11** (recommended)
- **Ubuntu 18.04, 20.04, 22.04** (recommended)
- **macOS 10.14+** (limited support)
- **Raspberry Pi OS** (ARM support)
- **NVIDIA Jetson** (full support)

### Do I need to install drivers?

- **Windows:** Drivers are included with the SDK
- **Linux:** DKMS kernel modules are installed with SDK
- **macOS:** No separate drivers needed

### What programming languages are supported?

| Language | Library | Notes |
|----------|---------|-------|
| **Python** | pyrealsense2 | Most beginner-friendly |
| **C++** | librealsense2 | Best performance |
| **C#** | RealSense.NET | .NET support |
| **JavaScript** | node-librealsense | Node.js binding |
| **Rust** | realsense-rust | Community maintained |

### Can I record and playback data?

Yes! Use BAG files:

```python
import pyrealsense2 as rs

# Record
config = rs.config()
config.enable_record_to_file('recording.bag')
pipeline.start(config)

# Playback
config = rs.config()
config.enable_device_from_file('recording.bag')
pipeline.start(config)
```

## Depth Data Questions

### What units is depth data in?

Depth data is in **millimeters** by default (16-bit unsigned integer). To convert to meters:

```python
depth_meters = depth_mm / 1000.0
```

### Why are some pixels showing 0 depth?

Zero depth indicates no valid measurement. Causes include:
- Object too close or too far
- Reflective surfaces (mirrors, glass)
- Very dark or very bright surfaces
- Occlusion between IR cameras
- Camera settings need adjustment

### How accurate is the depth measurement?

Accuracy varies by camera and distance:

| Camera | At 1m | At 4m |
|--------|-------|-------|
| D415 | ±2mm | ±14mm |
| D435 | ±2mm | ±14mm |
| D455 | ±2mm | ±14mm |

### What's the depth resolution?

| Camera | Max Depth Resolution | Max Color Resolution |
|--------|---------------------|---------------------|
| D415 | 1280x720 @ 90fps | 1920x1080 @ 30fps |
| D435 | 1280x720 @ 90fps | 1920x1080 @ 30fps |
| D455 | 1280x720 @ 90fps | 1280x800 @ 30fps |

## Python Questions

### How do I get the distance to a specific pixel?

```python
import pyrealsense2 as rs

pipeline = rs.pipeline()
pipeline.start()

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()

# Get distance at pixel (320, 240)
distance = depth_frame.get_distance(320, 240)
print(f"Distance: {distance:.2f} meters")
```

### How do I convert a pixel to 3D coordinates?

```python
import pyrealsense2 as rs

# Get intrinsics
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)
intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

# Convert pixel to 3D point
pixel = [320, 240]
depth = depth_frame.get_distance(pixel[0], pixel[1])
point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, pixel, depth)
print(f"3D Point: X={point_3d[0]:.3f}, Y={point_3d[1]:.3f}, Z={point_3d[2]:.3f}")
```

### How do I save a depth image?

```python
import numpy as np
import cv2

depth_image = np.asanyarray(depth_frame.get_data())

# Save as PNG (16-bit)
cv2.imwrite('depth.png', depth_image)

# Save as colorized image
depth_colormap = cv2.applyColorMap(
    cv2.convertScaleAbs(depth_image, alpha=0.03),
    cv2.COLORMAP_JET
)
cv2.imwrite('depth_color.png', depth_colormap)
```

### How do I create a point cloud?

```python
import pyrealsense2 as rs
import numpy as np

pc = rs.pointcloud()
points = pc.calculate(depth_frame)

# Get vertices as numpy array
vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
```

## Performance Questions

### How do I improve frame rate?

1. Reduce resolution: `config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 90)`
2. Disable unused streams
3. Use decimation filter to reduce data
4. Process frames asynchronously

### How do I reduce latency?

1. Use smaller frame queues
2. Process frames immediately
3. Use async callbacks instead of `wait_for_frames()`

```python
def frame_callback(frame):
    # Process frame immediately
    pass

pipeline.start(config, frame_callback)
```

### Can I use multiple cameras?

Yes! Each camera has a unique serial number:

```python
# Get all connected cameras
ctx = rs.context()
for device in ctx.query_devices():
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"Found camera: {serial}")

# Start specific camera
config.enable_device('123456789')
```

## Troubleshooting Questions

### Where can I get help?

1. **This FAQ and Troubleshooting Guide**
2. **Discord Community**: [discord.gg/SQdtSH4J](https://discord.gg/SQdtSH4J)
3. **GitHub Issues**: [realsenseai/librealsense](https://github.com/realsenseai/librealsense/issues)
4. **RealSense Community**: [community.realsenseai.com](https://community.realsenseai.com)

### How do I update firmware?

1. Open RealSense Viewer
2. Connect your camera
3. Click on the camera info panel
4. Click "Update Firmware" if available
5. Follow the on-screen instructions

### How do I reset camera settings?

```python
import pyrealsense2 as rs

ctx = rs.context()
for device in ctx.query_devices():
    device.hardware_reset()
```

Or in RealSense Viewer: More → Hardware Reset

---

**Still have questions?** Check the [Troubleshooting Guide](./troubleshooting.md) or ask in our [Discord community](https://discord.gg/SQdtSH4J).
