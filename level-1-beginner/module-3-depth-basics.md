# Module 3: Depth & Color Basics

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Understand what RGB-D data is and why it's important
- Adjust camera parameters for optimal performance
- Capture and export frames in different formats
- Interpret depth maps and point clouds

## 📊 Understanding RGB-D Data

### What is RGB-D?

**RGB-D** stands for **Red-Green-Blue-Depth**. It's a data format that combines:
- **RGB**: Traditional color information (3 channels)
- **D**: Depth information (1 channel with distance values)

This combination gives you a complete 3D understanding of your environment.

### Why RGB-D Matters

Traditional cameras only capture 2D color information. RGB-D cameras provide:
- **Spatial understanding**: Know how far objects are
- **3D reconstruction**: Build 3D models of scenes
- **Object segmentation**: Separate objects from background
- **Robotic navigation**: Avoid obstacles and plan paths

## 🎨 Understanding Depth Maps

### How Depth is Represented

Depth maps are grayscale images where:
- **Pixel intensity** represents distance
- **Brighter pixels** = closer objects
- **Darker pixels** = farther objects
- **Black pixels** = no depth data (invalid/occluded)

```
Distance:    0.5m    1.0m    2.0m    5.0m
Depth Map:   ████    ████    ██      █
Color:       White   Gray    Dark    Black
```

### Depth Map Visualization

```python
import cv2
import numpy as np

# Example depth map visualization
depth_map = np.array([
    [100, 150, 200, 250],  # Closer objects (brighter)
    [80,  120, 180, 220],  # Medium distance
    [50,  80,  120, 160],  # Farther objects (darker)
    [30,  50,  80,  100]   # Very far objects
])

# Normalize for display (0-255)
depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
```

## ⚙️ Camera Parameters and Settings

### Essential Parameters

#### 1. **Exposure**
- **What it does**: Controls how long the sensor captures light
- **Low exposure**: Darker images, less motion blur
- **High exposure**: Brighter images, more motion blur
- **Range**: Typically 1-166ms

#### 2. **Gain**
- **What it does**: Amplifies the sensor signal
- **Low gain**: Less noise, darker images
- **High gain**: More noise, brighter images
- **Range**: Typically 16-248

#### 3. **IR Emitter/Laser Projector**
- **What it does**: Projects infrared pattern for depth calculation
- **On**: Better depth in low texture areas
- **Off**: Less power consumption, works in bright sunlight

#### 4. **Depth Units**
- **What it does**: Sets the scale for depth measurements
- **Common values**: 1mm, 0.1mm, 0.001mm
- **Default**: 1mm (depth values in millimeters)

### Adjusting Parameters in RealSense Viewer

1. **Open RealSense Viewer**
2. **Connect your camera**
3. **Click on the camera name** to access controls
4. **Adjust parameters** in real-time:
   - Exposure: Use slider or enter value
   - Gain: Adjust for brightness
   - Laser Power: Enable/disable IR emitter
   - Depth Units: Set measurement scale

### Parameter Optimization Tips

#### For Indoor Use
```python
# Recommended indoor settings
exposure = 85        # ms
gain = 16           # Low gain for less noise
laser_power = 150   # Enable for better depth
depth_units = 1     # mm
```

#### For Outdoor Use
```python
# Recommended outdoor settings
exposure = 1        # ms (fast to avoid sunlight)
gain = 100          # Higher gain for brightness
laser_power = 0     # Disable (sunlight interference)
depth_units = 1     # mm
```

#### For Moving Objects
```python
# Recommended for motion
exposure = 1        # ms (minimize motion blur)
gain = 200          # Compensate for low exposure
laser_power = 150   # Enable for better tracking
depth_units = 1     # mm
```

## 💾 Data Formats and Export

### Supported File Formats

#### 1. **PNG Files**
- **Color images**: Standard RGB format
- **Depth images**: 16-bit grayscale
- **Use case**: Image processing, computer vision

#### 2. **PLY Files**
- **Format**: 3D point cloud format
- **Contains**: X, Y, Z coordinates + RGB color
- **Use case**: 3D modeling, visualization

#### 3. **BAG Files**
- **Format**: RealSense proprietary format
- **Contains**: All streams + metadata + timestamps
- **Use case**: Recording, playback, analysis

### Exporting from RealSense Viewer

#### Saving Single Frames
1. **Capture frame**: Click the camera icon
2. **Choose format**: PNG for images, PLY for point clouds
3. **Select location**: Choose save directory
4. **Name files**: Use descriptive names

#### Recording BAG Files
1. **Start recording**: Click record button
2. **Choose streams**: Select which streams to record
3. **Set duration**: Record for desired time
4. **Stop recording**: Click stop button

### Programmatic Export

```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configure streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    # Wait for frames
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    # Convert to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # Save images
    cv2.imwrite('depth.png', depth_image)
    cv2.imwrite('color.png', color_image)
    
    # Create point cloud
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    
    # Save point cloud
    points.export_to_ply('pointcloud.ply', color_frame)
    
finally:
    pipeline.stop()
```

## 🔍 Understanding Point Clouds

### What is a Point Cloud?

A **point cloud** is a collection of 3D points, where each point has:
- **X, Y, Z coordinates**: Position in 3D space
- **RGB color**: Color information (optional)
- **Additional data**: Confidence, normal vectors, etc.

### Point Cloud Structure

```python
# Example point cloud data
point_cloud = {
    'points': [
        [x1, y1, z1, r1, g1, b1],  # Point 1
        [x2, y2, z2, r2, g2, b2],  # Point 2
        [x3, y3, z3, r3, g3, b3],  # Point 3
        # ... more points
    ],
    'metadata': {
        'width': 640,
        'height': 480,
        'total_points': 307200
    }
}
```

### Visualizing Point Clouds

```python
import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud('pointcloud.ply')

# Visualize
o3d.visualization.draw_geometries([pcd])
```

## 🧪 Hands-On Exercises

### Exercise 1: Parameter Tuning
1. **Open RealSense Viewer**
2. **Connect your camera**
3. **Adjust exposure** from 1ms to 166ms
4. **Observe changes** in image quality
5. **Find optimal setting** for your environment

### Exercise 2: Depth Map Analysis
1. **Capture a depth frame** with various objects
2. **Save as PNG** and analyze pixel values
3. **Identify objects** by their depth values
4. **Measure distances** using depth data

### Exercise 3: Point Cloud Export
1. **Record a 5-second BAG file**
2. **Export frames** as PLY files
3. **Load in 3D viewer** (MeshLab, CloudCompare)
4. **Explore the 3D data**

### Exercise 4: Data Quality Assessment
1. **Test different lighting conditions**
2. **Compare depth quality** with/without IR emitter
3. **Document optimal settings** for your use case

## 📝 Quiz Questions

1. **What does RGB-D stand for?**
   - A) Red-Green-Blue-Distance
   - B) Red-Green-Blue-Depth
   - C) Real-Good-Best-Depth
   - D) Range-Gradient-Brightness-Distance

2. **In a depth map, what do brighter pixels represent?**
   - A) Farther objects
   - B) Closer objects
   - C) Higher confidence
   - D) Better quality

3. **Which parameter controls motion blur?**
   - A) Gain
   - B) Exposure
   - C) Laser Power
   - D) Depth Units

## 🎯 Next Steps

Excellent! You now understand depth and color data fundamentals. In the next module, you'll write your first Python script to interact with RealSense cameras.

**Ready to continue?** → [Module 4: Your First Python Script](./module-4-python-script.md)

## 📚 Additional Resources

- [RealSense SDK Python Examples](https://github.com/realsenseai/librealsense/tree/master/wrappers/python/examples)
- [OpenCV Python Tutorials](https://opencv-python-tutroals.readthedocs.io/)
- [Open3D Documentation](http://www.open3d.org/docs/)
- [Point Cloud Data Formats](https://en.wikipedia.org/wiki/Point_cloud)
