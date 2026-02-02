# Module 4: Your First Python Script

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Install and use the pyrealsense2 library
- Write Python scripts to capture RealSense data
- Display depth maps using OpenCV
- Save 3D point clouds to files
- Handle common programming errors

## 🐍 Setting Up Python Environment

### Installing Required Libraries

```bash
# Install pyrealsense2 (RealSense Python bindings)
pip install pyrealsense2

# Install OpenCV for image processing
pip install opencv-python

# Install NumPy for numerical operations
pip install numpy

# Install Matplotlib for plotting (optional)
pip install matplotlib

# Install Open3D for 3D visualization (optional)
pip install open3d
```

### Verify Installation

```python
# Test script to verify installations
import pyrealsense2 as rs
import cv2
import numpy as np
import matplotlib.pyplot as plt

print("✅ All libraries imported successfully!")
print(f"RealSense SDK version: {rs.__version__}")
print(f"OpenCV version: {cv2.__version__}")
print(f"NumPy version: {np.__version__}")
```

## 🔧 Basic RealSense Pipeline

### Understanding the Pipeline

The RealSense pipeline is the main interface for working with cameras:

```python
import pyrealsense2 as rs

# 1. Create pipeline
pipeline = rs.pipeline()

# 2. Create configuration
config = rs.config()

# 3. Configure streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 4. Start streaming
pipeline.start(config)

# 5. Process frames
# ... (we'll add this next)

# 6. Stop streaming
pipeline.stop()
```

### Stream Types and Formats

| Stream Type | Format | Description | Use Case |
|-------------|--------|-------------|----------|
| `rs.stream.depth` | `rs.format.z16` | 16-bit depth | Distance measurement |
| `rs.stream.color` | `rs.format.bgr8` | 8-bit BGR color | Image processing |
| `rs.stream.infrared` | `rs.format.y8` | 8-bit grayscale | Low-light vision |
| `rs.stream.accel` | `rs.format.xyz32f` | 3D acceleration | Motion detection |
| `rs.stream.gyro` | `rs.format.xyz32f` | 3D angular velocity | Orientation tracking |

## 📸 Capturing Your First Frame

### Basic Frame Capture

```python
import pyrealsense2 as rs
import numpy as np
import cv2

def capture_single_frame():
    """Capture a single frame from RealSense camera"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Get individual frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Check if frames are valid
        if not depth_frame or not color_frame:
            print("❌ Failed to get frames")
            return None, None
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        print("✅ Successfully captured frames!")
        print(f"Depth shape: {depth_image.shape}")
        print(f"Color shape: {color_image.shape}")
        
        return depth_image, color_image
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return None, None
        
    finally:
        # Stop streaming
        pipeline.stop()

# Run the function
depth, color = capture_single_frame()
```

### Continuous Frame Capture

```python
import pyrealsense2 as rs
import numpy as np
import cv2
import time

def continuous_capture(duration=10):
    """Capture frames continuously for specified duration"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < duration:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            
            # Get frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if depth_frame and color_frame:
                # Convert to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # Process frames here
                frame_count += 1
                
                # Print progress
                if frame_count % 30 == 0:  # Every second
                    print(f"Captured {frame_count} frames...")
        
        print(f"✅ Captured {frame_count} frames in {duration} seconds")
        print(f"Average FPS: {frame_count/duration:.1f}")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        pipeline.stop()

# Run continuous capture
continuous_capture(5)  # Capture for 5 seconds
```

## 🖼️ Displaying Depth Maps with OpenCV

### Basic Depth Visualization

```python
import pyrealsense2 as rs
import numpy as np
import cv2

def display_depth_map():
    """Display depth map using OpenCV"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            
            # Get frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if depth_frame and color_frame:
                # Convert to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # Normalize depth image for display
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                
                # Display images
                cv2.imshow('Color Stream', color_image)
                cv2.imshow('Depth Stream', depth_colormap)
                
                # Break on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# Run the display function
display_depth_map()
```

### Advanced Depth Visualization

```python
import pyrealsense2 as rs
import numpy as np
import cv2

def advanced_depth_display():
    """Advanced depth visualization with distance measurement"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            
            # Get frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if depth_frame and color_frame:
                # Convert to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # Create depth colormap
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                
                # Get center pixel depth
                height, width = depth_image.shape
                center_x, center_y = width // 2, height // 2
                center_depth = depth_image[center_y, center_x]
                
                # Add distance text
                cv2.putText(depth_colormap, 
                           f'Center Distance: {center_depth}mm', 
                           (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           0.7, 
                           (255, 255, 255), 
                           2)
                
                # Add crosshair
                cv2.line(depth_colormap, 
                        (center_x - 10, center_y), 
                        (center_x + 10, center_y), 
                        (255, 255, 255), 2)
                cv2.line(depth_colormap, 
                        (center_x, center_y - 10), 
                        (center_x, center_y + 10), 
                        (255, 255, 255), 2)
                
                # Display images
                cv2.imshow('Color Stream', color_image)
                cv2.imshow('Depth Stream', depth_colormap)
                
                # Break on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# Run advanced display
advanced_depth_display()
```

## 💾 Saving 3D Point Clouds

### Basic Point Cloud Generation

```python
import pyrealsense2 as rs
import numpy as np

def save_point_cloud():
    """Generate and save 3D point cloud"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Get frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if depth_frame and color_frame:
            # Create point cloud
            pc = rs.pointcloud()
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            
            # Save point cloud
            points.export_to_ply('pointcloud.ply', color_frame)
            print("✅ Point cloud saved as 'pointcloud.ply'")
            
            # Get point cloud data
            vertices = np.asanyarray(points.get_vertices())
            colors = np.asanyarray(points.get_colors())
            
            print(f"Point cloud contains {len(vertices)} points")
            print(f"Color data shape: {colors.shape}")
            
        else:
            print("❌ Failed to get frames")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        pipeline.stop()

# Run point cloud generation
save_point_cloud()
```

### Advanced Point Cloud Processing

```python
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def advanced_point_cloud():
    """Advanced point cloud processing with filtering"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Get frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if depth_frame and color_frame:
            # Create point cloud
            pc = rs.pointcloud()
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            
            # Get point cloud data
            vertices = np.asanyarray(points.get_vertices())
            colors = np.asanyarray(points.get_colors())
            
            # Filter out invalid points
            valid_mask = vertices['f2'] > 0  # Filter by depth > 0
            valid_vertices = vertices[valid_mask]
            valid_colors = colors[valid_mask]
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(valid_vertices)
            pcd.colors = o3d.utility.Vector3dVector(valid_colors)
            
            # Save point cloud
            o3d.io.write_point_cloud('filtered_pointcloud.ply', pcd)
            print("✅ Filtered point cloud saved as 'filtered_pointcloud.ply'")
            
            # Visualize point cloud
            o3d.visualization.draw_geometries([pcd])
            
        else:
            print("❌ Failed to get frames")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        pipeline.stop()

# Run advanced point cloud processing
advanced_point_cloud()
```

## 🧪 Hands-On Exercises

### Exercise 1: Basic Frame Capture
1. **Create a Python script** that captures a single frame
2. **Save both depth and color images** as PNG files
3. **Print frame information** (dimensions, data types)
4. **Test with different resolutions**

### Exercise 2: Real-time Display
1. **Create a script** that displays live depth and color streams
2. **Add distance measurement** at the center of the image
3. **Include keyboard controls** (q to quit, s to save)
4. **Add frame rate display**

### Exercise 3: Point Cloud Generation
1. **Generate point clouds** from different scenes
2. **Compare point cloud quality** with/without IR emitter
3. **Filter point clouds** to remove noise
4. **Visualize in 3D viewer**

### Exercise 4: Error Handling
1. **Add try-catch blocks** to handle camera disconnection
2. **Implement retry logic** for failed frame captures
3. **Add logging** for debugging
4. **Test error scenarios**

## 📝 Quiz Questions

1. **What is the main purpose of the RealSense pipeline?**
   - A) Image processing
   - B) Camera communication
   - C) Data storage
   - D) Network streaming

2. **Which format is used for depth data?**
   - A) BGR8
   - B) Z16
   - C) Y8
   - D) XYZ32F

3. **What does the pointcloud.map_to() function do?**
   - A) Saves the point cloud
   - B) Aligns depth and color data
   - C) Filters the point cloud
   - D) Calculates normals

## 🎯 Next Steps

Fantastic! You can now write Python scripts to interact with RealSense cameras. In the next module, you'll build a practical distance measurement application.

**Ready to continue?** → [Module 5: Mini Project - Distance Measurement](./module-5-mini-project.md)

## 📚 Additional Resources

- [pyrealsense2 Documentation](https://github.com/realsenseai/librealsense/tree/master/wrappers/python)
- [OpenCV Python Tutorials](https://opencv-python-tutroals.readthedocs.io/)
- [NumPy Documentation](https://numpy.org/doc/)
- [Open3D Python API](http://www.open3d.org/docs/release/python_api/)
