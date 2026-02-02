# Module 1: Working with Point Clouds

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Generate high-quality point clouds from RealSense data
- Apply filtering and noise reduction techniques
- Visualize point clouds in 3D using Open3D and RViz
- Align depth and color streams for accurate color mapping
- Perform basic point cloud registration and merging

## 📊 Understanding Point Clouds

### What is a Point Cloud?

A **point cloud** is a collection of 3D points in space, where each point typically contains:
- **Position**: X, Y, Z coordinates
- **Color**: RGB values (optional)
- **Additional attributes**: Normal vectors, confidence, intensity, etc.

### Point Cloud Data Structure

```python
# Example point cloud data
point_cloud = {
    'points': np.array([
        [x1, y1, z1],  # Point 1
        [x2, y2, z2],  # Point 2
        [x3, y3, z3],  # Point 3
        # ... more points
    ]),
    'colors': np.array([
        [r1, g1, b1],  # Color 1
        [r2, g2, b2],  # Color 2
        [r3, g3, b3],  # Color 3
        # ... more colors
    ]),
    'normals': np.array([
        [nx1, ny1, nz1],  # Normal 1
        [nx2, ny2, nz2],  # Normal 2
        [nx3, ny3, nz3],  # Normal 3
        # ... more normals
    ])
}
```

## 🔧 Point Cloud Generation

### Basic Point Cloud Generation

```python
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def generate_basic_point_cloud():
    """Generate a basic point cloud from RealSense camera"""
    
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
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(vertices)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            
            return pcd
            
    except Exception as e:
        print(f"❌ Error generating point cloud: {e}")
        return None
        
    finally:
        pipeline.stop()

# Generate and visualize point cloud
pcd = generate_basic_point_cloud()
if pcd:
    o3d.visualization.draw_geometries([pcd])
```

### Advanced Point Cloud Generation with Alignment

```python
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def generate_aligned_point_cloud():
    """Generate point cloud with proper depth-color alignment"""
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        
        # Create align object
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Align frames
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        
        if aligned_depth_frame and aligned_color_frame:
            # Create point cloud
            pc = rs.pointcloud()
            pc.map_to(aligned_color_frame)
            points = pc.calculate(aligned_depth_frame)
            
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
            
            return pcd
            
    except Exception as e:
        print(f"❌ Error generating aligned point cloud: {e}")
        return None
        
    finally:
        pipeline.stop()

# Generate aligned point cloud
pcd = generate_aligned_point_cloud()
if pcd:
    o3d.visualization.draw_geometries([pcd])
```

## 🔍 Point Cloud Filtering

### Statistical Outlier Removal

```python
def remove_statistical_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """Remove statistical outliers from point cloud"""
    
    # Apply statistical outlier removal
    pcd_filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    
    print(f"Original points: {len(pcd.points)}")
    print(f"Filtered points: {len(pcd_filtered.points)}")
    
    return pcd_filtered

# Apply statistical outlier removal
pcd_clean = remove_statistical_outliers(pcd)
```

### Radius Outlier Removal

```python
def remove_radius_outliers(pcd, nb_points=16, radius=0.05):
    """Remove radius outliers from point cloud"""
    
    # Apply radius outlier removal
    pcd_filtered, _ = pcd.remove_radius_outlier(
        nb_points=nb_points,
        radius=radius
    )
    
    print(f"Original points: {len(pcd.points)}")
    print(f"Filtered points: {len(pcd_filtered.points)}")
    
    return pcd_filtered

# Apply radius outlier removal
pcd_clean = remove_radius_outliers(pcd)
```

### Voxel Downsampling

```python
def downsample_point_cloud(pcd, voxel_size=0.01):
    """Downsample point cloud using voxel grid"""
    
    # Apply voxel downsampling
    pcd_downsampled = pcd.voxel_down_sample(voxel_size)
    
    print(f"Original points: {len(pcd.points)}")
    print(f"Downsampled points: {len(pcd_downsampled.points)}")
    
    return pcd_downsampled

# Apply voxel downsampling
pcd_downsampled = downsample_point_cloud(pcd, voxel_size=0.005)
```

### Complete Filtering Pipeline

```python
def filter_point_cloud_pipeline(pcd):
    """Complete point cloud filtering pipeline"""
    
    print("🔧 Starting point cloud filtering pipeline...")
    
    # Step 1: Remove statistical outliers
    pcd_filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=20,
        std_ratio=2.0
    )
    print(f"✅ Statistical outlier removal: {len(pcd_filtered.points)} points")
    
    # Step 2: Remove radius outliers
    pcd_filtered, _ = pcd_filtered.remove_radius_outlier(
        nb_points=16,
        radius=0.05
    )
    print(f"✅ Radius outlier removal: {len(pcd_filtered.points)} points")
    
    # Step 3: Voxel downsampling
    pcd_downsampled = pcd_filtered.voxel_down_sample(0.01)
    print(f"✅ Voxel downsampling: {len(pcd_downsampled.points)} points")
    
    # Step 4: Estimate normals
    pcd_downsampled.estimate_normals()
    print("✅ Normal estimation completed")
    
    return pcd_downsampled

# Apply complete filtering pipeline
pcd_processed = filter_point_cloud_pipeline(pcd)
```

## 🎨 Point Cloud Visualization

### Basic Visualization with Open3D

```python
def visualize_point_cloud(pcd, window_name="Point Cloud"):
    """Visualize point cloud with Open3D"""
    
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)
    vis.add_geometry(pcd)
    
    # Set rendering options
    render_option = vis.get_render_option()
    render_option.point_size = 2.0
    render_option.background_color = np.array([0.1, 0.1, 0.1])
    
    # Run visualizer
    vis.run()
    vis.destroy_window()

# Visualize point cloud
visualize_point_cloud(pcd_processed)
```

### Advanced Visualization with Multiple Views

```python
def visualize_multiple_point_clouds(pcds, names=None):
    """Visualize multiple point clouds side by side"""
    
    if names is None:
        names = [f"Point Cloud {i+1}" for i in range(len(pcds))]
    
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Multiple Point Clouds")
    
    # Add each point cloud
    for i, pcd in enumerate(pcds):
        vis.add_geometry(pcd)
    
    # Set rendering options
    render_option = vis.get_render_option()
    render_option.point_size = 2.0
    render_option.background_color = np.array([0.1, 0.1, 0.1])
    
    # Run visualizer
    vis.run()
    vis.destroy_window()

# Visualize original and processed point clouds
visualize_multiple_point_clouds([pcd, pcd_processed], 
                               ["Original", "Processed"])
```

### Interactive Visualization

```python
def interactive_point_cloud_visualization(pcd):
    """Interactive point cloud visualization with controls"""
    
    # Create visualizer
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Interactive Point Cloud")
    vis.add_geometry(pcd)
    
    # Set rendering options
    render_option = vis.get_render_option()
    render_option.point_size = 2.0
    render_option.background_color = np.array([0.1, 0.1, 0.1])
    
    # Run visualizer
    vis.run()
    vis.destroy_window()
    
    # Get selected points
    selected_points = vis.get_picked_points()
    print(f"Selected {len(selected_points)} points")
    
    return selected_points

# Interactive visualization
selected_points = interactive_point_cloud_visualization(pcd_processed)
```

## 🔄 Point Cloud Registration

### Basic Point Cloud Registration

```python
def register_point_clouds(source, target, threshold=0.02):
    """Register two point clouds using ICP"""
    
    # Estimate normals
    source.estimate_normals()
    target.estimate_normals()
    
    # Apply ICP registration
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
    )
    
    print(f"Registration fitness: {reg_p2p.fitness}")
    print(f"Registration RMSE: {reg_p2p.inlier_rmse}")
    
    return reg_p2p

# Example: Register two point clouds
# reg_result = register_point_clouds(pcd1, pcd2)
```

### Point Cloud Merging

```python
def merge_point_clouds(pcds):
    """Merge multiple point clouds into one"""
    
    if not pcds:
        return None
    
    # Start with first point cloud
    merged = pcds[0]
    
    # Merge with remaining point clouds
    for pcd in pcds[1:]:
        merged += pcd
    
    print(f"Merged point cloud contains {len(merged.points)} points")
    
    return merged

# Example: Merge multiple point clouds
# merged_pcd = merge_point_clouds([pcd1, pcd2, pcd3])
```

## 🧪 Hands-On Exercises

### Exercise 1: Basic Point Cloud Generation
1. **Generate a point cloud** from your RealSense camera
2. **Save it as PLY file** and load it back
3. **Count the number of points** and calculate density
4. **Visualize with different point sizes** and colors

### Exercise 2: Filtering Comparison
1. **Generate a noisy point cloud** (move camera during capture)
2. **Apply different filtering methods**:
   - Statistical outlier removal
   - Radius outlier removal
   - Voxel downsampling
3. **Compare results** and document which method works best

### Exercise 3: Point Cloud Analysis
1. **Capture point clouds** of different objects
2. **Calculate basic statistics**:
   - Bounding box dimensions
   - Center of mass
   - Point density
3. **Create a comparison report**

### Exercise 4: 3D Scene Reconstruction
1. **Capture multiple point clouds** from different angles
2. **Register and merge** the point clouds
3. **Create a complete 3D model** of a scene
4. **Export as PLY file** for use in other applications

## 📝 Quiz Questions

1. **What is the main advantage of using depth-color alignment?**
   - A) Faster processing
   - B) Better color accuracy
   - C) Smaller file size
   - D) Higher resolution

2. **Which filtering method is best for removing isolated noise points?**
   - A) Voxel downsampling
   - B) Statistical outlier removal
   - C) Radius outlier removal
   - D) Normal estimation

3. **What does ICP stand for in point cloud registration?**
   - A) Iterative Closest Point
   - B) Integrated Color Processing
   - C) Intelligent Cloud Processing
   - D) Image Color Projection

## 🎯 Next Steps

Excellent! You now understand point cloud processing fundamentals. In the next module, you'll learn how to integrate RealSense cameras with ROS2.

**Ready to continue?** → [Module 2: Using RealSense in ROS2](./module-2-ros2.md)

## 📚 Additional Resources

- [Open3D Documentation](http://www.open3d.org/docs/)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [RealSense Point Cloud Examples](https://github.com/realsenseai/librealsense/tree/master/examples/pointcloud)
- [3D Computer Vision Tutorials](https://www.cv-foundation.org/openaccess/content_cvpr_2013/papers/Rusinkiewicz_Real-Time_3D_2013_CVPR_paper.pdf)
