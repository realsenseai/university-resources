# Module 2: Sensor Fusion

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Integrate RealSense cameras with LiDAR sensors
- Implement IMU fusion for improved localization
- Build multi-camera perception systems
- Synchronize data from multiple sensors
- Apply Kalman filtering for sensor fusion

## 🔗 Understanding Sensor Fusion

### What is Sensor Fusion?

**Sensor fusion** combines data from multiple sensors to achieve more accurate and reliable perception than any single sensor could provide alone. In robotics, this typically involves:

- **Complementary sensors**: Combining sensors with different strengths
- **Redundant sensors**: Using multiple similar sensors for reliability
- **Multi-modal sensors**: Integrating different sensing modalities

### Why Sensor Fusion Matters

| Sensor | Strengths | Weaknesses |
|--------|-----------|------------|
| **RealSense** | Dense depth, RGB data, compact | Limited range, affected by lighting |
| **LiDAR** | Long range, precise, lighting independent | Sparse data, expensive, no color |
| **IMU** | High frequency, orientation, acceleration | Drift over time, no position |
| **Wheel Encoders** | Ground truth velocity | Wheel slip, no absolute position |

## 🔧 RealSense + IMU Fusion

### Accessing the Built-in IMU

RealSense D435i, D455, and D457 cameras include a built-in IMU (accelerometer + gyroscope).

```python
import pyrealsense2 as rs
import numpy as np
from collections import deque
import threading
import time

class IMUDataProcessor:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable IMU streams
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
        
        # Enable depth and color
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # IMU data buffers
        self.accel_buffer = deque(maxlen=100)
        self.gyro_buffer = deque(maxlen=100)
        
        # State estimation
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        
        self.last_accel_time = None
        self.last_gyro_time = None
        
    def start(self):
        """Start the pipeline and IMU processing"""
        self.profile = self.pipeline.start(self.config)
        
    def process_accel(self, accel_frame):
        """Process accelerometer data"""
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        timestamp = accel_frame.get_timestamp() / 1000.0  # Convert to seconds
        
        accel = np.array([accel_data.x, accel_data.y, accel_data.z])
        
        self.accel_buffer.append({
            'data': accel,
            'timestamp': timestamp
        })
        
        # Update velocity and position
        if self.last_accel_time is not None:
            dt = timestamp - self.last_accel_time
            
            # Remove gravity (assuming camera is upright)
            accel_world = self.rotate_vector(accel, self.orientation)
            accel_world[2] -= 9.81  # Remove gravity
            
            # Integrate to get velocity
            self.velocity += accel_world * dt
            
            # Integrate to get position
            self.position += self.velocity * dt
            
        self.last_accel_time = timestamp
        
    def process_gyro(self, gyro_frame):
        """Process gyroscope data"""
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
        timestamp = gyro_frame.get_timestamp() / 1000.0
        
        gyro = np.array([gyro_data.x, gyro_data.y, gyro_data.z])
        
        self.gyro_buffer.append({
            'data': gyro,
            'timestamp': timestamp
        })
        
        # Update orientation
        if self.last_gyro_time is not None:
            dt = timestamp - self.last_gyro_time
            self.orientation = self.integrate_gyro(self.orientation, gyro, dt)
            
        self.last_gyro_time = timestamp
        
    def integrate_gyro(self, q, omega, dt):
        """Integrate gyroscope data to update quaternion"""
        # Quaternion derivative
        omega_quat = np.array([0, omega[0], omega[1], omega[2]])
        q_dot = 0.5 * self.quaternion_multiply(q, omega_quat)
        
        # Integrate
        q_new = q + q_dot * dt
        
        # Normalize
        return q_new / np.linalg.norm(q_new)
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def rotate_vector(self, v, q):
        """Rotate vector v by quaternion q"""
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        v_quat = np.array([0, v[0], v[1], v[2]])
        
        result = self.quaternion_multiply(
            self.quaternion_multiply(q, v_quat), q_conj
        )
        
        return result[1:4]
    
    def get_frames(self):
        """Get synchronized frames with IMU data"""
        frames = self.pipeline.wait_for_frames()
        
        # Process IMU frames
        for frame in frames:
            if frame.is_motion_frame():
                if frame.get_profile().stream_type() == rs.stream.accel:
                    self.process_accel(frame)
                elif frame.get_profile().stream_type() == rs.stream.gyro:
                    self.process_gyro(frame)
        
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        return {
            'depth': depth_frame,
            'color': color_frame,
            'orientation': self.orientation.copy(),
            'velocity': self.velocity.copy(),
            'position': self.position.copy()
        }
    
    def stop(self):
        """Stop the pipeline"""
        self.pipeline.stop()
```

### Extended Kalman Filter for IMU Fusion

```python
import numpy as np

class ExtendedKalmanFilter:
    def __init__(self):
        # State: [position (3), velocity (3), orientation (4), gyro_bias (3), accel_bias (3)]
        self.state_dim = 16
        self.x = np.zeros(self.state_dim)
        self.x[6] = 1.0  # Quaternion w component
        
        # Covariance matrix
        self.P = np.eye(self.state_dim) * 0.1
        
        # Process noise
        self.Q = np.eye(self.state_dim) * 0.01
        self.Q[0:3, 0:3] *= 0.001  # Position noise
        self.Q[3:6, 3:6] *= 0.01   # Velocity noise
        self.Q[6:10, 6:10] *= 0.001  # Orientation noise
        
        # Measurement noise
        self.R_accel = np.eye(3) * 0.1
        self.R_gyro = np.eye(3) * 0.01
        self.R_visual = np.eye(6) * 0.05
        
    def predict(self, accel, gyro, dt):
        """Predict step using IMU data"""
        # Extract state components
        pos = self.x[0:3]
        vel = self.x[3:6]
        quat = self.x[6:10]
        gyro_bias = self.x[10:13]
        accel_bias = self.x[13:16]
        
        # Correct IMU measurements for bias
        gyro_corrected = gyro - gyro_bias
        accel_corrected = accel - accel_bias
        
        # Rotate acceleration to world frame
        accel_world = self.rotate_by_quaternion(accel_corrected, quat)
        accel_world[2] -= 9.81  # Remove gravity
        
        # Update state
        self.x[0:3] = pos + vel * dt + 0.5 * accel_world * dt**2
        self.x[3:6] = vel + accel_world * dt
        self.x[6:10] = self.integrate_quaternion(quat, gyro_corrected, dt)
        
        # Jacobian of state transition
        F = self.compute_jacobian(dt, quat, accel_corrected)
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_visual(self, visual_pose):
        """Update step using visual odometry"""
        # Measurement: [position (3), orientation (3 - euler angles)]
        z = visual_pose
        
        # Predicted measurement
        h = np.zeros(6)
        h[0:3] = self.x[0:3]  # Position
        h[3:6] = self.quaternion_to_euler(self.x[6:10])  # Orientation
        
        # Innovation
        y = z - h
        
        # Measurement Jacobian
        H = np.zeros((6, self.state_dim))
        H[0:3, 0:3] = np.eye(3)  # Position
        H[3:6, 6:10] = self.euler_jacobian(self.x[6:10])  # Orientation
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_visual
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Normalize quaternion
        self.x[6:10] /= np.linalg.norm(self.x[6:10])
        
        # Update covariance
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P
        
    def rotate_by_quaternion(self, v, q):
        """Rotate vector v by quaternion q"""
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        v_quat = np.array([0, v[0], v[1], v[2]])
        
        result = self.quaternion_multiply(
            self.quaternion_multiply(q, v_quat), q_conj
        )
        return result[1:4]
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def integrate_quaternion(self, q, omega, dt):
        """Integrate angular velocity to update quaternion"""
        omega_mag = np.linalg.norm(omega)
        if omega_mag < 1e-10:
            return q
        
        delta_q = np.zeros(4)
        delta_q[0] = np.cos(omega_mag * dt / 2)
        delta_q[1:4] = omega / omega_mag * np.sin(omega_mag * dt / 2)
        
        q_new = self.quaternion_multiply(q, delta_q)
        return q_new / np.linalg.norm(q_new)
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        return np.array([roll, pitch, yaw])
    
    def compute_jacobian(self, dt, quat, accel):
        """Compute state transition Jacobian"""
        F = np.eye(self.state_dim)
        
        # Position depends on velocity
        F[0:3, 3:6] = np.eye(3) * dt
        
        return F
    
    def euler_jacobian(self, q):
        """Compute Jacobian of Euler angles w.r.t. quaternion"""
        # Simplified - full implementation would compute partial derivatives
        return np.eye(3, 4)
    
    def get_state(self):
        """Get current state estimate"""
        return {
            'position': self.x[0:3].copy(),
            'velocity': self.x[3:6].copy(),
            'orientation': self.x[6:10].copy(),
            'gyro_bias': self.x[10:13].copy(),
            'accel_bias': self.x[13:16].copy()
        }
```

## 📡 RealSense + LiDAR Fusion

### Point Cloud Registration

```python
import open3d as o3d
import numpy as np
import pyrealsense2 as rs

class LiDARRealSenseFusion:
    def __init__(self):
        # Extrinsic calibration: transformation from LiDAR to RealSense
        self.T_lidar_to_realsense = np.eye(4)
        
        # ICP parameters
        self.icp_threshold = 0.05
        self.icp_max_iterations = 50
        
    def set_extrinsics(self, R, t):
        """Set the transformation from LiDAR to RealSense frame"""
        self.T_lidar_to_realsense[:3, :3] = R
        self.T_lidar_to_realsense[:3, 3] = t
        
    def create_realsense_pointcloud(self, depth_frame, color_frame, intrinsics):
        """Create point cloud from RealSense data"""
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        
        vertices = np.asarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        colors = np.asarray(color_frame.get_data()).reshape(-1, 3) / 255.0
        
        # Filter invalid points
        valid_mask = ~np.all(vertices == 0, axis=1)
        vertices = vertices[valid_mask]
        colors = colors[valid_mask]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vertices)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd
    
    def create_lidar_pointcloud(self, lidar_points):
        """Create point cloud from LiDAR data"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(lidar_points[:, :3])
        
        # Add intensity as grayscale color if available
        if lidar_points.shape[1] > 3:
            intensity = lidar_points[:, 3]
            intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
            colors = np.column_stack([intensity_normalized] * 3)
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd
    
    def transform_lidar_to_realsense(self, lidar_pcd):
        """Transform LiDAR point cloud to RealSense frame"""
        return lidar_pcd.transform(self.T_lidar_to_realsense)
    
    def fuse_pointclouds(self, realsense_pcd, lidar_pcd, refine=True):
        """Fuse RealSense and LiDAR point clouds"""
        # Transform LiDAR to RealSense frame
        lidar_transformed = self.transform_lidar_to_realsense(lidar_pcd)
        
        if refine:
            # Refine alignment with ICP
            reg = o3d.pipelines.registration.registration_icp(
                lidar_transformed, realsense_pcd,
                self.icp_threshold,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.icp_max_iterations
                )
            )
            lidar_transformed.transform(reg.transformation)
        
        # Merge point clouds
        fused_pcd = realsense_pcd + lidar_transformed
        
        # Downsample to remove redundant points
        fused_pcd = fused_pcd.voxel_down_sample(voxel_size=0.01)
        
        return fused_pcd
    
    def calibrate_extrinsics(self, realsense_pcd, lidar_pcd):
        """Estimate extrinsic calibration from point cloud pairs"""
        # Initial alignment with FPFH features
        realsense_down = realsense_pcd.voxel_down_sample(0.02)
        lidar_down = lidar_pcd.voxel_down_sample(0.02)
        
        # Compute normals
        realsense_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        lidar_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        
        # Compute FPFH features
        realsense_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            realsense_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100)
        )
        lidar_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            lidar_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100)
        )
        
        # RANSAC registration
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            lidar_down, realsense_down,
            lidar_fpfh, realsense_fpfh,
            True, 0.05,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3,
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.05)
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
        )
        
        # Refine with ICP
        result_icp = o3d.pipelines.registration.registration_icp(
            lidar_down, realsense_down,
            0.02,
            result.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        self.T_lidar_to_realsense = result_icp.transformation
        return result_icp.transformation
```

## 📷 Multi-Camera Systems

### Synchronized Multi-Camera Setup

```python
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
from queue import Queue

class MultiCameraSystem:
    def __init__(self, serial_numbers):
        self.serial_numbers = serial_numbers
        self.pipelines = {}
        self.configs = {}
        self.profiles = {}
        
        # Synchronized frame queues
        self.frame_queues = {sn: Queue(maxsize=10) for sn in serial_numbers}
        
        # Camera extrinsics (relative to first camera)
        self.extrinsics = {serial_numbers[0]: np.eye(4)}
        
        self.running = False
        
    def configure_cameras(self, width=640, height=480, fps=30):
        """Configure all cameras"""
        ctx = rs.context()
        devices = ctx.query_devices()
        
        for sn in self.serial_numbers:
            # Find device
            device_found = False
            for dev in devices:
                if dev.get_info(rs.camera_info.serial_number) == sn:
                    device_found = True
                    break
            
            if not device_found:
                raise RuntimeError(f"Device {sn} not found")
            
            # Create pipeline and config
            pipeline = rs.pipeline()
            config = rs.config()
            
            config.enable_device(sn)
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            
            self.pipelines[sn] = pipeline
            self.configs[sn] = config
    
    def start(self):
        """Start all cameras"""
        for sn in self.serial_numbers:
            self.profiles[sn] = self.pipelines[sn].start(self.configs[sn])
        
        self.running = True
        
        # Start capture threads
        self.capture_threads = {}
        for sn in self.serial_numbers:
            thread = threading.Thread(target=self._capture_loop, args=(sn,))
            thread.daemon = True
            thread.start()
            self.capture_threads[sn] = thread
    
    def _capture_loop(self, serial_number):
        """Capture loop for a single camera"""
        pipeline = self.pipelines[serial_number]
        
        while self.running:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if depth_frame and color_frame:
                    timestamp = frames.get_timestamp()
                    
                    frame_data = {
                        'timestamp': timestamp,
                        'depth': np.asanyarray(depth_frame.get_data()),
                        'color': np.asanyarray(color_frame.get_data()),
                        'serial_number': serial_number
                    }
                    
                    # Non-blocking put
                    if not self.frame_queues[serial_number].full():
                        self.frame_queues[serial_number].put(frame_data)
                        
            except Exception as e:
                print(f"Error capturing from {serial_number}: {e}")
    
    def get_synchronized_frames(self, timeout_ms=100):
        """Get synchronized frames from all cameras"""
        frames = {}
        timestamps = {}
        
        for sn in self.serial_numbers:
            try:
                frame_data = self.frame_queues[sn].get(timeout=timeout_ms/1000.0)
                frames[sn] = frame_data
                timestamps[sn] = frame_data['timestamp']
            except:
                return None
        
        # Check synchronization (within 33ms for 30fps)
        ts_values = list(timestamps.values())
        max_diff = max(ts_values) - min(ts_values)
        
        if max_diff > 33:
            print(f"Warning: Frame sync difference {max_diff:.1f}ms")
        
        return frames
    
    def set_extrinsics(self, serial_number, transformation):
        """Set extrinsic transformation for a camera"""
        self.extrinsics[serial_number] = transformation
    
    def calibrate_stereo(self, sn1, sn2, checkerboard_size=(9, 6), square_size=0.025):
        """Calibrate extrinsics between two cameras using checkerboard"""
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Prepare object points
        objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        objp *= square_size
        
        objpoints = []
        imgpoints1 = []
        imgpoints2 = []
        
        print("Collecting calibration frames... Press 'c' to capture, 'q' to finish")
        
        while len(objpoints) < 20:
            frames = self.get_synchronized_frames()
            if frames is None:
                continue
            
            img1 = frames[sn1]['color']
            img2 = frames[sn2]['color']
            
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            
            ret1, corners1 = cv2.findChessboardCorners(gray1, checkerboard_size, None)
            ret2, corners2 = cv2.findChessboardCorners(gray2, checkerboard_size, None)
            
            # Draw corners
            vis1 = img1.copy()
            vis2 = img2.copy()
            if ret1:
                cv2.drawChessboardCorners(vis1, checkerboard_size, corners1, ret1)
            if ret2:
                cv2.drawChessboardCorners(vis2, checkerboard_size, corners2, ret2)
            
            combined = np.hstack([vis1, vis2])
            cv2.imshow('Calibration', combined)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') and ret1 and ret2:
                corners1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
                corners2 = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
                
                objpoints.append(objp)
                imgpoints1.append(corners1)
                imgpoints2.append(corners2)
                print(f"Captured {len(objpoints)} frames")
            elif key == ord('q'):
                break
        
        cv2.destroyAllWindows()
        
        if len(objpoints) < 10:
            raise RuntimeError("Not enough calibration frames")
        
        # Get camera intrinsics
        h, w = gray1.shape
        
        # Calibrate individual cameras
        ret1, mtx1, dist1, _, _ = cv2.calibrateCamera(objpoints, imgpoints1, (w, h), None, None)
        ret2, mtx2, dist2, _, _ = cv2.calibrateCamera(objpoints, imgpoints2, (w, h), None, None)
        
        # Stereo calibration
        ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            objpoints, imgpoints1, imgpoints2,
            mtx1, dist1, mtx2, dist2,
            (w, h),
            criteria=criteria,
            flags=cv2.CALIB_FIX_INTRINSIC
        )
        
        # Create transformation matrix
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = T.flatten()
        
        self.extrinsics[sn2] = transformation
        
        return transformation
    
    def fuse_pointclouds(self, frames, intrinsics):
        """Fuse point clouds from all cameras"""
        import open3d as o3d
        
        fused_pcd = o3d.geometry.PointCloud()
        
        for sn, frame_data in frames.items():
            depth = frame_data['depth']
            color = frame_data['color']
            
            # Create point cloud
            height, width = depth.shape
            fx, fy = intrinsics[sn]['fx'], intrinsics[sn]['fy']
            cx, cy = intrinsics[sn]['cx'], intrinsics[sn]['cy']
            
            points = []
            colors = []
            
            for v in range(0, height, 2):
                for u in range(0, width, 2):
                    z = depth[v, u] / 1000.0
                    if 0.1 < z < 10.0:
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append([x, y, z])
                        colors.append(color[v, u][::-1] / 255.0)
            
            if len(points) > 0:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(points))
                pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
                
                # Transform to common frame
                pcd.transform(self.extrinsics[sn])
                
                fused_pcd += pcd
        
        return fused_pcd.voxel_down_sample(0.01)
    
    def stop(self):
        """Stop all cameras"""
        self.running = False
        
        for sn in self.serial_numbers:
            self.pipelines[sn].stop()
```

## ⏱️ Data Synchronization

### Time Synchronization Framework

```python
import numpy as np
from collections import deque
import threading
import time

class SensorSynchronizer:
    def __init__(self, max_time_diff_ms=50):
        self.max_time_diff = max_time_diff_ms / 1000.0
        self.buffers = {}
        self.locks = {}
        
    def register_sensor(self, sensor_name, buffer_size=100):
        """Register a new sensor"""
        self.buffers[sensor_name] = deque(maxlen=buffer_size)
        self.locks[sensor_name] = threading.Lock()
        
    def add_data(self, sensor_name, timestamp, data):
        """Add data from a sensor"""
        with self.locks[sensor_name]:
            self.buffers[sensor_name].append({
                'timestamp': timestamp,
                'data': data
            })
    
    def get_synchronized(self, reference_timestamp):
        """Get synchronized data closest to reference timestamp"""
        synchronized = {}
        
        for sensor_name, buffer in self.buffers.items():
            with self.locks[sensor_name]:
                if len(buffer) == 0:
                    continue
                
                # Find closest sample
                closest = min(buffer, key=lambda x: abs(x['timestamp'] - reference_timestamp))
                
                time_diff = abs(closest['timestamp'] - reference_timestamp)
                if time_diff <= self.max_time_diff:
                    synchronized[sensor_name] = {
                        'data': closest['data'],
                        'time_diff': time_diff
                    }
        
        return synchronized
    
    def interpolate_data(self, sensor_name, target_timestamp):
        """Interpolate sensor data to target timestamp"""
        with self.locks[sensor_name]:
            buffer = list(self.buffers[sensor_name])
        
        if len(buffer) < 2:
            return None
        
        # Find bracketing samples
        before = None
        after = None
        
        for sample in buffer:
            if sample['timestamp'] <= target_timestamp:
                before = sample
            elif sample['timestamp'] > target_timestamp and after is None:
                after = sample
                break
        
        if before is None or after is None:
            return None
        
        # Linear interpolation
        t_before = before['timestamp']
        t_after = after['timestamp']
        
        alpha = (target_timestamp - t_before) / (t_after - t_before)
        
        # Interpolate (assumes numpy array data)
        interpolated = (1 - alpha) * before['data'] + alpha * after['data']
        
        return interpolated
```

## 🧪 Hands-On Exercises

### Exercise 1: IMU Integration
1. Enable IMU streaming on RealSense D435i/D455
2. Implement quaternion integration from gyroscope
3. Visualize orientation in real-time
4. Compare with visual odometry

### Exercise 2: Multi-Camera Calibration
1. Set up two RealSense cameras
2. Perform stereo calibration
3. Fuse point clouds from both cameras
4. Measure fusion accuracy

### Exercise 3: Kalman Filter Implementation
1. Implement Extended Kalman Filter
2. Fuse IMU and visual odometry
3. Compare fused vs. individual estimates
4. Analyze drift reduction

### Exercise 4: LiDAR Fusion
1. Register LiDAR and RealSense point clouds
2. Calibrate extrinsic transformation
3. Create fused dense+sparse point cloud
4. Evaluate coverage improvement

## 🎯 Next Steps

Ready to continue? → [Module 3: AI Perception Pipelines](./module-3-ai-perception.md)
