# 🔧 Level 4 Expert Troubleshooting Guide

This guide covers advanced troubleshooting for expert-level topics including humanoid robotics, OpenVINO optimization, SDK development, and capstone projects.

## 🤖 Humanoid Robotics Issues

### Multi-Camera Synchronization Problems

**Symptoms:**
- Frames from different cameras have different timestamps
- Point cloud fusion shows ghosting
- Sensor data misaligned

**Solutions:**

1. **Hardware Sync (Best)**
   ```python
   # Enable hardware sync on all cameras
   for device in ctx.query_devices():
       sensor = device.first_depth_sensor()
       if sensor.supports(rs.option.inter_cam_sync_mode):
           sensor.set_option(rs.option.inter_cam_sync_mode, 1)  # Master
           # Set others to 2 (Slave)
   ```

2. **Software Timestamp Alignment**
   ```python
   def align_frames(frames_list, max_diff_ms=33):
       """Align frames from multiple cameras by timestamp"""
       reference_ts = frames_list[0].get_timestamp()
       aligned = [frames_list[0]]
       
       for frames in frames_list[1:]:
           ts = frames.get_timestamp()
           if abs(ts - reference_ts) <= max_diff_ms:
               aligned.append(frames)
           else:
               return None  # Frames too far apart
       
       return aligned
   ```

3. **Use Global Timestamps**
   ```python
   sensor.set_option(rs.option.global_time_enabled, True)
   ```

### Balance Control Instability

**Symptoms:**
- Robot oscillates
- Falls unexpectedly
- Overreacts to disturbances

**Solutions:**

1. **Tune PID Controllers**
   ```python
   class BalancePID:
       def __init__(self):
           # Start conservative
           self.kp = 0.5  # Proportional
           self.ki = 0.01  # Integral (low to prevent windup)
           self.kd = 0.1  # Derivative (damping)
           
       def tune_for_stability(self, overshoot):
           if overshoot > 0.1:
               self.kp *= 0.9  # Reduce proportional
               self.kd *= 1.1  # Increase damping
   ```

2. **Increase Control Frequency**
   - Target 100Hz+ for balance control
   - Use separate high-priority thread

3. **Filter Sensor Noise**
   ```python
   from scipy.signal import butter, filtfilt
   
   def low_pass_filter(data, cutoff=10, fs=100):
       b, a = butter(2, cutoff / (fs / 2), btype='low')
       return filtfilt(b, a, data)
   ```

### Manipulation Grasp Failures

**Solutions:**

1. **Improve Object Pose Estimation**
   ```python
   # Use ICP for refinement
   import open3d as o3d
   
   result = o3d.pipelines.registration.registration_icp(
       source_pcd, target_pcd,
       max_correspondence_distance=0.02,
       estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
   )
   refined_pose = result.transformation
   ```

2. **Add Grasp Quality Metrics**
   ```python
   def evaluate_grasp(grasp_pose, object_pcd):
       # Check force closure
       # Check collision
       # Check reachability
       score = compute_grasp_score(grasp_pose, object_pcd)
       return score > threshold
   ```

3. **Use Compliant Control**
   - Implement force feedback
   - Use impedance control near contact

## ⚡ OpenVINO Issues

### Model Conversion Fails

**Error:** `Model conversion failed` or unsupported operations

**Solutions:**

1. **Check Supported Operations**
   ```bash
   # List supported ops
   python -c "from openvino.runtime import Core; print(Core().available_devices)"
   ```

2. **Simplify ONNX Model**
   ```python
   import onnx
   from onnxsim import simplify
   
   model = onnx.load("model.onnx")
   model_simplified, check = simplify(model)
   onnx.save(model_simplified, "model_simplified.onnx")
   ```

3. **Use Custom Operations**
   ```python
   # Define custom extension
   from openvino.runtime import Core
   
   core = Core()
   core.add_extension("custom_ops.so")
   model = core.read_model("model.xml")
   ```

### Quantization Accuracy Drop

**Symptoms:**
- INT8 model much less accurate than FP32
- Outputs contain NaN or Inf

**Solutions:**

1. **Use Representative Calibration Data**
   ```python
   # Ensure calibration data covers edge cases
   calibration_data = [
       load_normal_images(),
       load_edge_cases(),
       load_different_lighting()
   ]
   ```

2. **Try Mixed Precision**
   ```python
   from nncf import NNCFConfig
   
   config = NNCFConfig.from_dict({
       "compression": {
           "algorithm": "quantization",
           "preset": "mixed",  # Keep sensitive layers in FP32
           "ignored_scopes": [
               "{re}.*final_layer.*"  # Don't quantize output layer
           ]
       }
   })
   ```

3. **Quantization-Aware Training**
   ```python
   from nncf import create_compressed_model
   
   # Train with quantization simulation
   nncf_model, compression_ctrl = create_compressed_model(model, nncf_config)
   
   for epoch in range(num_epochs):
       train(nncf_model)
       compression_ctrl.scheduler.epoch_step()
   ```

### Inference Speed Not Improved

**Solutions:**

1. **Check Device Configuration**
   ```python
   core = Core()
   
   # For latency
   config = {"PERFORMANCE_HINT": "LATENCY"}
   
   # For throughput
   config = {"PERFORMANCE_HINT": "THROUGHPUT"}
   
   compiled_model = core.compile_model(model, "GPU", config)
   ```

2. **Use Async Inference**
   ```python
   infer_queue = AsyncInferQueue(compiled_model, num_requests=4)
   
   for frame in frames:
       infer_queue.start_async({0: frame})
   
   infer_queue.wait_all()
   ```

3. **Profile Execution**
   ```python
   # Enable profiling
   config = {"PERF_COUNT": "YES"}
   compiled_model = core.compile_model(model, "CPU", config)
   
   # Get timing info
   perf_counts = infer_request.get_profiling_info()
   for info in perf_counts:
       print(f"{info.node_name}: {info.real_time}")
   ```

## 🔧 SDK Development Issues

### Custom Processing Block Not Working

**Solutions:**

1. **Verify Frame Types**
   ```python
   def process(self, frame):
       # Check frame type
       if not frame.is_depth_frame():
           self.get_logger().warn("Expected depth frame")
           return frame
       
       depth_frame = frame.as_depth_frame()
       # Process...
   ```

2. **Handle Frame Metadata**
   ```python
   # Preserve metadata when creating new frames
   new_frame = create_frame_with_data(processed_data)
   new_frame.set_timestamp(original_frame.get_timestamp())
   new_frame.set_frame_number(original_frame.get_frame_number())
   ```

3. **Memory Management**
   ```cpp
   // C++ - Use frame allocator
   rs2::frame_queue output_queue;
   
   auto callback = [&](rs2::frame f) {
       // Process and enqueue
       output_queue.enqueue(processed_frame);
   };
   ```

### ROS2 Node Crashes

**Solutions:**

1. **Handle Exceptions**
   ```python
   def callback(self, msg):
       try:
           self.process(msg)
       except Exception as e:
           self.get_logger().error(f"Processing error: {e}")
           # Don't re-raise - keep node alive
   ```

2. **Check QoS Compatibility**
   ```python
   from rclpy.qos import QoSProfile, QoSReliabilityPolicy
   
   # Match subscriber QoS to publisher
   qos = QoSProfile(
       reliability=QoSReliabilityPolicy.BEST_EFFORT,
       depth=10
   )
   ```

3. **Debug with GDB**
   ```bash
   ros2 run --prefix 'gdb -ex run --args' my_package my_node
   ```

### Python Bindings Don't Work

**Error:** `ImportError: cannot import name 'xxx' from 'pyrealsense2'`

**Solutions:**

1. **Check Version Compatibility**
   ```python
   import pyrealsense2 as rs
   print(f"pyrealsense2 version: {rs.__version__}")
   ```

2. **Rebuild with Python Bindings**
   ```bash
   cd librealsense/build
   cmake .. -DBUILD_PYTHON_BINDINGS=true \
       -DPYTHON_EXECUTABLE=$(which python3)
   make -j$(nproc)
   sudo make install
   ```

3. **Set Python Path**
   ```bash
   export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8/site-packages
   ```

## 📦 Capstone Project Issues

### Project Won't Build

**Solutions:**

1. **Check Dependencies**
   ```bash
   # Generate dependency list
   pip freeze > requirements.txt
   
   # Install dependencies
   pip install -r requirements.txt
   ```

2. **Use Docker for Reproducibility**
   ```dockerfile
   FROM ros:humble
   
   RUN apt-get update && apt-get install -y \
       ros-humble-realsense2-camera \
       python3-opencv
   
   COPY . /app
   WORKDIR /app
   RUN pip install -r requirements.txt
   ```

3. **Clear Build Cache**
   ```bash
   rm -rf build/ install/ log/
   colcon build --cmake-clean-first
   ```

### Tests Failing in CI

**Solutions:**

1. **Mock Hardware**
   ```python
   from unittest.mock import MagicMock
   
   @pytest.fixture
   def mock_pipeline():
       pipeline = MagicMock()
       pipeline.wait_for_frames.return_value = create_mock_frames()
       return pipeline
   ```

2. **Use Recorded Data**
   ```python
   # Record test data
   config.enable_record_to_file("test_data.bag")
   
   # Use in tests
   config.enable_device_from_file("test_data.bag")
   ```

3. **Add Retry Logic**
   ```python
   @pytest.mark.flaky(reruns=3)
   def test_camera_connection():
       # Flaky test that may need retries
       pass
   ```

### Documentation Generation Fails

**Solutions:**

1. **Fix Docstring Format**
   ```python
   def function(arg1: int, arg2: str) -> dict:
       """
       Brief description.
       
       Args:
           arg1: Description of arg1
           arg2: Description of arg2
           
       Returns:
           Description of return value
           
       Raises:
           ValueError: When arg1 is negative
       """
       pass
   ```

2. **Generate with Sphinx**
   ```bash
   cd docs
   sphinx-apidoc -o source ../src
   make html
   ```

## 🐛 General Debugging

### Enable Maximum Logging

```python
import logging

# Set all loggers to DEBUG
logging.basicConfig(level=logging.DEBUG)

# RealSense logging
rs.log_to_console(rs.log_severity.debug)

# ROS2 logging
import rclpy
rclpy.logging.set_logger_level('my_node', rclpy.logging.LoggingSeverity.DEBUG)
```

### Generate Core Dumps

```bash
# Enable core dumps
ulimit -c unlimited

# Run program
./my_program

# Analyze with GDB
gdb ./my_program core
```

### Use Sanitizers

```bash
# Compile with sanitizers
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"

# Run
ASAN_OPTIONS=detect_leaks=1 ./my_program
```

## 🆘 Getting Expert Help

1. **Prepare Detailed Report**
   ```bash
   # System info
   uname -a > report.txt
   cat /etc/os-release >> report.txt
   
   # GPU info
   nvidia-smi >> report.txt 2>/dev/null || echo "No NVIDIA GPU" >> report.txt
   
   # RealSense info
   rs-enumerate-devices -s >> report.txt
   
   # Package versions
   pip freeze >> report.txt
   ```

2. **Create Minimal Reproduction**
   - Isolate the issue
   - Provide minimal code to reproduce
   - Include test data if possible

3. **Contact Support**
   - [Discord Expert Channel](https://discord.gg/SQdtSH4J)
   - [GitHub Issues](https://github.com/IntelRealSense/librealsense/issues)
   - [OpenVINO Issues](https://github.com/openvinotoolkit/openvino/issues)

---

**Need more help?** For capstone-specific questions, email capstone@realsense-university.com
