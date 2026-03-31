# Module 4: Cross-Platform Development

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Deploy RealSense applications on NVIDIA Jetson platforms
- Optimize RealSense for Raspberry Pi
- Implement performance and power optimization techniques
- Use cross-compilation for embedded systems
- Handle platform-specific considerations

## 🚀 NVIDIA Jetson Development

### Jetson Setup and Installation

```bash
# Install RealSense SDK on Jetson
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev

# Install Python bindings
pip3 install pyrealsense2

# Install OpenCV with CUDA support
sudo apt-get install python3-opencv
```

### Jetson Performance Optimization

```python
import pyrealsense2 as rs
import numpy as np
import cv2

class JetsonOptimizedRealSense:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Optimize for Jetson
        self.setup_jetson_config()
        
    def setup_jetson_config(self):
        """Configure RealSense for Jetson optimization"""
        # Lower resolution for better performance
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        
        # Enable hardware acceleration
        self.config.enable_device_from_file("", rs.config.SERIAL_NUMBER)
        
    def start_streaming(self):
        """Start optimized streaming"""
        try:
            self.pipeline.start(self.config)
            print("✅ Jetson-optimized streaming started")
        except Exception as e:
            print(f"❌ Error starting stream: {e}")
    
    def get_frames_optimized(self):
        """Get frames with Jetson optimizations"""
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if depth_frame and color_frame:
                # Convert to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # Apply Jetson-specific optimizations
                depth_image = self.optimize_depth_for_jetson(depth_image)
                color_image = self.optimize_color_for_jetson(color_image)
                
                return depth_image, color_image
            else:
                return None, None
                
        except Exception as e:
            print(f"❌ Error getting frames: {e}")
            return None, None
    
    def optimize_depth_for_jetson(self, depth_image):
        """Optimize depth image for Jetson processing"""
        # Apply bilateral filter for noise reduction
        depth_filtered = cv2.bilateralFilter(depth_image, 5, 50, 50)
        
        # Downsample for faster processing
        depth_downsampled = cv2.resize(depth_filtered, (320, 240))
        
        return depth_downsampled
    
    def optimize_color_for_jetson(self, color_image):
        """Optimize color image for Jetson processing"""
        # Convert to grayscale for faster processing
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # Apply histogram equalization
        equalized = cv2.equalizeHist(gray)
        
        return equalized
```

## 🍓 Raspberry Pi Development

### Raspberry Pi Setup

```bash
# Install RealSense SDK on Raspberry Pi
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev

# Install Python dependencies
pip3 install pyrealsense2 opencv-python numpy

# Enable camera interface
sudo raspi-config
# Navigate to Interface Options > Camera > Enable
```

### Raspberry Pi Optimization

```python
class RaspberryPiOptimizedRealSense:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Optimize for Raspberry Pi
        self.setup_pi_config()
        
    def setup_pi_config(self):
        """Configure RealSense for Raspberry Pi optimization"""
        # Very low resolution for Pi performance
        self.config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 10)
        self.config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 10)
        
        # Disable unnecessary features
        self.config.disable_all_streams()
        self.config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 10)
        
    def start_streaming(self):
        """Start Pi-optimized streaming"""
        try:
            self.pipeline.start(self.config)
            print("✅ Raspberry Pi-optimized streaming started")
        except Exception as e:
            print(f"❌ Error starting stream: {e}")
    
    def get_frames_pi_optimized(self):
        """Get frames with Pi optimizations"""
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if depth_frame:
                # Convert to numpy array
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Apply Pi-specific optimizations
                depth_image = self.optimize_depth_for_pi(depth_image)
                
                return depth_image
            else:
                return None
                
        except Exception as e:
            print(f"❌ Error getting frames: {e}")
            return None
    
    def optimize_depth_for_pi(self, depth_image):
        """Optimize depth image for Pi processing"""
        # Simple median filter for noise reduction
        depth_filtered = cv2.medianBlur(depth_image, 3)
        
        # Threshold to remove far objects
        depth_thresholded = np.where(depth_filtered > 2000, 0, depth_filtered)
        
        return depth_thresholded
```

## ⚡ Performance Optimization Techniques

### Memory Management

```python
import gc
import psutil

class PerformanceOptimizer:
    def __init__(self):
        self.memory_threshold = 80  # 80% memory usage threshold
        
    def check_memory_usage(self):
        """Check current memory usage"""
        memory_percent = psutil.virtual_memory().percent
        return memory_percent
    
    def optimize_memory(self):
        """Optimize memory usage"""
        if self.check_memory_usage() > self.memory_threshold:
            # Force garbage collection
            gc.collect()
            print("🧹 Memory optimized")
    
    def resize_for_performance(self, image, max_size=640):
        """Resize image for better performance"""
        height, width = image.shape[:2]
        
        if max(height, width) > max_size:
            scale = max_size / max(height, width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            
            resized = cv2.resize(image, (new_width, new_height))
            return resized
        
        return image
```

### Power Management

```python
class PowerManager:
    def __init__(self):
        self.power_mode = "balanced"  # balanced, performance, power_save
        
    def set_power_mode(self, mode):
        """Set power management mode"""
        self.power_mode = mode
        
        if mode == "power_save":
            self.enable_power_save_mode()
        elif mode == "performance":
            self.enable_performance_mode()
        else:
            self.enable_balanced_mode()
    
    def enable_power_save_mode(self):
        """Enable power saving mode"""
        # Reduce frame rate
        # Lower resolution
        # Disable unnecessary features
        print("🔋 Power save mode enabled")
    
    def enable_performance_mode(self):
        """Enable performance mode"""
        # Increase frame rate
        # Higher resolution
        # Enable all features
        print("⚡ Performance mode enabled")
    
    def enable_balanced_mode(self):
        """Enable balanced mode"""
        # Moderate settings
        print("⚖️ Balanced mode enabled")
```

## 🔧 Cross-Compilation

### Docker-based Cross-Compilation

```dockerfile
# Dockerfile for cross-compilation
FROM ubuntu:20.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev

# Install RealSense SDK
RUN git clone https://github.com/realsenseai/librealsense.git
WORKDIR librealsense
RUN mkdir build && cd build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4
RUN make install

# Install Python bindings
RUN pip3 install pyrealsense2

# Copy application code
COPY . /app
WORKDIR /app

# Build application
RUN python3 setup.py build_ext --inplace
```

### Build Script for Multiple Platforms

```bash
#!/bin/bash
# build.sh - Cross-platform build script

PLATFORMS=("linux-x86_64" "linux-aarch64" "windows-x86_64")

for platform in "${PLATFORMS[@]}"; do
    echo "Building for $platform..."
    
    case $platform in
        "linux-x86_64")
            docker run --rm -v $(pwd):/app -w /app ubuntu:20.04 \
                bash -c "apt-get update && apt-get install -y python3-pip && pip3 install pyrealsense2 && python3 setup.py build"
            ;;
        "linux-aarch64")
            docker run --rm -v $(pwd):/app -w /app --platform linux/arm64 ubuntu:20.04 \
                bash -c "apt-get update && apt-get install -y python3-pip && pip3 install pyrealsense2 && python3 setup.py build"
            ;;
        "windows-x86_64")
            # Windows build using Wine or Windows container
            echo "Windows build not implemented in this example"
            ;;
    esac
    
    echo "✅ Build completed for $platform"
done
```

## 🧪 Hands-On Exercises

### Exercise 1: Jetson Deployment
1. Set up RealSense on NVIDIA Jetson
2. Implement performance optimizations
3. Test with different resolutions
4. Monitor GPU usage

### Exercise 2: Raspberry Pi Optimization
1. Deploy RealSense on Raspberry Pi
2. Implement Pi-specific optimizations
3. Test power consumption
4. Compare performance with desktop

### Exercise 3: Cross-Platform Testing
1. Create cross-platform application
2. Test on different architectures
3. Implement platform detection
4. Optimize for each platform

### Exercise 4: Performance Benchmarking
1. Create performance benchmarks
2. Test different optimization techniques
3. Measure memory usage
4. Document performance results

## 🎯 Next Steps

Ready to continue? → [Module 5: Mini Project - Obstacle Detection](./module-5-mini-project.md)
