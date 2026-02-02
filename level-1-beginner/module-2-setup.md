# Module 2: Setup & Installation

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Properly connect and configure your RealSense camera
- Install the RealSense SDK 2.0 on your system
- Test your camera with the RealSense Viewer
- Troubleshoot common installation issues

## 📦 Unboxing & Hardware Setup

### What's in the Box

Your RealSense camera package typically includes:
- RealSense camera unit
- USB 3.0 cable (Type-C to Type-A)
- Mounting bracket and screws
- Quick start guide
- Warranty information

### 🔌 Connecting Your Camera

1. **Locate USB 3.0 Port**: Ensure you're using a USB 3.0 (blue) or USB 3.1+ port
   - USB 2.0 ports will not provide sufficient bandwidth
   - Look for the blue color or "SS" (SuperSpeed) marking

2. **Connect the Cable**:
   - Connect the Type-C end to your RealSense camera
   - Connect the Type-A end to your computer's USB 3.0 port

3. **Power Indicator**: The camera should show a small LED indicator when properly connected

### ⚠️ Important Notes

- **USB 3.0 Required**: RealSense cameras require USB 3.0 for full functionality
- **Cable Length**: Keep the cable under 3 meters for optimal performance
- **Power**: The camera is powered through USB (no external power needed)
- **Mounting**: Use the included bracket for stable mounting

## 💻 Software Installation

### Windows Installation

#### Method 1: RealSenseSDK Installer (Recommended)

1. **Download the SDK**:
   - Visit [ RealSense SDK Downloads](https://www.realsenseai.com/developers/)
   - Download " RealSense SDK 2.0" for Windows

2. **Run the Installer**:
   ```bash
   # Run the downloaded .exe file as Administrator
   RealSense.SDK-WIN10-2.54.1.4348.exe
   ```

3. **Follow Installation Wizard**:
   - Accept license agreement
   - Choose installation directory (default recommended)
   - Install additional components (RealSense Viewer, examples)

4. **Verify Installation**:
   ```bash
   # Open Command Prompt and check version
   realsense-viewer --version
   ```

#### Method 2: Python Package Installation

```bash
# Install pyrealsense2 for Python development
pip install pyrealsense2

# Install additional dependencies
pip install opencv-python numpy matplotlib
```

### Ubuntu/Linux Installation

https://github.com/realsenseai/librealsense/blob/master/doc/installation.md 

### macOS Installation

https://github.com/realsenseai/librealsense/blob/master/doc/installation_osx.md 

## 🧪 Testing Your Installation

### Using RealSense Viewer

The RealSense Viewer is a GUI application that lets you test and configure your camera:

1. **Launch RealSense Viewer**:
   ```bash
   # Windows
   realsense-viewer
   
   # Linux/macOS
   realsense-viewer
   ```

2. **Connect Your Camera**:
   - The camera should appear in the device list
   - Click on your camera to connect

3. **Test Streams**:
   - Enable Color stream (RGB)
   - Enable Depth stream
   - Adjust settings as needed

4. **Verify Data Quality**:
   - Check that depth data looks reasonable
   - Verify color and depth are aligned
   - Test different resolutions and frame rates

### Command Line Testing

```bash
# List connected RealSense devices
rs-enumerate-devices

# Capture a single frame
rs-save-to-disk

# Record a bag file
rs-record
```

### Python Testing

Create a simple test script:

```python
import pyrealsense2 as rs
import numpy as np

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
    
    if not depth_frame or not color_frame:
        print("Failed to get frames")
    else:
        print("✅ Camera working correctly!")
        print(f"Depth frame: {depth_frame.get_width()}x{depth_frame.get_height()}")
        print(f"Color frame: {color_frame.get_width()}x{color_frame.get_height()}")
        
finally:
    pipeline.stop()
```

## 🔧 Troubleshooting Common Issues

### Camera Not Detected

**Problem**: Camera doesn't appear in device list

**Solutions**:
1. **Check USB Connection**:
   - Ensure USB 3.0 port is being used
   - Try different USB cable
   - Test on different computer

2. **Driver Issues (Windows)**:
   ```bash
   # Uninstall and reinstall drivers
   # Use Device Manager to check for conflicts
   ```

3. **Permissions (Linux)**:
   ```bash
   # Add user to plugdev group
   sudo usermod -a -G plugdev $USER
   
   # Create udev rules
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-realsense-libusb.rules
   sudo udevadm control --reload-rules && udevadm trigger
   ```

### Poor Depth Quality

**Problem**: Depth data is noisy or inaccurate

**Solutions**:
1. **Lighting Conditions**:
   - Ensure adequate lighting
   - Avoid direct sunlight
   - Use IR emitter (if available)

2. **Surface Properties**:
   - Avoid reflective surfaces
   - Ensure surfaces have texture
   - Keep objects within depth range

3. **Camera Settings**:
   - Adjust exposure and gain
   - Enable laser projector (if available)
   - Check for obstructions

### Performance Issues

**Problem**: Low frame rates or high CPU usage

**Solutions**:
1. **Reduce Resolution**:
   - Use lower resolution streams
   - Reduce frame rate if needed

2. **USB Bandwidth**:
   - Disconnect other USB 3.0 devices
   - Use dedicated USB 3.0 controller

3. **System Resources**:
   - Close unnecessary applications
   - Ensure adequate RAM and CPU

## 🧪 Hands-On Exercises

### Exercise 1: Basic Camera Test
1. Connect your RealSense camera
2. Launch RealSense Viewer
3. Enable both color and depth streams
4. Take a screenshot of the viewer
5. Record a 10-second bag file

### Exercise 2: Python Connection Test
1. Create a Python script to connect to your camera
2. Print camera information (serial number, firmware version)
3. Capture one frame of each stream type
4. Save the frames as images

### Exercise 3: Troubleshooting Practice
1. Disconnect your camera
2. Try to run the Python script
3. Note the error message
4. Reconnect and verify it works again

## 📝 Quiz Questions

1. **What type of USB port is required for RealSense cameras?**
   - A) USB 2.0
   - B) USB 3.0
   - C) USB-C
   - D) Any USB port

2. **Which command lists connected RealSense devices?**
   - A) `rs-list-devices`
   - B) `rs-enumerate-devices`
   - C) `rs-show-devices`
   - D) `rs-devices`

3. **What is the main purpose of the RealSense Viewer?**
   - A) Programming interface
   - B) Testing and configuration
   - C) Data processing
   - D) Network streaming

## 🎯 Next Steps

Great! Your RealSense camera is now set up and working. In the next module, you'll learn about depth and color data fundamentals.

**Ready to continue?** → [Module 3: Depth & Color Basics](./module-3-depth-basics.md)

## 📚 Additional Resources

- [RealSense SDK Documentation](https://www.realsenseai.com/developers/)
- [RealSense Viewer User Guide](https://www.realsenseai.com/get-started-depth-camera/)
- [Troubleshooting Guide](https://www.realsenseai.com/support/)
- [Hardware Setup Guide](https://www.realsenseai.com/get-started/)
