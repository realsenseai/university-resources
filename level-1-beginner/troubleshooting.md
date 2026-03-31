# 🔧 Level 1 Troubleshooting Guide

This guide covers common issues encountered when getting started with RealSense cameras.

## 🔌 Connection Issues

### Camera Not Detected

**Symptoms:**
- `rs-enumerate-devices` shows no devices
- RealSense Viewer doesn't find camera
- `RuntimeError: No device connected` in Python

**Solutions:**

1. **Check USB Connection**
   ```bash
   # Verify USB connection (Linux)
   lsusb | grep Intel
   # Should show: Intel Corp. RealSense...
   ```

2. **Use USB 3.0 Port**
   - RealSense requires USB 3.0 (blue port)
   - USB 2.0 will not work properly
   - Try different USB 3.0 ports

3. **Check Cable Quality**
   - Use the included RealSense cable
   - Avoid USB hubs or extensions
   - Try a different USB 3.0 cable

4. **Linux Permissions**
   ```bash
   # Add udev rules
   sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   
   # Add user to video group
   sudo usermod -a -G video $USER
   # Log out and back in
   ```

5. **Windows Driver Issues**
   - Open Device Manager
   - Look for RealSense device under "Cameras"
   - Right-click → Update driver
   - Reinstall RealSense SDK if needed

### Camera Disconnects Randomly

**Solutions:**

1. **Power Issues**
   - Use powered USB hub
   - Try different USB port
   - Check for power-saving settings

2. **Disable USB Selective Suspend (Windows)**
   - Control Panel → Power Options
   - Change plan settings → Change advanced power settings
   - USB settings → USB selective suspend → Disabled

3. **Linux USB Power Management**
   ```bash
   # Disable USB autosuspend
   echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
   ```

## 📷 Image Quality Issues

### Depth Image is Noisy

**Solutions:**

1. **Adjust Visual Preset**
   ```python
   import pyrealsense2 as rs
   
   pipeline = rs.pipeline()
   config = rs.config()
   profile = pipeline.start(config)
   
   device = profile.get_device()
   depth_sensor = device.first_depth_sensor()
   
   # Use High Accuracy preset
   depth_sensor.set_option(rs.option.visual_preset, 
                           rs.rs400_visual_preset.high_accuracy)
   ```

2. **Apply Post-Processing Filters**
   ```python
   # Create filters
   decimation = rs.decimation_filter()
   spatial = rs.spatial_filter()
   temporal = rs.temporal_filter()
   hole_filling = rs.hole_filling_filter()
   
   # Apply to depth frame
   filtered = decimation.process(depth_frame)
   filtered = spatial.process(filtered)
   filtered = temporal.process(filtered)
   filtered = hole_filling.process(filtered)
   ```

3. **Improve Lighting**
   - Avoid direct sunlight
   - Use diffuse indoor lighting
   - Remove IR interference sources

### Black Regions in Depth

**Causes:**
- Objects too close (< 0.1m for D435)
- Objects too far (> 10m)
- Reflective surfaces
- Very dark surfaces

**Solutions:**

1. **Check Operating Range**
   | Camera | Min Range | Max Range |
   |--------|-----------|-----------|
   | D415 | 0.16m | 10m |
   | D435 | 0.11m | 10m |
   | D455 | 0.4m | 20m |
   | D457 | 0.25m | 20m |

2. **Use Hole Filling**
   ```python
   hole_filling = rs.hole_filling_filter()
   hole_filling.set_option(rs.option.holes_fill, 1)  # Fill from left
   ```

### Color and Depth Misaligned

**Solution:**
```python
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# Create align object
align = rs.align(rs.stream.color)

# In your loop:
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()
```

## 💻 Software Issues

### SDK Installation Failed

**Linux Solutions:**

```bash
# Remove old installation
sudo apt remove librealsense2*
sudo rm -rf /usr/local/lib/librealsense*

# Add repository
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

# Install
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev
```

**Windows Solutions:**
1. Run installer as Administrator
2. Disable antivirus temporarily
3. Install Visual C++ Redistributable

**macOS Solutions:**
```bash
# Using Homebrew
brew update
brew install librealsense
```

### Python Import Error

**Error:** `ImportError: No module named 'pyrealsense2'`

**Solutions:**

1. **Install pyrealsense2**
   ```bash
   pip install pyrealsense2
   # or
   pip3 install pyrealsense2
   ```

2. **Check Python Version**
   ```bash
   python --version  # Must be 3.6+
   pip show pyrealsense2  # Check installation
   ```

3. **Virtual Environment Issues**
   ```bash
   # Make sure you're in the right environment
   which python
   pip list | grep pyrealsense2
   ```

### Frame Timeout Error

**Error:** `RuntimeError: Frame didn't arrive within 5000`

**Solutions:**

1. **Increase Timeout**
   ```python
   frames = pipeline.wait_for_frames(timeout_ms=10000)
   ```

2. **Reduce Frame Rate**
   ```python
   config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)  # 15 FPS
   ```

3. **Close Other Applications**
   - Close RealSense Viewer
   - Close other programs using the camera

## 🐧 Linux-Specific Issues

### Kernel Module Not Loading

```bash
# Check if module is loaded
lsmod | grep realsense

# Load manually
sudo modprobe uvcvideo

# Check dmesg for errors
dmesg | tail -50
```

### Permission Denied

```bash
# Quick fix
sudo chmod 666 /dev/video*

# Permanent fix
sudo usermod -a -G video $USER
# Log out and back in
```

## 🪟 Windows-Specific Issues

### Blue Screen of Death (BSOD)

1. Update graphics drivers
2. Update USB drivers
3. Try different USB port
4. Disable USB power management

### Camera Works in Viewer but Not Code

1. Close RealSense Viewer completely
2. Restart your Python script
3. Check for multiple camera instances

## 🍎 macOS-Specific Issues

### Camera Not Recognized

1. Check System Preferences → Security & Privacy → Camera
2. Grant camera access to Terminal/IDE
3. Try resetting USB subsystem:
   ```bash
   sudo kextunload -b com.apple.driver.usb.AppleUSBXHCI
   sudo kextload -b com.apple.driver.usb.AppleUSBXHCI
   ```

## 📊 Performance Issues

### Low Frame Rate

**Solutions:**

1. **Reduce Resolution**
   ```python
   config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
   ```

2. **Disable Unused Streams**
   ```python
   # Only enable what you need
   config.enable_stream(rs.stream.depth)
   # Don't enable color if not needed
   ```

3. **Use Decimation Filter**
   ```python
   decimation = rs.decimation_filter()
   decimation.set_option(rs.option.filter_magnitude, 4)  # Reduce by 4x
   ```

### High CPU Usage

1. Process frames less frequently
2. Use hardware acceleration (if available)
3. Reduce post-processing filters

## 🆘 Getting More Help

If these solutions don't work:

1. **Gather Information**
   ```bash
   rs-enumerate-devices -s  # Camera info
   uname -a                  # System info
   pip show pyrealsense2    # SDK version
   ```

2. **Check Logs**
   - RealSense Viewer: Help → Show Log
   - Linux: `dmesg | grep -i realsense`

3. **Contact Support**
   - [Discord Community](https://discord.gg/SQdtSH4J)
   - [GitHub Issues](https://github.com/IntelRealSense/librealsense/issues)
   - [RealSense Community Forum](https://community.intel.com/t5/Intel-RealSense/ct-p/realsense)

---

**Still stuck?** → Check the [FAQ](./faq.md) for more common questions.
