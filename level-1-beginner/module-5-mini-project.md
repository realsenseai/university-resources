# Module 5: Mini Project: Distance Measurement

## 🎯 Learning Objectives

By the end of this module, you will be able to:
- Build a complete distance measurement application
- Implement real-time object detection and measurement
- Create a user-friendly interface
- Handle edge cases and error conditions
- Apply all concepts learned in Level 1

## 🎯 Project Overview

**Goal**: Create a RealSense-powered distance measurement tool that can:
- Measure distances to objects in real-time
- Display measurements on screen
- Save measurement data
- Provide a simple user interface

## 🛠️ Project Requirements

### Functional Requirements
- [ ] Real-time distance measurement
- [ ] Visual feedback with crosshair
- [ ] Distance display in millimeters and inches
- [ ] Save measurements to file
- [ ] Keyboard controls for interaction

### Technical Requirements
- [ ] Use pyrealsense2 for camera access
- [ ] Use OpenCV for image display
- [ ] Handle camera connection errors
- [ ] Provide clear user instructions
- [ ] Code should be well-commented

## 🏗️ Project Structure

```
distance_measurement/
├── main.py                 # Main application
├── camera_handler.py       # Camera management
├── measurement.py          # Distance calculation
├── ui.py                   # User interface
├── data/
│   └── measurements.csv    # Saved measurements
└── README.md              # Project documentation
```

## 💻 Implementation

### Step 1: Camera Handler

Create `camera_handler.py`:

```python
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCamera:
    """RealSense camera handler class"""
    
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.is_streaming = False
        
    def initialize(self):
        """Initialize the camera"""
        try:
            # Create pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(
                rs.stream.depth, 
                self.width, 
                self.height, 
                rs.format.z16, 
                self.fps
            )
            self.config.enable_stream(
                rs.stream.color, 
                self.width, 
                self.height, 
                rs.format.bgr8, 
                self.fps
            )
            
            # Start streaming
            self.pipeline.start(self.config)
            self.is_streaming = True
            
            print("✅ Camera initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Camera initialization failed: {e}")
            return False
    
    def get_frames(self):
        """Get current frames from camera"""
        if not self.is_streaming:
            return None, None
            
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            
            # Get individual frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if depth_frame and color_frame:
                # Convert to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                return depth_image, color_image
            else:
                return None, None
                
        except Exception as e:
            print(f"❌ Error getting frames: {e}")
            return None, None
    
    def stop(self):
        """Stop camera streaming"""
        if self.pipeline and self.is_streaming:
            self.pipeline.stop()
            self.is_streaming = False
            print("✅ Camera stopped")
    
    def get_camera_info(self):
        """Get camera information"""
        if not self.is_streaming:
            return None
            
        try:
            # Get device info
            device = self.pipeline.get_active_profile().get_device()
            return {
                'name': device.get_info(rs.camera_info.name),
                'serial': device.get_info(rs.camera_info.serial_number),
                'firmware': device.get_info(rs.camera_info.firmware_version)
            }
        except Exception as e:
            print(f"❌ Error getting camera info: {e}")
            return None
```

### Step 2: Measurement Logic

Create `measurement.py`:

```python
import numpy as np
import cv2
import csv
import datetime
from typing import Tuple, Optional

class DistanceMeasurer:
    """Distance measurement and data handling"""
    
    def __init__(self, data_file='data/measurements.csv'):
        self.data_file = data_file
        self.measurements = []
        
    def measure_distance(self, depth_image: np.ndarray, x: int, y: int) -> Optional[float]:
        """Measure distance at specific pixel coordinates"""
        try:
            # Check bounds
            if x < 0 or x >= depth_image.shape[1] or y < 0 or y >= depth_image.shape[0]:
                return None
            
            # Get depth value
            depth_value = depth_image[y, x]
            
            # Check if depth is valid (not zero)
            if depth_value == 0:
                return None
            
            # Convert to millimeters (assuming depth units are in mm)
            distance_mm = depth_value
            return distance_mm
            
        except Exception as e:
            print(f"❌ Error measuring distance: {e}")
            return None
    
    def mm_to_inches(self, mm: float) -> float:
        """Convert millimeters to inches"""
        return mm / 25.4
    
    def get_measurement_info(self, depth_image: np.ndarray, x: int, y: int) -> dict:
        """Get comprehensive measurement information"""
        distance_mm = self.measure_distance(depth_image, x, y)
        
        if distance_mm is None:
            return {
                'distance_mm': None,
                'distance_inches': None,
                'valid': False,
                'message': 'No valid depth data'
            }
        
        distance_inches = self.mm_to_inches(distance_mm)
        
        return {
            'distance_mm': distance_mm,
            'distance_inches': distance_inches,
            'valid': True,
            'message': f'{distance_mm:.1f}mm ({distance_inches:.2f}")'
        }
    
    def save_measurement(self, x: int, y: int, distance_mm: float, timestamp: str = None):
        """Save measurement to CSV file"""
        if timestamp is None:
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        measurement = {
            'timestamp': timestamp,
            'x': x,
            'y': y,
            'distance_mm': distance_mm,
            'distance_inches': self.mm_to_inches(distance_mm)
        }
        
        self.measurements.append(measurement)
        
        # Save to CSV
        try:
            with open(self.data_file, 'a', newline='') as csvfile:
                fieldnames = ['timestamp', 'x', 'y', 'distance_mm', 'distance_inches']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header if file is empty
                if csvfile.tell() == 0:
                    writer.writeheader()
                
                writer.writerow(measurement)
                
            print(f"✅ Measurement saved: {measurement['message']}")
            
        except Exception as e:
            print(f"❌ Error saving measurement: {e}")
    
    def load_measurements(self) -> list:
        """Load measurements from CSV file"""
        try:
            with open(self.data_file, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                return list(reader)
        except FileNotFoundError:
            print(f"📁 No existing measurements file found")
            return []
        except Exception as e:
            print(f"❌ Error loading measurements: {e}")
            return []
```

### Step 3: User Interface

Create `ui.py`:

```python
import cv2
import numpy as np
from typing import Tuple, Optional

class MeasurementUI:
    """User interface for distance measurement"""
    
    def __init__(self, window_name="RealSense Distance Measurement"):
        self.window_name = window_name
        self.crosshair_size = 20
        self.crosshair_thickness = 2
        self.crosshair_color = (0, 255, 0)  # Green
        self.text_color = (255, 255, 255)   # White
        self.text_bg_color = (0, 0, 0)      # Black
        
    def draw_crosshair(self, image: np.ndarray, x: int, y: int) -> np.ndarray:
        """Draw crosshair at specified coordinates"""
        # Draw horizontal line
        cv2.line(image, 
                (x - self.crosshair_size, y), 
                (x + self.crosshair_size, y), 
                self.crosshair_color, 
                self.crosshair_thickness)
        
        # Draw vertical line
        cv2.line(image, 
                (x, y - self.crosshair_size), 
                (x, y + self.crosshair_size), 
                self.crosshair_color, 
                self.crosshair_thickness)
        
        return image
    
    def draw_text_with_background(self, image: np.ndarray, text: str, position: Tuple[int, int]) -> np.ndarray:
        """Draw text with background for better visibility"""
        x, y = position
        
        # Get text size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Draw background rectangle
        cv2.rectangle(image, 
                     (x, y - text_height - baseline), 
                     (x + text_width, y + baseline), 
                     self.text_bg_color, 
                     -1)
        
        # Draw text
        cv2.putText(image, text, (x, y), font, font_scale, self.text_color, thickness)
        
        return image
    
    def draw_measurement_info(self, image: np.ndarray, measurement_info: dict, x: int, y: int) -> np.ndarray:
        """Draw measurement information on image"""
        if not measurement_info['valid']:
            text = measurement_info['message']
            self.draw_text_with_background(image, text, (10, 30))
            return image
        
        # Draw distance information
        distance_text = f"Distance: {measurement_info['message']}"
        self.draw_text_with_background(image, distance_text, (10, 30))
        
        # Draw coordinates
        coord_text = f"Position: ({x}, {y})"
        self.draw_text_with_background(image, coord_text, (10, 60))
        
        # Draw instructions
        instructions = [
            "Controls:",
            "Mouse: Move crosshair",
            "Click: Measure distance",
            "S: Save measurement",
            "Q: Quit"
        ]
        
        for i, instruction in enumerate(instructions):
            self.draw_text_with_background(image, instruction, (10, 90 + i * 25))
        
        return image
    
    def create_depth_colormap(self, depth_image: np.ndarray) -> np.ndarray:
        """Create colored depth map for visualization"""
        # Normalize depth image
        depth_normalized = cv2.convertScaleAbs(depth_image, alpha=0.03)
        
        # Apply colormap
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        return depth_colormap
    
    def display_images(self, color_image: np.ndarray, depth_image: np.ndarray, 
                      measurement_info: dict, x: int, y: int) -> bool:
        """Display images with measurement information"""
        # Create depth colormap
        depth_colormap = self.create_depth_colormap(depth_image)
        
        # Draw crosshair on both images
        color_with_crosshair = self.draw_crosshair(color_image.copy(), x, y)
        depth_with_crosshair = self.draw_crosshair(depth_colormap.copy(), x, y)
        
        # Draw measurement info
        color_with_info = self.draw_measurement_info(color_with_crosshair, measurement_info, x, y)
        depth_with_info = self.draw_measurement_info(depth_with_crosshair, measurement_info, x, y)
        
        # Display images
        cv2.imshow('Color Stream', color_with_info)
        cv2.imshow('Depth Stream', depth_with_info)
        
        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        return key == ord('q')  # Return True if 'q' is pressed
```

### Step 4: Main Application

Create `main.py`:

```python
import cv2
import os
from camera_handler import RealSenseCamera
from measurement import DistanceMeasurer
from ui import MeasurementUI

def main():
    """Main application function"""
    
    # Create data directory if it doesn't exist
    os.makedirs('data', exist_ok=True)
    
    # Initialize components
    camera = RealSenseCamera()
    measurer = DistanceMeasurer()
    ui = MeasurementUI()
    
    # Initialize camera
    if not camera.initialize():
        print("❌ Failed to initialize camera. Exiting.")
        return
    
    # Get camera info
    camera_info = camera.get_camera_info()
    if camera_info:
        print(f"📷 Camera: {camera_info['name']}")
        print(f"🔢 Serial: {camera_info['serial']}")
        print(f"⚙️ Firmware: {camera_info['firmware']}")
    
    # Load existing measurements
    existing_measurements = measurer.load_measurements()
    print(f"📊 Loaded {len(existing_measurements)} existing measurements")
    
    # Mouse callback for crosshair movement
    crosshair_x, crosshair_y = 320, 240  # Center of 640x480 image
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal crosshair_x, crosshair_y
        if event == cv2.EVENT_MOUSEMOVE:
            crosshair_x, crosshair_y = x, y
    
    # Set mouse callback
    cv2.setMouseCallback('Color Stream', mouse_callback)
    cv2.setMouseCallback('Depth Stream', mouse_callback)
    
    print("\n🎯 Distance Measurement Tool Started!")
    print("Move your mouse to position the crosshair, click to measure, press 's' to save, 'q' to quit")
    
    try:
        while True:
            # Get frames from camera
            depth_image, color_image = camera.get_frames()
            
            if depth_image is None or color_image is None:
                print("❌ Failed to get frames from camera")
                break
            
            # Get measurement at crosshair position
            measurement_info = measurer.get_measurement_info(
                depth_image, crosshair_x, crosshair_y
            )
            
            # Display images with UI
            should_quit = ui.display_images(
                color_image, depth_image, measurement_info, crosshair_x, crosshair_y
            )
            
            if should_quit:
                break
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s') and measurement_info['valid']:
                # Save measurement
                measurer.save_measurement(
                    crosshair_x, crosshair_y, measurement_info['distance_mm']
                )
            
            elif key == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n⏹️ Application interrupted by user")
    
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    
    finally:
        # Cleanup
        camera.stop()
        cv2.destroyAllWindows()
        print("✅ Application closed successfully")

if __name__ == "__main__":
    main()
```

### Step 5: Project Documentation

Create `README.md`:

```markdown
# RealSense Distance Measurement Tool

A Python application for real-time distance measurement using RealSense cameras.

## Features

- Real-time distance measurement
- Visual crosshair for precise targeting
- Distance display in millimeters and inches
- Save measurements to CSV file
- Simple keyboard and mouse controls

## Requirements

- RealSense camera (D405, D415, D435, D455, D457, or D555)
- Python 3.7+
- Required packages: pyrealsense2, opencv-python, numpy

## Installation

```bash
pip install pyrealsense2 opencv-python numpy
```

## Usage

```bash
python main.py
```

## Controls

- **Mouse**: Move crosshair to target objects
- **Click**: Measure distance at crosshair position
- **S**: Save current measurement to file
- **Q**: Quit application

## Output

Measurements are saved to `data/measurements.csv` with the following format:
- Timestamp
- X, Y coordinates
- Distance in millimeters
- Distance in inches

## Troubleshooting

- Ensure camera is connected via USB 3.0
- Check that RealSense SDK is properly installed
- Verify camera permissions on Linux/macOS
```

## 🧪 Testing Your Application

### Test Cases

1. **Basic Functionality**:
   - Launch the application
   - Verify camera connection
   - Test crosshair movement
   - Measure distances to various objects

2. **Edge Cases**:
   - Test with no objects in view
   - Test with reflective surfaces
   - Test with very close objects
   - Test with very far objects

3. **Data Persistence**:
   - Save multiple measurements
   - Verify CSV file creation
   - Check data format and accuracy

4. **Error Handling**:
   - Disconnect camera during operation
   - Test with invalid coordinates
   - Verify graceful shutdown

### Performance Testing

```python
# Add to main.py for performance testing
import time

def performance_test():
    """Test application performance"""
    start_time = time.time()
    frame_count = 0
    
    # Run for 10 seconds
    while time.time() - start_time < 10:
        depth_image, color_image = camera.get_frames()
        if depth_image is not None and color_image is not None:
            frame_count += 1
    
    fps = frame_count / 10
    print(f"Average FPS: {fps:.1f}")
```

## 🎯 Project Extensions

### Advanced Features

1. **Multiple Measurement Points**:
   - Allow multiple crosshairs
   - Measure distances between points
   - Calculate object dimensions

2. **Object Detection**:
   - Integrate with OpenCV object detection
   - Automatically measure detected objects
   - Track objects over time

3. **3D Visualization**:
   - Display 3D point clouds
   - Show measurement points in 3D
   - Export 3D models

4. **Data Analysis**:
   - Plot measurement trends
   - Statistical analysis
   - Export to different formats

## 📝 Quiz Questions

1. **What is the main advantage of using a class-based approach for the camera handler?**
   - A) Faster execution
   - B) Better code organization
   - C) Smaller file size
   - D) Easier installation

2. **Why is error handling important in the measurement application?**
   - A) To make the code faster
   - B) To handle invalid depth data
   - C) To reduce memory usage
   - D) To improve image quality

3. **What format is used for saving measurements?**
   - A) JSON
   - B) XML
   - C) CSV
   - D) Binary

## 🎉 Congratulations!

You've successfully completed Level 1 of RealSense University! You now have:
- A solid understanding of RealSense cameras
- Hands-on experience with the RealSense SDK
- A working distance measurement application
- Skills to build more complex RealSense applications

## 🎯 Next Steps

Ready to advance to the next level? Check out [Level 2: Intermediate — Building Vision-Aware Apps](../level-2-intermediate/) to learn about:
- Point cloud processing
- ROS2 integration
- Advanced computer vision applications
- Cross-platform development

## 📚 Additional Resources

- [RealSense SDK Examples](https://github.com/realsenseai/librealsense/tree/master/examples)
- [OpenCV Python Tutorials](https://opencv-python-tutroals.readthedocs.io/)
- [CSV File Handling in Python](https://docs.python.org/3/library/csv.html)
- [RealSense Community Forum](https://github.com/realsenseai/librealsense/discussions)
