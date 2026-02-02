# ⚙️ Level 2: Intermediate — Building Vision-Aware Apps

Welcome to Level 2 of RealSense University! This level builds upon your beginner knowledge to create more sophisticated applications that integrate RealSense with robotics frameworks and computer vision tools.

## 🎯 Learning Objectives

By the end of Level 2, you will be able to:
- Process and visualize 3D point clouds effectively
- Integrate RealSense cameras with ROS2
- Build depth-based computer vision applications
- Develop cross-platform RealSense applications
- Create obstacle detection systems for robotics

## 📋 Prerequisites

- Completion of Level 1 or equivalent experience
- Basic understanding of robotics concepts
- Familiarity with Linux command line
- Basic knowledge of ROS2 (helpful but not required)
- Understanding of computer vision fundamentals

## 📚 Modules Overview

| Module | Topic | Duration | Difficulty |
|--------|-------|----------|------------|
| [Module 1](./module-1-point-clouds.md) | Working with Point Clouds | 90 min | ⭐⭐ |
| [Module 2](./module-2-ros2.md) | Using RealSense in ROS2 | 120 min | ⭐⭐⭐ |
| [Module 3](./module-3-depth-applications.md) | Depth-Based Applications | 90 min | ⭐⭐ |
| [Module 4](./module-4-cross-platform.md) | Cross-Platform Development | 60 min | ⭐⭐ |
| [Module 5](./module-5-mini-project.md) | Mini Project: Obstacle Detection | 150 min | ⭐⭐⭐ |

## 🛠️ Required Hardware & Software

### Hardware
- ** RealSense Camera**: D435, D455, or D457 recommended
- **Development Computer**: Linux (Ubuntu 20.04+), Windows 10+, or macOS
- **Additional Sensors**: IMU-enabled camera for advanced features
- **Robotics Platform**: Optional for ROS2 integration

### Software
- **RealSense SDK 2.0**: Latest version
- **ROS2**: Humble or Iron (for ROS2 modules)
- **Open3D**: 3D data processing
- **OpenCV**: Computer vision
- **Python 3.8+**: Development environment

## 🚀 Quick Start Guide

1. **Complete Level 1** or ensure you have equivalent experience
2. **Set up ROS2** following [Module 2](./module-2-ros2.md) instructions
3. **Install additional libraries** for point cloud processing
4. **Complete modules** in order for best learning experience
5. **Build the obstacle detection project** to apply your knowledge

## 📖 Module Details

### [Module 1: Working with Point Clouds](./module-1-point-clouds.md)
Master point cloud generation, filtering, and visualization techniques.

**Key Topics:**
- Point cloud generation and processing
- Filtering and noise reduction
- Visualization with Open3D and RViz
- Aligning depth and color streams
- Point cloud registration and merging

### [Module 2: Using RealSense in ROS2](./module-2-ros2.md)
Integrate RealSense cameras with the Robot Operating System 2.

**Key Topics:**
- Installing realsense-ros package
- Publishing RGB, depth, and IMU topics
- Visualizing data in RViz
- Using TF frames for coordinate transforms
- Creating custom ROS2 nodes

### [Module 3: Depth-Based Applications](./module-3-depth-applications.md)
Build practical applications using depth data for computer vision.

**Key Topics:**
- Obstacle detection and distance alerts
- Background segmentation with depth masks
- Real-time object tracking
- Gesture recognition
- 3D object detection

### [Module 4: Cross-Platform Development](./module-4-cross-platform.md)
Develop RealSense applications for different platforms and architectures.

**Key Topics:**
- Running on NVIDIA Jetson platforms
- Optimizing for Raspberry Pi
- Performance and power optimization
- Cross-compilation techniques
- Platform-specific considerations

### [Module 5: Mini Project: Obstacle Detection](./module-5-mini-project.md)
Build a complete obstacle detection system for autonomous mobile robots.

**Key Topics:**
- ROS2-based obstacle detection node
- Real-time depth processing
- Safety zone definition
- Integration with robot navigation
- Performance optimization

## 🎯 Assessment & Progress

Each module includes:
- **Hands-on exercises** with real-world applications
- **Code examples** you can run and modify
- **Integration challenges** with robotics frameworks
- **Performance optimization** techniques
- **Troubleshooting guides** for common issues

## 🔧 Development Environment Setup

### Ubuntu/Linux Setup

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool

# Install RealSense ROS2 package
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Install additional dependencies
sudo apt install python3-opencv python3-numpy python3-matplotlib
pip3 install open3d
```

### Windows Setup

```bash
# Install ROS2 Humble for Windows
# Download from: https://github.com/ros2/ros2/releases
# Follow installation instructions

# Install RealSense SDK
# Download from: https://www.intelrealsense.com/sdk-2/
# Install with ROS2 support

# Install Python dependencies
pip install pyrealsense2 opencv-python numpy matplotlib open3d
```

### macOS Setup

```bash
# Install ROS2 Humble
brew install ros-humble-desktop

# Install RealSense SDK
brew install librealsense

# Install Python dependencies
pip3 install pyrealsense2 opencv-python numpy matplotlib open3d
```

## 🧪 Hands-On Learning Approach

### Project-Based Learning
Each module includes practical projects that build upon each other:
- **Point Cloud Processing**: Create a 3D scene reconstruction tool
- **ROS2 Integration**: Build a robot perception system
- **Depth Applications**: Develop a smart surveillance system
- **Cross-Platform**: Optimize for embedded systems
- **Final Project**: Complete obstacle detection system

### Real-World Applications
- **Robotics**: Autonomous navigation and manipulation
- **Industrial**: Quality inspection and safety systems
- **Healthcare**: Patient monitoring and rehabilitation
- **Security**: Intrusion detection and access control
- **Entertainment**: AR/VR and gaming applications

## 🆘 Getting Help

If you encounter issues:
1. Check the [troubleshooting section](./troubleshooting.md)
2. Review the [FAQ](./faq.md)
3. Join our [Discord community](https://discord.gg/realsense-university)
4. Search [GitHub issues](https://github.com/your-org/realsense-university/issues)
5. Consult [ROS2 documentation](https://docs.ros.org/en/humble/)

## 🎉 Completion Certificate

Upon completing all modules and the mini project, you'll receive a **Level 2 Completion Certificate** and be ready to advance to [Level 3: Advanced](./../level-3-advanced/).

## 🔗 Related Resources

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [RealSense ROS2 Package](https://github.com/realsenseai/realsense-ros)
- [Open3D Documentation](http://www.open3d.org/docs/)
- [OpenCV Python Tutorials](https://opencv-python-tutroals.readthedocs.io/)

---

**Ready to start?** Let's begin with [Module 1: Working with Point Clouds](./module-1-point-clouds.md)!
