# FSDS Autonomous Driving System

A complete autonomous vehicle development stack for **Formula Student Driverless Simulator (FSDS)** using **ROS 2 Humble** and **GPU-accelerated deep learning** on Arch Linux.

---

## 🎯 Project Overview

This repository implements a modular autonomous racing pipeline:

| Phase | Component | Status |
|-------|-----------|--------|
| **Phase 1** | LiDAR Perception |Detect traffic cones in real-time |
| **Phase 2** | SLAM & Mapping | Build persistent global cone map |
| **Phase 3** | Path Planning |Compute racing line through track |
| **Phase 4** | Control & MPC | Steering/throttle optimization |
| **Phase 5** | Neural Networks | Vision-based perception pipeline |

---

## 🔧 Software Stack

- **Host OS:** Arch Linux (rolling release)
- **Container:** Docker with NVIDIA Container Toolkit
- **ROS 2:** Humble
- **Simulator:** FSDS v2.2.0
- **Python:** 3.10+
- **Deep Learning:** TensorFlow 2.15+ with GPU support
- **NVIDIA Driver:** 580.105.08
- **CUDA:** 12.2 (/usr/local/cuda-12.2/)
- **cuDNN:** 8.9.5
- **GCC:** 12

---

## 📦 Installation

### 1. Clone Repository
```bash
git clone https://github.com/TarunSamala/fsds-autonomous-driving.git
cd fsds-autonomous-driving
```

### 2. Download FSDS Simulator (Host)
```bash
cd ~
wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip
unzip fsds-v2.2.0-linux.zip
cd fsds-v2.2.0-linux && chmod +x FSDS.sh
```

---

## 🔗 ROS 2 Topics Reference

### Published Topics (from our nodes)
```
/detected_cones           MarkerArray     Phase 1 output (gray cylinders, car frame)
/cone_map                 MarkerArray     Phase 2 output (yellow cylinders, global)
/control_command          ControlCommand  Keyboard input → simulator
```

### Subscribed Topics (from FSDS bridge)
```
/lidar/Lidar1             PointCloud2     3D LiDAR scan
/lidar/Lidar2             PointCloud2     Secondary LiDAR
/fsds/cam1/image_color    Image           Camera 1
/fsds/cam2/image_color    Image           Camera 2
/testing_only/odom        Odometry        Car pose + velocity
/testing_only/track       Marker          Ground truth cones (white)
/imu                      Imu             Inertial measurement
/gss                      Float64         Ground speed
```

See `docs/ROS2-TOPICS.md` for full reference.

---

## 🔨 Development Workflow

### After Any Python Code Change
```bash
cd /workspace/ros2_ws
colcon build --packages-select autonomous_driving --symlink-install
source install/setup.bash
ros2 run autonomous_driving <node_name>
```

---

## 📚 References

- **FSDS Official:** https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator
- **ROS 2 Humble Docs:** https://docs.ros.org/en/humble/
- **Arch Linux Wiki:** https://wiki.archlinux.org

---
