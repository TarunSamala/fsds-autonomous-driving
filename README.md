# 🚗 FSDS Autonomous Driving System

> **A complete autonomous vehicle development stack for Formula Student Driverless Simulator (FSDS) using ROS 2 Humble and GPU-accelerated deep learning on Arch Linux.**

**Status:** Phase 1 (Waypoint Following) - In Active Development  
**Last Updated:** January 3, 2026  
**Location:** Nārnaund, Haryana, India

---

## 📋 Project Overview

This repository implements a **modular autonomous racing pipeline** for the Formula Student Driverless challenge:

| Phase | Component | Status | Description |
|-------|-----------|--------|-------------|
| **Phase 1** | LiDAR Perception | ✅ Working | DBSCAN cone detection from PointCloud2 |
| **Phase 2** | SLAM & Mapping | ✅ Working | Global cone map with de-duplication |
| **Phase 3** | Waypoint Following | 🔧 Debugging | Pure Pursuit control (Phase 1 blocker) |
| **Phase 4** | Path Planning | 📋 TODO | Trajectory optimization & speed ramping |
| **Phase 5** | Neural Networks | 📋 TODO | Vision-based perception with TensorFlow GPU |

---

## 🔧 Hardware & Software Stack

### Hardware
```
CPU:        Intel i7-11800H @ 4.6 GHz (8 cores)
GPU:        NVIDIA RTX 3050 Laptop Mobile
RAM:        16 GB
Storage:    NVMe SSD, 35GB+ free
Display:    X11 (not Wayland)
Network:    WiFi 192.168.1.9
```

### Software Stack
```
Host OS:          Arch Linux (rolling release, Jan 2026)
Desktop:          KDE Plasma 6.5.4 + X11
Container:        Docker + NVIDIA Container Toolkit
ROS 2:            Humble
Simulator:        FSDS v2.2.0 (native ROS2 bridge)
Python:           3.13.11
Deep Learning:    TensorFlow 2.15+ with CUDA GPU support
NVIDIA Driver:    590.48.01
CUDA Toolkit:     12.2.91
cuDNN:            8.9.5
GCC:              15.2.1
Git:              2.52.0
```

---

## 🚀 Quick Start

### Prerequisites
```bash
# Host (Arch Linux)
cd ~/FSDS
./FSDS.sh &                    # Start simulator in background

# Wait 15 seconds for FSDS to fully load
sleep 15

# Launch ROS2 environment (automatic bridge setup)
/usr/local/bin/ros2-start      # Enters interactive Docker container
```

### Inside ROS2 Container
```bash
# Terminal 1: Manual control
ros2 run autonomous_driving keyboard_controller

# Terminal 2: LiDAR cone detection
ros2 run autonomous_driving lidar_listener

# Terminal 3: SLAM mapping
ros2 run autonomous_driving slam_node

# Terminal 4: Record waypoints
ros2 run autonomous_driving waypoint_recorder_perfect

# Terminal 5: Visualize everything
rviz2 &
# Set Fixed Frame to: fsds/map
# Add displays: /detected_cones, /cone_map, /recorded_waypoints, /testing_only/odom
```

---

## 📦 Installation

### 1. Clone Repository
```bash
cd ~/Projects  # or your preferred location
git clone https://github.com/TarunSamala/fsds-autonomous-driving.git
cd fsds-autonomous-driving
```

### 2. Install FSDS Simulator (Host)
```bash
cd ~
wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip
unzip fsds-v2.2.0-linux.zip
cd FSDS && chmod +x FSDS.sh
```

### 3. Enable ROS2 Bridge in FSDS
Edit `~/FSDS/settings.json`:
```json
{
    "SeeDocsAt": "https://FS-Driverless.github.io/Formula-Student-Driverless-Simulator/",
    "SettingsVersion": 1.2,
    "ViewMode": "SpringArmChase",
    "ClockSpeed": 1.0,
    "RosBridge": {
        "RosPublisher": true,
        "RosSubscriber": true,
        "Port": 13000,
        "Frequency": 50
    },
    // ... rest of settings.json unchanged
}
```

### 4. Docker Container (Already Set Up)
```bash
# Verify container exists
docker ps -a | grep ros2-dev-build

# If needed, rebuild:
cd /workspace/ros2_ws
colcon build --packages-select autonomous_driving --symlink-install
```

---

## 🔌 ROS 2 Architecture

### Published Topics (Our Nodes)
| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/detected_cones` | MarkerArray | fsds/Lidar1 | DBSCAN clustered cone centers |
| `/cone_map` | MarkerArray | fsds/FSCar | Global de-duplicated cone map |
| `/recorded_waypoints` | MarkerArray | fsds/FSCar | Recorded path visualization |
| `/recorded_path` | Path | fsds/FSCar | Waypoint trajectory |

### Subscribed Topics (FSDS Native)
| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/testing_only/odom` | Odometry | fsds/FSCar | Ground truth pose (50 Hz) |
| `/lidar/Lidar1` | PointCloud2 | fsds/Lidar1 | 3D LiDAR scan |
| `/imu` | Imu | fsds/FSCar | Inertial measurement |
| `/gss` | Float64 | - | Ground speed sensor |

### Published Topics (Manual Control)
| Topic | Type | Description |
|-------|------|-------------|
| `/control_command` | ControlCommand (fs_msgs) | Steering, throttle, brake [-1.0, 1.0] |

---

## 🏗️ Node Architecture

```
keyboard_control.py
  └─> /control_command (throttle, steering, brake)
      └─> FSDS Simulator (processes control inputs)

lidar_listener.py
  └─ Input: /lidar/Lidar1 (PointCloud2)
  └─ Algorithm: SimpleDBSCAN (eps=0.20m, min_samples=5)
  └─> /detected_cones (MarkerArray)

slam_node.py
  └─ Input: /detected_cones + /testing_only/odom
  └─ Algorithm: Running average cone de-duplication (5cm merge)
  └─> /cone_map (global MarkerArray)

waypoint_recorder_perfect.py
  └─ Input: /testing_only/odom (manual drive only)
  └─ Auto-saves when loop closes (within 1.0m of start after 100+ points)
  └─> waypoints.json + /recorded_waypoints

waypoint_follower.py
  ├─ Input: waypoints.json + /testing_only/odom
  ├─ Algorithm: Pure Pursuit steering control
  ├─ Lookahead: 1.2m ahead of car
  ├─ Wheelbase: 0.4m
  └─> /control_command (steering, 0.08 throttle)
  
  ⚠️ STATUS: Oscillation issues - DEBUGGING PHASE
```

---

## 📊 Known Issues & Status

### Critical (Phase 1 Blocker)
- **waypoint_follower.py:** Oscillates violently, leaves track
  - Cause: Lookahead logic or steering gain mismatch
  - Solution: Debug lookahead point selection on straight section first
  - Test: `ros2 run autonomous_driving waypoint_follower throttle:=0.05 lookahead:=1.5`

### Important (Quality-of-Life)
- **Frame alignment:** Some publishers use `fsds/FSCar` (should be `fsds/map` for globals)
  - Affects: slam_node.py, waypoint_recorder_perfect.py visualization
  - Fix: Update header frame_id to `fsds/map` for global outputs

- **Waypoint spacing:** Double-lane paths from oscillatory manual driving
  - Fix: Add heading continuity check or filter outliers

### Minor
- Cone duplication on lap 2 (de-duplication working, minor artifacts only)
- `odom_publisher.py` unused (unnecessary TF overhead)
- `scan_republisher.py` unused (not needed for current pipeline)

---

## 🔧 Development Workflow

### After Code Changes
```bash
# Inside container
cd /workspace/ros2_ws
colcon build --packages-select autonomous_driving --symlink-install
source install/setup.bash

# Run updated node
ros2 run autonomous_driving <node_name>
```

### Debugging with RViz
```bash
# Inside container
rviz2 &

# Configure in GUI:
# 1. Fixed Frame: fsds/map (or fsds/FSCar)
# 2. Add displays:
#    - /testing_only/odom (Odometry)
#    - /detected_cones (MarkerArray)
#    - /cone_map (MarkerArray)
#    - /recorded_waypoints (MarkerArray)
#    - TF (to see frame tree)

# Monitor topics
ros2 topic hz /testing_only/odom      # Should be 50 Hz
ros2 topic echo /detected_cones --once  # First detection
```

### Testing Pure Pursuit (Phase 1)
```bash
# Terminal 1: Keyboard control
ros2 run autonomous_driving keyboard_controller
# Drive manually with WASD to record waypoint loop

# Terminal 2: Monitor odometry
ros2 topic hz /testing_only/odom

# Terminal 3: Test waypoint following
ros2 run autonomous_driving waypoint_follower \
  throttle:=0.05 \
  lookahead:=1.2 \
  wheelbase:=0.4
# Watch oscillation behavior → debug in RViz
```

---

## 📚 Key References

### FSDS Documentation
- **Official GitHub:** https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator
- **ROS2 Bridge Setup:** https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/ros-bridge/
- **Getting Started:** https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/getting-started-with-ros/

### ROS 2 & Robotics
- **ROS 2 Humble Docs:** https://docs.ros.org/en/humble/
- **TF2 & Coordinate Frames:** https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html
- **F1TENTH Autonomous Racing:** https://f1tenth-coursekit.readthedocs.io/

### System & Tools
- **Arch Linux Wiki (Docker):** https://wiki.archlinux.org/title/Docker
- **Arch Linux Wiki (NVIDIA):** https://wiki.archlinux.org/title/NVIDIA
- **NVIDIA CUDA Docs:** https://docs.nvidia.com/cuda/
- **TensorFlow GPU Setup:** https://www.tensorflow.org/install/gpu

### Control Theory
- **Pure Pursuit Algorithm:** [Mobile Robot Kinematics](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Land_Vehicles.pdf)
- **SLAM Fundamentals:** [An Introduction to SLAM](https://ieeexplore.ieee.org/document/5681215)
- **Particle Filters:** [A Tutorial on Particle Filters](https://www.robots.ox.ac.uk/~parg/pubs/brml_04_mcmc.pdf)

---

## 🔄 Development Roadmap

### Phase 1 (Current - Debugging)
- [ ] Fix waypoint_follower.py oscillation
  - Test on straight section only
  - Tune lookahead distance + steering gain
  - Validate wheelbase matching
- [ ] Fix frame alignment (use `fsds/map` globally)
- [ ] Complete 1-2 full laps autonomously

### Phase 2 (After Phase 1)
- [ ] Cone-based localization correction
- [ ] Speed ramping (slow on turns, fast on straights)
- [ ] Safety: Auto-brake if off-track >0.5m
- [ ] Improve waypoint quality (filter oscillations)

### Phase 3 (Future)
- [ ] Trajectory optimization (RRT, MPC)
- [ ] Vision-based perception (TensorFlow GPU)
- [ ] Multi-sensor fusion (LiDAR + camera + IMU)
- [ ] Real-time path planning

### Phase 4 (Later)
- [ ] Cone color classification (yellow vs blue)
- [ ] Dynamic obstacle avoidance
- [ ] Velocity planning for 75+ mph target
- [ ] Competition deployment

---

## 📝 File Structure
```
fsds-autonomous-driving/
├── README.md                           (This file)
├── setup.py                            (ROS2 package config)
├── autonomous_driving/                 (Main Python package)
│   ├── keyboard_control.py            ✅ Manual WASD control
│   ├── lidar_listener.py              ✅ Cone detection
│   ├── slam_node.py                   ✅ SLAM mapping
│   ├── waypoint_recorder_perfect.py   ✅ Waypoint recording
│   ├── waypoint_follower.py           🔧 Pure Pursuit (debugging)
│   ├── odom_publisher.py              ⚠️ Optional TF broadcaster
│   └── scan_republisher.py            ⚠️ Optional LaserScan converter
└── docs/                               (Documentation)
    ├── ROS2-TOPICS.md                 (Detailed topic reference)
    └── DEBUGGING.md                   (Troubleshooting guide)
```

---

## 🚀 Getting Help

### Common Issues

**"Bridge fails to connect"**
```bash
# Verify FSDS running
ps aux | grep FSOnline

# Check RosBridge in settings.json
grep -A3 "RosBridge" ~/FSDS/settings.json

# Test ROS topics
ros2 topic list | grep odom  # Should show /testing_only/odom
```

**"Waypoint follower oscillates"**
```bash
# Watch in RViz as it drives
# Check lookahead point selection (debug output needed)
# Test on straight section first (single waypoint pair)
# Reduce steering gain or increase lookahead distance
```

**"LiDAR not detecting cones"**
```bash
# Verify cone detection
ros2 topic echo /detected_cones --once

# Check frame
# Adjust DBSCAN parameters: eps, min_samples
```

---

## 👨‍💻 Contributing

This is an **active research project**. Contributions welcome:

1. **Fork** the repository
2. **Create feature branch:** `git checkout -b feature/your-feature`
3. **Commit changes:** `git commit -m "Describe your improvement"`
4. **Push:** `git push origin feature/your-feature`
5. **Create Pull Request** with test results from Phase 1 testing

---

## 📜 License

MIT License - See LICENSE file for details

---

## 🙏 Acknowledgments

- **FSDS Team:** Formula Student Driverless Simulator framework
- **ROS 2 Team:** Middleware and ecosystem
- **NVIDIA:** GPU acceleration support
- **Arch Linux Community:** Rolling-release stability

---

**Last Updated:** Saturday, January 3, 2026  
**Maintainer:** Tarun Samala (autonomous vehicle researcher)  
**Status:** Phase 1 Development - Active Debugging  
**Next Milestone:** Successful autonomous lap completion
