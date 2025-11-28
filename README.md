# LiDAR-Camera Fusion

Real-time sensor fusion for LiDAR and camera data on Raspberry Pi with ROS 2.

## Quick Start

```bash
# Setup (Ubuntu 24.04 + ROS 2 Humble)
./start.sh

# Test system
python3 test_system.py

# Run fusion
python3 fusion.py

# Visualize
ros2 run rviz2 rviz2 -d l2_fusion.rviz
```

## Features

- **Real-time Fusion**: Colorize LiDAR point clouds with camera images
- **Hardware Support**: Unitree L2/L1 LiDAR + Raspberry Pi Camera v3
- **Performance Monitoring**: Built-in diagnostics and timing
- **Synthetic Testing**: Simulator for development
- **ROS 2 Integration**: Standard message types and TF

## Requirements

- Ubuntu 24.04
- ROS 2 Humble
- Python 3.8+
- NumPy, OpenCV

## Files

- `fusion.py` - Main fusion node
- `calibration.py` - Camera-LiDAR calibration
- `simulator.py` - Synthetic data generator
- `test_system.py` - System validation
- `l2_fusion.rviz` - RViz configuration
- `start.sh` - Setup script
