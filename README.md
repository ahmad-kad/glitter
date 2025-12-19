# LiDAR-Camera Fusion

Real-time sensor fusion for LiDAR and camera data on Raspberry Pi with ROS 2.

## Quick Start

### Hardware Setup (One-time)

```bash
# 1. Install Ubuntu 24.04 on Raspberry Pi 5
# (Follow HARDWARE_SETUP.md for detailed instructions)

# 2. Run automated setup
chmod +x scripts/setup_ubuntu_24.sh
./scripts/setup_ubuntu_24.sh

# 3. Install camera drivers
./scripts/install_camera_drivers.sh --type picamera

# 4. Install Unitree L2 LiDAR
./scripts/install_unitree_l2.sh

# 5. Validate hardware
./scripts/test_hardware.sh
```

### Development Setup

```bash
# Install Python dependencies
pip3 install -r requirements.txt --break-system-packages

# Run system tests
python3 tests/test_system.py

# Run infrastructure tests
python3 tests/test_infrastructure.py --benchmark
```

### Run Fusion

```bash
# Start LiDAR driver
ros2 launch unilidar_sdk run.launch.py

# Start camera driver (in another terminal)
# (Depends on your camera type - see HARDWARE_SETUP.md)

# Run fusion
python3 src/core/fusion.py

# Or use simplified infrastructure
python3 src/core/simple_fusion_node.py

# Visualize (see docs/rviz_guide.md for detailed instructions)
ros2 run rviz2 rviz2 -d config/l2_fusion.rviz
# Or use the launch script:
./scripts/utilities/launch_lidar_rviz.sh
```

## Features

- **Real-time Fusion**: Colorize LiDAR point clouds with camera images at 20+ FPS
- **Hardware Support**: Unitree L2/L1 LiDAR + Raspberry Pi Camera v3/Global Shutter/AI Camera
- **Memory Pool System**: Zero-GC real-time processing (< 20ms latency)
- **Async I/O Pipeline**: Non-blocking ROS operations
- **Rolling Buffer**: Motion-compensated temporal synchronization
- **Comprehensive Testing**: Synthetic simulation + hardware validation
- **Production Ready**: Fault tolerance, monitoring, thermal management

## System Requirements

- **Hardware**: Raspberry Pi 5 (8GB RAM), Unitree L2 LiDAR, Camera
- **OS**: Ubuntu 24.04 LTS Server (64-bit ARM)
- **ROS**: ROS 2 Jazzy Jalisco
- **Python**: 3.10+
- **Memory**: 4GB+ available RAM
- **Storage**: 32GB+ free space

## Project Structure

```
glitter/
├── README.md                          # This file
├── HARDWARE_SETUP.md                  # Hardware installation guide
├── DEVELOPMENT_GUIDE.md               # Development setup
├── requirements.txt                   # Python dependencies
│
├── scripts/                           # Scripts and utilities
│   ├── setup_ubuntu_24.sh            # Ubuntu + ROS setup
│   ├── install_camera_drivers.sh     # Camera installation
│   ├── install_unitree_l2.sh         # LiDAR setup
│   ├── test_hardware.sh              # Hardware validation
│   └── utilities/                    # Utility scripts
│       ├── launch_lidar_rviz.sh     # Launch LiDAR + RViz
│       ├── build_lidar_driver.sh     # Build LiDAR driver
│       ├── verify_pointcloud.sh      # Verify point cloud
│       ├── accumulate_pointcloud.py  # Point cloud accumulator
│       ├── save_pointcloud.py        # Save point cloud
│       └── visualize_with_open3d.py  # Open3D visualization
│
├── src/                               # Source code
│   ├── camera/                     # Camera-specific modules
│   │   └── pi_camera_node.py       # Raspberry Pi camera node
│   │
│   ├── core/                       # Core fusion logic
│   │   ├── fusion.py               # Main fusion node
│   │   ├── simple_fusion_node.py   # Simplified fusion node
│   │   ├── calibration.py          # Calibration tools
│   │   └── map.py                  # Mapping functionality
│   │
│   ├── infrastructure/             # Real-time infrastructure
│   │   ├── infrastructure.py       # Simplified infrastructure
│   │   ├── moving_sensor_handler.py # Vehicle-mounted sensor handler
│   │   ├── memory_pool.py          # Memory management
│   │   ├── async_pipeline.py       # Async I/O
│   │   ├── rolling_buffer.py       # Temporal sync
│   │   └── test_infrastructure.py  # Infrastructure tests
│   │
│   └── utils/                      # Utilities
│       ├── utils.py                # Core utilities
│       └── simulator.py            # Synthetic testing
│
├── tests/                          # Test suite
│   ├── test_system.py             # System validation
│   ├── hardware_tests/            # Hardware-specific tests
│   │   ├── test_camera.py         # Camera validation
│   │   ├── test_lidar.py          # LiDAR validation
│   │   ├── test_integration.py    # Full system tests
│   │   └── visual_camera_test.py  # Camera visualization test
│
├── scripts/                        # Scripts and utilities
│   ├── setup_ubuntu_24.sh         # Ubuntu + ROS setup
│   ├── install_camera_drivers.sh  # Camera installation
│   ├── install_unitree_l2.sh      # LiDAR setup
│   ├── test_hardware.sh           # Hardware validation
│   ├── activate_env.sh            # Environment activation
│   ├── setup_env.sh               # Environment setup
│   └── utilities/                 # Utility scripts
│       ├── launch_lidar_rviz.sh   # Launch LiDAR + RViz
│       ├── build_lidar_driver.sh  # Build LiDAR driver
│       ├── verify_pointcloud.sh   # Verify point cloud
│       └── accumulate_pointcloud.py # Point cloud accumulator
│
├── config/                        # Configuration files
│   ├── l2_fusion.rviz            # RViz visualization
│   ├── l2_fusion_simple.rviz     # Simple RViz config
│   └── camera_calibration.yaml   # Camera parameters (future)
│
└── docs/                          # Documentation
    ├── project_guide.md          # Technical documentation
    ├── REAL_WORLD_ROADMAP.md     # Feature roadmap
    ├── rviz_guide.md             # RViz visualization guide
    ├── troubleshooting.md        # Troubleshooting guide
    ├── camera_troubleshooting.md # Camera-specific troubleshooting
    ├── pi_camera_setup.md        # Pi Camera setup guide
    └── quick_start.md            # Quick start guide
```

## Testing Strategy

### 1. Synthetic Testing (Development)

```bash
# Run all infrastructure benchmarks
python3 tests/test_infrastructure.py --benchmark

# Test specific scenarios
python3 tests/test_infrastructure.py --scenario "Urban Environment"

# Run unit tests
python3 tests/test_system.py
```

### 2. Hardware Testing (Deployment)

```bash
# Comprehensive hardware validation
./scripts/test_hardware.sh

# Individual component tests
python3 tests/hardware_tests/test_camera.py
python3 tests/hardware_tests/test_lidar.py

# Full integration test (60 seconds)
python3 tests/hardware_tests/test_integration.py --extended 60
```

### 3. Performance Validation

Expected performance on Raspberry Pi 5:
- **Fusion Rate**: 20+ FPS
- **Memory Usage**: < 1.5GB
- **CPU Usage**: 60-80%
- **Latency**: < 50ms end-to-end
- **Thermal**: < 75°C sustained

## Key Optimizations

### Real-Time Performance
- **Memory Pools**: Pre-allocated buffers eliminate GC pauses
- **Async I/O**: Non-blocking ROS operations
- **SIMD Operations**: Vectorized math for 2-4x speedup
- **Rolling Buffer**: Motion-aware temporal synchronization

### Reliability Features
- **Graceful Degradation**: Automatic fallback modes
- **Sensor Monitoring**: Health tracking and automatic recovery
- **Thermal Management**: Prevent overheating with smart throttling
- **Network Resilience**: Handle packet loss and latency

### Code Quality
- **78% Reduction**: From 2,463 lines to 546 lines of core code
- **Modular Design**: Clean separation of concerns
- **Comprehensive Testing**: Unit, integration, and hardware tests
- **Documentation**: Complete setup and development guides

## Getting Help

1. **Hardware Issues**: Check `HARDWARE_SETUP.md`
2. **Development Setup**: See `DEVELOPMENT_GUIDE.md`
3. **RViz Visualization**: See `docs/rviz_guide.md`
4. **Troubleshooting**: See `docs/troubleshooting.md`
5. **Performance Issues**: Run `python3 tests/test_infrastructure.py --benchmark`
6. **Code Architecture**: Read `docs/REAL_WORLD_ROADMAP.md`

## License

This project is open source. See individual files for license information.
