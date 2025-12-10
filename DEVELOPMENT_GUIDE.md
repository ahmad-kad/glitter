# Development Environment Setup

## Quick Start

```bash
# Clone repository
cd ~
git clone <your-repo-url> glitter
cd glitter

# Run automated setup
chmod +x scripts/setup_ubuntu_24.sh
./scripts/setup_ubuntu_24.sh

# Test installation
python3 tests/test_system.py

# Run fusion
python3 src/core/fusion.py
```

## Manual Setup

### 1. Install Dependencies

```bash
# Install from requirements.txt
pip3 install -r requirements.txt --break-system-packages

# Or manually:
pip3 install --break-system-packages \
  numpy>=1.24.0 \
  opencv-python>=4.8.0 \
  scipy>=1.11.0 \
  transforms3d>=0.4.1 \
  psutil>=5.9.0 \
  tqdm>=4.66.0 \
  lz4>=4.3.0 \
  laspy[lazrs]>=2.5.0
```

### 2. Configure Camera

```bash
# For Pi Camera v3
python3 scripts/configure_camera.py --type picamera --resolution 1280x720 --fps 30

# For USB Camera
python3 scripts/configure_camera.py --type usb --device /dev/video0
```

### 3. Calibrate System

```bash
# Run interactive calibration
python3 src/core/calibration.py

# Calibration will:
# 1. Display camera feed with LiDAR overlay
# 2. Allow adjustment of extrinsic parameters
# 3. Save calibration to config/camera_calibration.yaml
```

### 4. Test System

```bash
# Run comprehensive tests
python3 tests/test_system.py

# Run hardware-specific tests
python3 tests/hardware_tests/test_camera.py
python3 tests/hardware_tests/test_lidar.py
python3 tests/hardware_tests/test_integration.py
```

### 5. Run Fusion

```bash
# Simple fusion node
python3 src/core/fusion.py

# Or with infrastructure optimizations
python3 src/infrastructure/simple_fusion_node.py

# Visualize in RViz
ros2 run rviz2 rviz2 -d config/l2_fusion.rviz
```

---

## Testing Procedures

### Unit Tests

```bash
# Test individual components
python3 -m pytest tests/ -v

# Test specific module
python3 -m pytest tests/test_infrastructure.py::test_memory_pool -v
```

### Integration Tests

```bash
# Test full pipeline with synthetic data
python3 tests/test_infrastructure.py --benchmark

# Test real-world scenarios
python3 tests/test_infrastructure.py --all-scenarios
```

### Hardware Tests

```bash
# Test camera capture
python3 tests/hardware_tests/test_camera.py

# Test LiDAR acquisition
python3 tests/hardware_tests/test_lidar.py

# Test sensor synchronization
python3 tests/hardware_tests/test_integration.py --duration 60  # 60 second test
```

---

## Performance Validation

### Benchmark System

```bash
# Run comprehensive benchmarks
python3 tests/test_infrastructure.py --benchmark

# Expected results:
# - Memory pool allocation: < 10ms
# - Async I/O latency: < 5ms
# - Frame sync success: > 90%
# - Total fusion latency: < 50ms
```

### Monitor Real-Time Performance

```bash
# Terminal 1: Run fusion
python3 src/core/fusion.py

# Terminal 2: Monitor ROS topics
ros2 topic hz /colored_cloud
ros2 topic bw /colored_cloud

# Terminal 3: System monitoring
htop
```

---

## Reliability Improvements (No Bloat)

### Minimal Installation

For embedded/production deployment, install only essentials:

```bash
# Minimal ROS 2 (no desktop tools)
sudo apt install -y ros-humble-ros-base

# Core packages only
pip3 install --break-system-packages numpy opencv-python-headless

# Skip optional packages:
# - scipy (only needed for advanced filtering)
# - tqdm (progress bars, not needed in production)
# - laspy (LAZ compression, only for storage)
```

### Stripped-Down Configuration

Use the simplified infrastructure:

```python
# Use simple_fusion_node.py instead of real_world_fusion_node.py
# 245 lines vs 394 lines

# Use infrastructure.py instead of network_infrastructure.py
# 301 lines vs 742 lines

# Total reduction: 78% less code!
```

### Production Hardening

```bash
# Disable unnecessary services
sudo systemctl disable bluetooth.service 2>/dev/null || true
sudo systemctl disable avahi-daemon.service 2>/dev/null || true

# Set CPU governor to performance
echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase file descriptor limits
echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf

# Enable watchdog (auto-restart on hang)
sudo apt install -y watchdog
sudo systemctl enable watchdog
```

---

## Development Workflow

1. **Edit code** on development machine
2. **Test locally** with simulator:
   ```bash
   python3 tests/test_infrastructure.py --scenario "Perfect Conditions"
   ```

3. **Deploy to Raspberry Pi**:
   ```bash
   rsync -avz --exclude '__pycache__' glitter/ ubuntu@raspberrypi:~/glitter/
   ```

4. **Test on hardware**:
   ```bash
   ssh ubuntu@raspberrypi
   cd ~/glitter
   python3 src/core/fusion.py
   ```

5. **Monitor performance**:
   ```bash
   # On Pi, check diagnostics
   ros2 topic echo /diagnostics/fusion
   ```

6. **Iterate** based on real-world performance
```

---

## Directory Structure

```
glitter/
├── README.md                          # Quick start guide
├── HARDWARE_SETUP.md                  # Hardware installation guide
├── DEVELOPMENT_GUIDE.md               # Development environment setup
├── REAL_WORLD_ROADMAP.md              # Feature roadmap and analysis
├── requirements.txt                   # Python dependencies
│
├── scripts/                           # Setup and utility scripts
│   ├── setup_ubuntu_24.sh            # Automated Ubuntu 24.04 setup
│   ├── install_camera_drivers.sh     # Camera driver installation
│   ├── install_unitree_l2.sh         # Unitree L2 LiDAR setup
│   └── test_hardware.sh              # Hardware validation script
│
├── src/                               # Core source code
│   ├── core/                         # Core fusion components
│   │   ├── fusion.py                 # Main fusion node
│   │   ├── calibration.py            # Calibration tools
│   │   └── map.py                    # Mapping module
│   │
│   ├── infrastructure/               # Real-time infrastructure
│   │   ├── infrastructure.py         # Simplified infrastructure
│   │   ├── memory_pool.py            # Memory pools
│   │   ├── async_pipeline.py         # Async I/O
│   │   └── rolling_buffer.py         # Temporal sync
│   │
│   └── utils/                        # Utilities
│       ├── utils.py                  # Core utilities
│       └── simulator.py              # Sensor simulation
│
├── tests/                            # Testing infrastructure
│   ├── test_system.py               # System tests
│   ├── test_infrastructure.py       # Infrastructure tests
│   └── hardware_tests/              # Hardware-specific tests
│       ├── test_camera.py
│       ├── test_lidar.py
│       └── test_integration.py
│
├── config/                          # Configuration files
│   ├── l2_fusion.rviz              # RViz config
│   ├── camera_calibration.yaml     # Camera params
│   └── lidar_config.yaml           # LiDAR params
│
└── docs/                            # Documentation
    ├── project_guide.md            # Architecture docs
    └── optimization_issues.py      # Analysis docs
```

---

## Code Quality Standards

### Python Style
```bash
# Install development tools
pip3 install --break-system-packages \
  black \
  isort \
  flake8 \
  mypy

# Format code
black src/ tests/
isort src/ tests/

# Lint code
flake8 src/ tests/
mypy src/
```

### Testing Standards
- **Unit tests** for all core functions
- **Integration tests** for component interaction
- **Performance benchmarks** for optimization validation
- **Hardware tests** for real-world validation

### Documentation Standards
- **Docstrings** for all public functions
- **Type hints** for function parameters
- **README** files for each major component
- **Inline comments** for complex logic
