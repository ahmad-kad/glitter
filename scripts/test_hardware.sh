#!/bin/bash
# Hardware Validation Script for LiDAR-Camera Fusion
# Tests all hardware components systematically

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
ROS_DISTRO="jazzy"

# Test results
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }

run_test() {
    local test_name="$1"
    local test_func="$2"

    ((TESTS_RUN++))
    log_info "Running: $test_name"

    if $test_func; then
        ((TESTS_PASSED++))
        log_success "$test_name"
        return 0
    else
        ((TESTS_FAILED++))
        log_fail "$test_name"
        return 1
    fi
}

test_system_info() {
    log_info "System Information:"
    echo "  OS: $(lsb_release -d | cut -f2)"
    echo "  Kernel: $(uname -r)"
    echo "  Architecture: $(uname -m)"
    echo "  CPU: $(nproc) cores"
    echo "  Memory: $(free -h | grep "^Mem:" | awk '{print $2}')"
    echo "  Storage: $(df -h / | tail -1 | awk '{print $2}')"

    return 0
}

test_ros_installation() {
    if ! command -v ros2 >/dev/null 2>&1; then
        log_fail "ROS 2 not found"
        return 1
    fi

    # Test ROS 2 version
    local ros_version
    ros_version=$(ros2 --version 2>/dev/null | head -1)
    if [[ -z "$ros_version" ]]; then
        log_fail "ROS 2 version check failed"
        return 1
    fi

    log_info "ROS Version: $ros_version"

    # Test rosdep
    if ! rosdep --version >/dev/null 2>&1; then
        log_fail "rosdep not working"
        return 1
    fi

    # Test colcon
    if ! colcon version >/dev/null 2>&1; then
        log_fail "colcon not working"
        return 1
    fi

    return 0
}

test_python_packages() {
    python3 << EOF
import sys
required_packages = [
    'numpy', 'cv2', 'scipy', 'transforms3d',
    'psutil', 'lz4'
]

missing_packages = []
for package in required_packages:
    try:
        __import__(package)
    except ImportError:
        missing_packages.append(package)

if missing_packages:
    print(f"Missing packages: {', '.join(missing_packages)}")
    sys.exit(1)

print("All Python packages available")
EOF
}

test_network_configuration() {
    # Check network interfaces
    local interfaces
    interfaces=$(ip link show | grep -E "^[0-9]+:" | cut -d: -f2 | tr -d ' ' | grep -v lo)

    log_info "Network interfaces: $interfaces"

    # Check for Ethernet connection (typical for L2)
    if ! ip addr show | grep -q "192.168.1.100"; then
        log_warning "L2 network configuration not found (192.168.1.100)"
        log_info "Run: sudo ./scripts/install_unitree_l2.sh --network-only"
    else
        log_info "L2 network configuration found"
    fi

    # Test internet connectivity
    if ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1; then
        log_info "Internet connectivity: OK"
    else
        log_warning "No internet connectivity"
    fi

    return 0
}

test_camera_hardware() {
    local camera_found=false

    # Test Pi Camera (libcamera)
    if command -v libcamera-hello >/dev/null 2>&1; then
        log_info "Testing Pi Camera..."
        if libcamera-hello --list-cameras 2>/dev/null | grep -q "Available"; then
            camera_found=true
            log_info "Pi Camera detected"

            # Test capture
            if timeout 5 libcamera-jpeg -o /tmp/camera_test.jpg --timeout 1000 2>/dev/null; then
                if [[ -f "/tmp/camera_test.jpg" ]]; then
                    rm /tmp/camera_test.jpg
                    log_info "Pi Camera capture: OK"
                fi
            else
                log_warning "Pi Camera capture test failed"
            fi
        fi
    fi

    # Test USB Camera (v4l2)
    if [[ -e "/dev/video0" ]]; then
        log_info "Testing USB Camera..."
        if v4l2-ctl --device=/dev/video0 --all >/dev/null 2>&1; then
            camera_found=true
            log_info "USB Camera detected at /dev/video0"

            # Get camera info
            local camera_info
            camera_info=$(v4l2-ctl --device=/dev/video0 --info 2>/dev/null | grep -E "Card type|Bus info" | head -2)
            log_info "Camera info: $camera_info"
        fi
    fi

    if [[ "$camera_found" == "true" ]]; then
        return 0
    else
        log_warning "No camera detected"
        return 1
    fi
}

test_lidar_hardware() {
    local L2_IP="192.168.1.150"

    log_info "Testing Unitree L2 connectivity..."

    # Test ping
    if ping -c 3 -W 2 "$L2_IP" >/dev/null 2>&1; then
        log_info "L2 reachable at $L2_IP"

        # Get ping stats
        local ping_stats
        ping_stats=$(ping -c 3 "$L2_IP" | tail -1)
        log_info "Ping stats: $ping_stats"

        # Test if ROS driver is available
        if [[ -d "~/ros2_ws/src/unilidar_sdk_ros2" ]]; then
            log_info "L2 ROS driver found"
            return 0
        else
            log_warning "L2 ROS driver not installed"
            log_info "Run: ./scripts/install_unitree_l2.sh --driver-only"
            return 1
        fi
    else
        log_warning "L2 not reachable at $L2_IP"
        log_info "Check: L2 powered on, Ethernet connected, IP configuration"
        return 1
    fi
}

test_ros_topics() {
    # Source ROS if available
    if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
        source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true
    fi

    if [[ -f "~/ros2_ws/install/setup.bash" ]]; then
        source ~/ros2_ws/install/setup.bash 2>/dev/null || true
    fi

    log_info "Testing ROS topics..."

    # Start L2 driver temporarily (if available)
    local driver_started=false
    if [[ -f "~/ros2_ws/install/lib/unilidar_sdk/unilidar_sdk_node" ]]; then
        timeout 10s ros2 launch unilidar_sdk run.launch.py >/dev/null 2>&1 &
        local driver_pid=$!
        driver_started=true
        sleep 3  # Wait for driver to start
    fi

    # Check for expected topics
    local topics
    topics=$(timeout 5 ros2 topic list 2>/dev/null)

    if [[ -z "$topics" ]]; then
        log_warning "No ROS topics found (driver may not be running)"
        return 1
    fi

    # Check for L2 topics
    if echo "$topics" | grep -q "livox/lidar"; then
        log_info "L2 point cloud topic found: /livox/lidar"

        # Test topic frequency
        local hz_output
        hz_output=$(timeout 3 ros2 topic hz /livox/lidar 2>/dev/null | grep "average rate")
        if [[ -n "$hz_output" ]]; then
            log_info "L2 frequency: $hz_output"
        fi
    else
        log_warning "L2 point cloud topic not found"
    fi

    # Clean up
    if [[ "$driver_started" == "true" ]]; then
        kill $driver_pid 2>/dev/null || true
        wait $driver_pid 2>/dev/null || true
    fi

    return 0
}

test_system_performance() {
    log_info "Testing system performance..."

    # CPU performance
    local cpu_count
    cpu_count=$(nproc)
    log_info "CPU cores: $cpu_count"

    # Memory
    local mem_info
    mem_info=$(free -h | grep "^Mem:")
    log_info "Memory: $mem_info"

    # Test CPU governor
    if [[ -f "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor" ]]; then
        local governor
        governor=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)
        log_info "CPU governor: $governor"

        if [[ "$governor" != "performance" ]]; then
            log_warning "CPU governor not set to performance (current: $governor)"
        fi
    fi

    # Test Python performance (simple benchmark)
    local python_perf
    python_perf=$(python3 -c "
import time
start = time.time()
for i in range(100000):
    x = i * i
end = time.time()
print(f'{end - start:.3f}')
")
    log_info "Python performance test: ${python_perf}s for 100k operations"

    return 0
}

test_project_files() {
    log_info "Testing project file structure..."

    local required_dirs=("src/core" "src/infrastructure" "src/utils" "tests" "config" "scripts")
    local missing_dirs=()

    for dir in "${required_dirs[@]}"; do
        if [[ ! -d "$dir" ]]; then
            missing_dirs+=("$dir")
        fi
    done

    if [[ ${#missing_dirs[@]} -gt 0 ]]; then
        log_warning "Missing directories: ${missing_dirs[*]}"
        return 1
    fi

    # Check for key files
    local required_files=(
        "requirements.txt"
        "src/core/fusion.py"
        "src/infrastructure/infrastructure.py"
        "tests/test_system.py"
    )

    local missing_files=()
    for file in "${required_files[@]}"; do
        if [[ ! -f "$file" ]]; then
            missing_files+=("$file")
        fi
    done

    if [[ ${#missing_files[@]} -gt 0 ]]; then
        log_warning "Missing files: ${missing_files[*]}"
        return 1
    fi

    log_info "Project structure: OK"
    return 0
}

print_summary() {
    echo ""
    echo "========================================"
    echo "HARDWARE VALIDATION SUMMARY"
    echo "========================================"

    echo "Tests Run: $TESTS_RUN"
    echo -e "Passed: ${GREEN}$TESTS_PASSED${NC}"
    echo -e "Failed: ${RED}$TESTS_FAILED${NC}"

    local success_rate
    success_rate=$((TESTS_PASSED * 100 / TESTS_RUN))

    echo "Success Rate: ${success_rate}%"

    if [[ $TESTS_FAILED -eq 0 ]]; then
        echo -e "${GREEN}✓ All hardware validation tests passed!${NC}"
        echo ""
        echo "Ready to run LiDAR-Camera fusion!"
        echo "Next: python3 src/core/fusion.py"
    else
        echo -e "${RED}✗ Some tests failed. Check output above.${NC}"
        echo ""
        echo "Fix issues and re-run: ./scripts/test_hardware.sh"
        exit 1
    fi
}

main() {
    echo "=========================================="
    echo "LiDAR-Camera Fusion Hardware Validation"
    echo "=========================================="

    # Run all tests
    run_test "System Information" test_system_info
    run_test "ROS Installation" test_ros_installation
    run_test "Python Packages" test_python_packages
    run_test "Network Configuration" test_network_configuration
    run_test "Camera Hardware" test_camera_hardware
    run_test "LiDAR Hardware" test_lidar_hardware
    run_test "ROS Topics" test_ros_topics
    run_test "System Performance" test_system_performance
    run_test "Project Files" test_project_files

    # Print summary
    print_summary
}

main "$@"
