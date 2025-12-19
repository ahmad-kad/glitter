#!/usr/bin/env python3
"""System test script for LiDAR-camera fusion"""

import sys

def test_python():
    """Test Python environment"""
    print("Testing Python environment...")
    try:
        import numpy as np
        import cv2
        print("✓ Core libraries available")
        return True
    except ImportError as e:
        print(f"✗ Missing library: {e}")
        return False

def test_compilation():
    """Test code compilation"""
    print("Testing code compilation...")
    try:
        import py_compile
        py_compile.compile('fusion.py', doraise=True)
        py_compile.compile('calibration.py', doraise=True)
        py_compile.compile('utils.py', doraise=True)
        print("✓ Code compiles successfully")
        return True
    except Exception as e:
        print(f"✗ Compilation error: {e}")
        return False

def test_simulator():
    """Test simulator functionality"""
    print("Testing simulator...")
    try:
        from simulator import SensorSimulator, SensorConfig
        config = SensorConfig()
        config.num_objects = 1
        sim = SensorSimulator(config)
        points, colors = sim.generate_point_cloud()
        image, meta = sim.generate_camera_image(points, colors)

        if len(points) > 0 and meta['projected_points'] >= 0:
            print("✓ Simulator generates valid data")
            return True
        else:
            print("✗ Simulator output invalid")
            return False
    except Exception as e:
        print(f"✗ Simulator error: {e}")
        return False

def main():
    """Run all tests"""
    print("LiDAR-Camera Fusion System Test")
    print("=" * 35)

    tests = [
        ("Python Environment", test_python),
        ("Code Compilation", test_compilation),
        ("Simulator", test_simulator),
    ]

    passed = 0
    total = len(tests)

    for name, test_func in tests:
        print(f"\n{name}:")
        if test_func():
            passed += 1
        print()

    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All tests passed! System ready.")
    else:
        print("⚠️ Some tests failed. Check output above.")
        sys.exit(1)

if __name__ == "__main__":
    main()