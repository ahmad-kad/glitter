#!/bin/bash
# Activation script for project virtual environment
# Source this file to activate the environment: source activate_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "Error: Virtual environment not found at $VENV_DIR"
    echo "Run ./setup_env.sh first to create the environment"
    return 1 2>/dev/null || exit 1
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Source ROS 2 if available (check multiple distributions)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Humble environment sourced"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "ROS 2 Jazzy environment sourced"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    echo "ROS 2 Iron environment sourced"
fi

# Source ROS 2 workspace if it exists
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
    echo "ROS 2 workspace sourced"
fi

# Add project src to PYTHONPATH
export PYTHONPATH="$SCRIPT_DIR/src:$HOME/.local/lib/python3.12/site-packages:/usr/local/lib/aarch64-linux-gnu/python3.12/site-packages:$PYTHONPATH"

echo "Project environment activated!"
echo "  Virtual environment: $VENV_DIR"
echo "  Project directory: $SCRIPT_DIR"
echo "  Python: $(which python3)"
echo "  PYTHONPATH: $PYTHONPATH"

