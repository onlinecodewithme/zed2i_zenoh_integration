#!/bin/bash

# ZED2i Navigation and SLAM Launch Script
# This script launches the complete Nav2 SLAM system with ZED2i camera

echo "Starting ZED2i Navigation and SLAM with Nav2..."
echo "=============================================="

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 not sourced. Please run:"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo "Error: Workspace not built. Please run:"
    echo "colcon build --symlink-install"
    exit 1
fi

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Check if ZED camera is connected
echo "Checking ZED camera connection..."
if ! lsusb | grep -q "ZED"; then
    echo "Warning: ZED camera not detected. Make sure it's connected."
    echo "You can still run the system for testing purposes."
fi

# Launch the Nav2 SLAM system
echo "Launching Nav2 SLAM system..."
echo ""
echo "Features available:"
echo "- Real-time SLAM mapping"
echo "- Autonomous navigation"
echo "- Path planning and obstacle avoidance"
echo "- RViz2 visualization with Nav2 tools"
echo ""
echo "Usage:"
echo "1. Wait for all nodes to start"
echo "2. Use RViz2 'Nav2 Goal' tool to set navigation targets"
echo "3. Save maps with: ros2 run nav2_map_server map_saver_cli -f ~/my_map"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "=============================================="

# Set display for GUI applications
export DISPLAY=:0

# Launch the system
ros2 launch zed_occupancy_mapping zed_nav2_slam.launch.py
