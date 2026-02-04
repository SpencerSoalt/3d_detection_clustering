#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "${WS}/install/setup.bash" ]; then
    source ${WS}/install/setup.bash
fi

# Set up display for RViz (if running GUI)
if [ -n "$DISPLAY" ]; then
    xhost +local:docker 2>/dev/null || true
fi

# Print environment info
echo "============================================"
echo "ROS 2 3D Detection Environment"
echo "============================================"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "Workspace: ${WS}"
echo "============================================"
echo ""
echo "Available commands:"
echo "  - Launch detector: ros2 launch voxel_detector voxel_detector.launch.py"
echo "  - Run RViz: rviz2"
echo "  - List topics: ros2 topic list"
echo "  - Play rosbag: ros2 bag play /path/to/bag"
echo "============================================"

# Execute the command passed to docker run
exec "$@"
