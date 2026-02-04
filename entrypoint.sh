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

# Execute the command passed to docker run
exec "$@"
