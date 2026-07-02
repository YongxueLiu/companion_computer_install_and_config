#!/bin/bash
# Source this file to set up the environment for running VINS-Fusion
# with the custom OpenCV 4.8.0 build on Jetson Nano / ROS2 Humble.
#
# Usage:
#   source /home/lyx/ros2_vins/setup_vins_env.sh

# ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Workspace overlay
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
fi

# OpenCV 4.8.0 libraries (required to avoid ABI conflict with system 4.5d)
export LD_LIBRARY_PATH="/home/lyx/.local/opencv-4.8.0/lib:${LD_LIBRARY_PATH}"

echo "VINS-Fusion environment ready."
echo "OpenCV 4.8.0 lib: /home/lyx/.local/opencv-4.8.0/lib"
echo "Workspace: ${WS_ROOT}"
