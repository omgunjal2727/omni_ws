#!/bin/bash

# Script to build workspace and launch RPLidar visualization in RViz2
# This script should be run INSIDE the Docker container

set -e

echo "========================================="
echo "Setting up ROS 2 Workspace and RPLidar"
echo "========================================="

# Source ROS 2 Humble
echo ""
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /home/container_user/omni_ws

# Install dependencies
echo ""
echo "Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble

# Build the workspace
echo ""
echo "Building workspace..."
colcon build --symlink-install

# Source the workspace
echo ""
echo "Sourcing workspace..."
source install/setup.bash

# Check if RPLidar device exists and set permissions
if [ -e /dev/ttyUSB0 ]; then
    echo ""
    echo "Setting RPLidar device permissions..."
    sudo chmod 666 /dev/ttyUSB0
    echo "RPLidar device ready at /dev/ttyUSB0"
else
    echo ""
    echo "========================================="
    echo "WARNING: RPLidar device not found!"
    echo "========================================="
    echo "Please connect your RPLidar to /dev/ttyUSB0"
    echo "After connecting, run: sudo chmod 666 /dev/ttyUSB0"
    echo ""
    read -p "Press Enter to continue anyway or Ctrl+C to exit..."
fi

# Launch visualization
echo ""
echo "========================================="
echo "Launching RPLidar Visualization in RViz2"
echo "========================================="
echo ""

ros2 launch omni_base_bringup visualize_lidar.launch.py
