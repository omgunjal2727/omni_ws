#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the packages
cd /home/omg/omni_ws
colcon build --packages-select omni_base_description omni_base_bringup

# Source the workspace
source install/setup.bash

echo ""
echo "Build complete! You can now run:"
echo "  ros2 launch omni_base_bringup visualize_lidar.launch.py"
echo ""
echo "Make sure your RPLidar is connected to /dev/ttyUSB0"
echo "You may need to give permissions: sudo chmod 666 /dev/ttyUSB0"
