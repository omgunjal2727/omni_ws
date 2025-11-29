#!/bin/bash

# Quick test script to verify container can start
# This runs a simple command in the container to verify it works

echo "Testing container startup..."

docker run --rm \
    --entrypoint /bin/bash \
    omni-base-ros2:latest \
    -c "source /opt/ros/humble/setup.bash && echo 'ROS 2 Humble sourced successfully!' && ros2 pkg list | head -5"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Container test passed!"
    echo "✓ ROS 2 is properly configured"
    echo ""
    echo "You can now run: ./run_container.sh"
else
    echo ""
    echo "✗ Container test failed"
    echo "Please check the error messages above"
fi
