#!/bin/bash

# Script to run the Docker container with proper X11 forwarding and device access

set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "========================================="
echo "Starting ROS 2 Omni Base Container"
echo "========================================="

# Check if X11 is allowed
echo ""
echo "Ensuring X11 forwarding is enabled..."
xhost +local:docker > /dev/null 2>&1 || echo "Warning: Could not enable X11 forwarding"

# Check if RPLidar device exists
if [ -e /dev/ttyUSB0 ]; then
    echo "RPLidar device found at /dev/ttyUSB0"
    DEVICE_ARG="--device=/dev/ttyUSB0"
else
    echo "Warning: RPLidar device not found at /dev/ttyUSB0"
    echo "You can still run the container, but RPLidar won't work until you connect it."
    DEVICE_ARG=""
fi

echo ""
echo "Starting container..."
echo ""

# Run the container
docker run -it --rm \
    --name omni-base-ros2 \
    --privileged \
    --net=host \
    --ipc=host \
    --pid=host \
    --entrypoint /bin/bash \
    -e DISPLAY=${DISPLAY} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${SCRIPT_DIR}:/home/container_user/omni_ws" \
    ${DEVICE_ARG} \
    omni-base-ros2:latest

echo ""
echo "Container stopped."
