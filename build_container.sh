#!/bin/bash

# Script to build the Docker container for ROS 2 Humble with RPLidar support

set -e

echo "========================================="
echo "Building Docker Container for ROS 2 Omni Base"
echo "========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build the Docker image
echo ""
echo "Building Docker image..."
docker build -t omni-base-ros2:latest -f "${SCRIPT_DIR}/.devcontainer/Dockerfile" "${SCRIPT_DIR}"

echo ""
echo "========================================="
echo "Docker container built successfully!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Allow X11 forwarding: xhost +local:docker"
echo "2. Run the container: ./run_container.sh"
echo ""
