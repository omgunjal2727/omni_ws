# Omni Base Robot Project

This project contains the ROS 2 workspace for the Omni Base robot, featuring simulation, hardware integration, and autonomous navigation capabilities.

## Prerequisites

- **OS**: Linux (Ubuntu 22.04 recommended)
- **Docker**: Installed and configured
- **Hardware**:
    - RPLidar A2M8 (connected to `/dev/ttyUSB0`)
    - Omni-directional robot base (optional for simulation)
- **X11 Server**: Required for GUI display (RViz2) from the container

## Quick Start

The following 4 terminal commands are everything you need to get started.

### 1. Run the Docker Container

```bash
docker run -it --rm \
  --name omni-base-ros2 \
  --network host \
  --privileged \
  --device /dev/ttyUSB0 \
  -v /home/omg/omni_ws:/home/container_user/omni_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -w /home/container_user/omni_ws \
  omni_slam:cartographer \
  /bin/bash
```

### 2. Enter the Container (if needed)

If you need a second terminal:

```bash
docker exec -it omni-base-ros2 /bin/bash
```

### 3. Build the Workspace

Inside the container, you must build the workspace since we are mounting the source code.
**First, navigate to the workspace directory:**

```bash
cd ~/omni_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch Robot Description

Inside the container:

```bash
ros2 launch omni_base_description description.launch.py
```

### 5. Launch Lidar

Inside the container:

```bash
ros2 launch omni_base_bringup rplidar.launch.py
```

### 6. Launch Cartographer SLAM

Inside the container:

```bash
ros2 launch omni_base_bringup cartographer.launch.py
```

### 7. Launch RViz2

Inside the container:

```bash
rviz2 -d ~/omni_ws/src/omni_base_bringup/config/cartographer_view.rviz
```

### SLAM (Simultaneous Localization and Mapping)

We use **Cartographer** for SLAM. To start mapping:

```bash
ros2 launch omni_base_bringup cartographer.launch.py
```

This will launch:
- Cartographer node
- Occupancy grid node (publishes `/map`)
- RViz2 (if configured in the launch file, otherwise launch separately)

### Navigation (Nav2)

To launch the navigation stack:

```bash
ros2 launch omni_base_bringup nav2.launch.py
```

### Full Bringup

To launch the complete system (hardware drivers, description, etc.):

```bash
ros2 launch omni_base_bringup bringup.launch.py
```

## Troubleshooting

### RPLidar Not Detected
- Check connection: `ls /dev/ttyUSB*`
- Update serial port in `src/omni_base_bringup/launch/rplidar.launch.py` if needed.

### Permission Denied (`/dev/ttyUSB0`)
- Run `sudo chmod 666 /dev/ttyUSB0` on the host or inside the container.

### GUI/X11 Issues
- Ensure you ran `xhost +local:docker` on the host before starting the container.
- Check `echo $DISPLAY` inside the container.

## Package Structure

- `omni_base_description`: URDF/Xacro robot description.
- `omni_base_bringup`: Launch files and high-level configurations.
- `omni_base_control`: Control logic and hardware interfaces.
- `omni_base_navigation`: Nav2 configuration and maps.
