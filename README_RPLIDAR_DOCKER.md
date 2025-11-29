# RPLidar Visualization in RViz2 with Docker

This guide will help you build a Docker container and visualize your RPLidar sensor data in RViz2.

## Prerequisites

- Docker installed on your system
- RPLidar A2M8 connected to `/dev/ttyUSB0`
- X11 server running (for GUI display)

## Quick Start

### 1. Build the Docker Container

```bash
./build_container.sh
```

This will build a Docker image with:
- ROS 2 Humble
- Gazebo Harmonic
- RPLidar ROS driver
- RViz2
- All necessary dependencies

### 2. Enable X11 Forwarding

```bash
xhost +local:docker
```

This allows the Docker container to display GUI applications (RViz2) on your host.

### 3. Run the Container

```bash
./run_container.sh
```

This will:
- Start the Docker container
- Mount your workspace at `/home/ubuntu/omni_ws`
- Enable X11 forwarding for GUI
- Grant access to `/dev/ttyUSB0` (RPLidar device)
- Drop you into a bash shell inside the container

### 4. Setup and Visualize (Inside Container)

Once inside the container, run:

```bash
./setup_and_visualize.sh
```

This script will:
1. Source ROS 2 Humble
2. Install dependencies with rosdep
3. Build the workspace with colcon
4. Set RPLidar device permissions
5. Launch RPLidar and RViz2 visualization

## Manual Steps (Alternative)

If you prefer to run commands manually inside the container:

### Build the Workspace

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/omni_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### Set Device Permissions

```bash
sudo chmod 666 /dev/ttyUSB0
```

### Launch Visualization

```bash
ros2 launch omni_base_bringup visualize_lidar.launch.py
```

## What Gets Launched

The `visualize_lidar.launch.py` file launches:

1. **Robot State Publisher** - Publishes the robot's TF tree from URDF
2. **Joint State Publisher** - Publishes joint states
3. **RPLidar Node** - Connects to the RPLidar and publishes scan data
4. **RViz2** - Visualizes the robot and laser scan data

## RViz2 Configuration

The launch file uses a pre-configured RViz2 setup at:
```
src/omni_base_bringup/config/lidar_view.rviz
```

This configuration includes:
- Robot model display
- LaserScan display for RPLidar data
- TF frames
- Proper fixed frame settings

## Troubleshooting

### RPLidar Not Detected

If your RPLidar is on a different port:

1. Find the device: `ls /dev/ttyUSB*`
2. Update the launch file: `src/omni_base_bringup/launch/rplidar.launch.py`
3. Change the `serial_port` parameter to match your device

### Permission Denied on /dev/ttyUSB0

```bash
sudo chmod 666 /dev/ttyUSB0
```

Or add your user to the dialout group (requires logout/login):
```bash
sudo usermod -a -G dialout $USER
```

### X11 Display Issues

If RViz2 doesn't show up:

1. Ensure X11 forwarding is enabled:
   ```bash
   xhost +local:docker
   ```

2. Check DISPLAY variable inside container:
   ```bash
   echo $DISPLAY
   ```

3. Try setting it manually:
   ```bash
   export DISPLAY=:0
   ```

### Container Already Running

If you get an error that the container name is already in use:

```bash
docker stop omni-base-ros2
docker rm omni-base-ros2
```

Then run `./run_container.sh` again.

## Development Workflow

### Using VS Code Dev Containers

The workspace includes a `.devcontainer` configuration. To use it:

1. Install the "Dev Containers" extension in VS Code
2. Open the workspace folder
3. Press `F1` and select "Dev Containers: Reopen in Container"

This will automatically build and start the container with proper configuration.

### Rebuilding After Changes

If you modify the Dockerfile or add new dependencies:

```bash
./build_container.sh
```

Then restart the container:
```bash
./run_container.sh
```

### Building Only Specific Packages

Inside the container:

```bash
colcon build --packages-select omni_base_bringup omni_base_description
source install/setup.bash
```

## Package Structure

```
omni_ws/
├── src/
│   ├── omni_base_description/    # Robot URDF and description
│   ├── omni_base_bringup/        # Launch files and configs
│   ├── omni_base_control/        # Control nodes
│   └── omni_base_navigation/     # Navigation configs
├── build_container.sh            # Build Docker image
├── run_container.sh              # Run Docker container
└── setup_and_visualize.sh        # Setup and launch visualization
```

## Next Steps

After successfully visualizing the RPLidar:

1. **SLAM**: Use `ros2 launch omni_base_bringup slam.launch.py` to create a map
2. **Navigation**: Use `ros2 launch omni_base_bringup nav2.launch.py` for autonomous navigation
3. **Full Bringup**: Use `ros2 launch omni_base_bringup bringup.launch.py` to start everything

## Additional Resources

- [RPLidar ROS Documentation](https://github.com/Slamtec/rplidar_ros)
- [RViz2 User Guide](https://github.com/ros2/rviz)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
