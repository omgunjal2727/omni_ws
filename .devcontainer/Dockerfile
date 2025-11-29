FROM ghcr.io/soham2560/humble-harmonic:latest

# 🔥 FIX ENTRYPOINT PERMISSIONS
USER root
RUN chmod +x /ros_entrypoint.sh

# Define ARGs BEFORE using them
# ==========
ARG USERNAME=container_user
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Use root for installation
USER root

# ==========
# Install ROS dependencies (Lidar, SLAM Toolbox, Cartographer, RViz)
# ==========
RUN apt-get update && apt-get install -y \
    sudo \
    ros-humble-rplidar-ros \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-cartographer-ros-msgs \
    && rm -rf /var/lib/apt/lists/*

# ==========
# Sudo access for non-root user
# ==========
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ==========
# Add user to dialout group for /dev/ttyUSB0 access
# ==========
RUN groupadd -f dialout || true \
    && usermod -a -G dialout ${USERNAME}

# ==========
# Ensure default shell is bash so .bashrc loads
# ==========
RUN chsh -s /bin/bash ${USERNAME} || true

# ==========
# Add ROS sourcing to .bashrc
# ==========
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# ==========
# Switch to non-root user
# ==========
USER ${USERNAME}
