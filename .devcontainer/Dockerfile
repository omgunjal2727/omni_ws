FROM ghcr.io/soham2560/humble-harmonic:latest

# Switch to root to install packages
USER root

# Install sudo and other dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    ros-humble-rplidar-ros \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# The base image already has container_user with UID/GID 1000
# We'll use that user and just ensure it has sudo access
ARG USERNAME=container_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Ensure the user has passwordless sudo access
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Source ROS setup in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Switch to non-root user
USER $USERNAME
