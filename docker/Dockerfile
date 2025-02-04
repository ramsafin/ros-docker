# Base image for CUDA and OpenGL
FROM nvidia/cudagl:11.4.1-runtime-ubuntu20.04

# Build argument to specify if running in WSL
ARG WSL=false

# Allow installation of additional ROS packages during build
# Usage: docker build --build-arg EXTRA_ROS_PKGS="ros-noetic-moveit ros-noetic-navigation" -t <ros_image> .
ARG EXTRA_ROS_PKGS=""

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set locale to avoid localization issues
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Disable MIT-SHM for Qt applications to prevent shared memory issues
ENV QT_X11_NO_MITSHM=1

# NVIDIA runtime for GPU acceleration
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV LD_LIBRARY_PATH=${WSL:+/usr/lib/wsl/lib:}${LD_LIBRARY_PATH}

# Install core dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git nano curl wget sudo gnupg2 mesa-utils lsb-release build-essential \
    && rm -rf /var/lib/apt/lists/*

# Set ROS distribution
ENV ROS_DISTRO=noetic

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -sSL 'http://packages.ros.org/ros.key' | apt-key add - \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-rosdep python3-catkin-tools python3-rosinstall \
    ros-${ROS_DISTRO}-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends $EXTRA_ROS_PKGS \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create a user for ROS
ENV ROS_USER=ros
ENV ROS_USER_HOME_DIR=/home/${ROS_USER}
ENV ROS_WORKSPACE=${ROS_USER_HOME_DIR}/catkin_ws

RUN useradd -m -s /bin/bash ${ROS_USER} \
    && mkdir -p ${ROS_WORKSPACE} \
    && chown -R ${ROS_USER}:${ROS_USER} ${ROS_USER_HOME_DIR} \
    && echo "${ROS_USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${ROS_USER}

# Copy the entrypoint script
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Switch to ROS user
USER ${ROS_USER}

# Set workspace
WORKDIR ${ROS_WORKSPACE}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
