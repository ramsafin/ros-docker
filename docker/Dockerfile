# See https://hub.docker.com/u/nvidia
FROM nvidia/cudagl:11.4.1-runtime-ubuntu20.04

# Locale
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV QT_X11_NO_MITSHM=1
ENV DEBIAN_FRONTEND=noninteractive

# NVIDIA runtime for GPU acceleration
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Install tools, ROS and system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    curl \
    wget \
    sudo \
    gnupg2 \
    mesa-utils \
    lsb-release \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -sSL 'http://packages.ros.org/ros.key' | apt-key add - \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosinstall \
    ros-${ROS_DISTRO}-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set user and workspace
ENV ROS_USER=ros
ENV ROS_USER_HOME_DIR=/home/${ROS_USER}
ENV ROS_WORKSPACE=${ROS_USER_HOME_DIR}/catkin_ws

# Add user "ros", create the workspace, and give the user sudo permissions
RUN useradd -m -s /bin/bash ${ROS_USER} \
    && mkdir -p ${ROS_WORKSPACE} \
    && chown -R ${ROS_USER}:${ROS_USER} ${ROS_USER_HOME_DIR} \
    && echo "${ROS_USER}:password" | chpasswd \
    && adduser ${ROS_USER} sudo

# Copy and set up the entrypoint script
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Switch to the ROS user
USER ${ROS_USER}

# Default working directory for ROS workspace
WORKDIR ${ROS_WORKSPACE}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
