FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install additional dependencies, update rosdep, and clean up
RUN apt-get update && \
    apt-get install -y \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    python3-catkin-tools python3-colcon-common-extensions && \
    # Initialize rosdep
    rosdep update && \
    # Clean up to reduce image size
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Default working directory for ROS workspace
WORKDIR /catkin_ws

CMD ["bash"]