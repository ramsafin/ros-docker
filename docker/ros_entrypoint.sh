#!/bin/bash
set -e

# Source the ROS setup file
source /opt/ros/$ROS_DISTRO/setup.bash

# Source the catkin workspace setup file (if it exists)
if [ -f "${ROS_WORKSPACE}/devel/setup.bash" ]; then
    source "${ROS_WORKSPACE}/devel/setup.bash"
fi

# Execute the command passed to the container
exec "$@"