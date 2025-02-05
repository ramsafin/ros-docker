#!/bin/bash
set -e

# Unset the noninteractive frontend to enable interactive package installation
unset DEBIAN_FRONTEND

# Source the ROS setup file if it exists
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

# Conditionally set LD_LIBRARY_PATH for WSLg compatibility
if [ "$WSL" = "true" ]; then
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH}
fi

# If no arguments are provided, start an interactive bash shell
if [ $# -eq 0 ]; then
    exec bash
else
    # or execute the passed command
    exec "$@"
fi
