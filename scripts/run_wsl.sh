#!/bin/bash
set -e

DOCKER_IMAGE="ros-noetic-wsl:latest"
DOCKER_CONTAINER="ros-noetic-dev-wsl"

echo "Running Docker container for WSL..."
docker run -it --rm \
    --gpus all \
    --net=host \
    --runtime=nvidia \
    --device="/dev/dxg" \
    -e DISPLAY \
    -e WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR \
    -v /mnt/wslg:/mnt/wslg \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/catkin_ws:/home/ros/catkin_ws \
    --name "${DOCKER_CONTAINER}" \
    $DOCKER_IMAGE
