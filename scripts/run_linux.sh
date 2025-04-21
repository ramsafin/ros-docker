#!/bin/bash
set -e

DOCKER_IMAGE="ros-noetic-linux:latest"
DOCKER_CONTAINER="ros-noetic-dev-linux"

echo "Running Docker container for Linux..."

docker run -it --rm \
    --gpus all \
    --net=host \
    --runtime=nvidia \
    -e DISPLAY \
    -e WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/catkin_ws:/home/ros/catkin_ws \
    --name "${DOCKER_CONTAINER}" \
    $DOCKER_IMAGE
