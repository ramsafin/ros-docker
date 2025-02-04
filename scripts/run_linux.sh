#!/bin/bash
set -e

DOCKER_IMAGE="ros-noetic-linux:latest"
DOCKER_CONTAINER="ros-noetic-dev-linux"

echo "Running Docker container for Linux..."

docker run -it --rm \
    --net=host \
    -e DISPLAY \
    -v $(pwd)/catkin_ws:/home/ros/catkin_ws \
    --name "${DOCKER_CONTAINER}" \
    $DOCKER_IMAGE
