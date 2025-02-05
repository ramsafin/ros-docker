#!/bin/bash
set -e

DOCKER_IMAGE="ros-noetic-wsl:latest"

echo "Building ROS Noetic Docker image for WSL..."

docker build --build-arg WSL=true -t $DOCKER_IMAGE -f docker/Dockerfile.noetic .

echo "Build completed: $DOCKER_IMAGE"