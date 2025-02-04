#!/bin/bash
set -e

DOCKER_IMAGE="ros-noetic-linux:latest"

echo "Building ROS Noetic Docker image for Linux..."

docker build -t $DOCKER_IMAGE -f docker/Dockerfile .

echo "Build completed: $DOCKER_IMAGE"