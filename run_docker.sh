#!/bin/bash

# default arguments
IMAGE_NAME="ros-noetic:gpu"
CONTAINER_NAME="ros-noetic-dev"

# parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --name) CONTAINER_NAME="$2"; shift ;;
        --image) IMAGE_NAME="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# allow the container to access the X server securely
xhost +SI:localuser:$(whoami)

# run the ROS Noetic container
docker run -it --rm \
    --gpus all \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/catkin_ws:/catkin_ws" \
    --workdir="/catkin_ws" \
    --name="$CONTAINER_NAME" \
    "$IMAGE_NAME"

# revoke access after container exits
xhost -SI:localuser:$(whoami)
