#!/bin/bash

CONTAINER=ur_ros2

# Start the ROS2 container
docker compose up -d

xhost +local:docker

docker exec -it $CONTAINER bash