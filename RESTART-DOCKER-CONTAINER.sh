#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
PROJECT="ros2"
SERVICE="${PROJECT}_ros_foxy_development"
CONTAINER="ros_${SERVICE}_1"
echo "PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

################################################################################

# Display GUI through X Server by granting full access to any external client.
xhost +

################################################################################

# Enter the Docker container with a Bash shell (with or without a custom 'roslaunch' command).
docker-compose -p ros -f ./docker/docker-compose.yml up -d --build ${SERVICE}
docker-compose -p ros -f ./docker/docker-compose.yml restart -t 10 ${SERVICE}
docker exec -i -t ${CONTAINER} bash
# Timestamp refresh
