#!/bin/bash
set -e

# Source ROS and catkin workspace setup files
source "/opt/ros/noetic/setup.bash"
source "/catkin_ws/devel/setup.bash"

# Execute the command passed to the docker run command
exec "$@" 