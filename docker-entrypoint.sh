#!/bin/bash
set -e

# Source ROS and catkin workspace setup files
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  source "/opt/ros/noetic/setup.bash"
fi

if [ -f "/catkin_ws/devel/setup.bash" ]; then
  source "/catkin_ws/devel/setup.bash"
fi

# Execute the command passed to the docker run command
exec "$@"