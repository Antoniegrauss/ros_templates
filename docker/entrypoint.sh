#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
 
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install all dependencies for the project
rosdep update
rosdep install --from-paths src/ --ignore-src --rosdistro ${ROS_DISTRO} -y
 
# Build the project
colcon build

# Source the base workspace
source install/setup.bash

# Execute the command passed into this entrypoint
exec "$@"