#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
 
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Build the project
colcon build

# Source the base workspace
source install/setup.bash

# Execute the command passed into this entrypoint
exec "$@"