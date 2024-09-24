#!/bin/bash

# Source ROS
source /opt/ros/humble/setup.bash
echo "Running: 'source /opt/ros/humble/setup.bash'"

# Compile
colcon build

# Source installation
source install/setup.bash
echo "Running: 'source install/setup.bash'"