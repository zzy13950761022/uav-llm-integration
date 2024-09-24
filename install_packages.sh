#!/bin/bash

# Update the system package index
echo "Updating package index..."
sudo apt update

# Install the required ROS 2 Humble packages
echo "Installing ROS 2 Humble packages..."
sudo apt install -y \
  ros-dev-tools \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-joint-state-publisher \
  ros-humble-rqt-robot-steering

# Confirm that the packages were installed successfully
if [[ $? -eq 0 ]]; then
  echo "Installation completed successfully!"
else
  echo "Installation failed. Please check for errors."
fi