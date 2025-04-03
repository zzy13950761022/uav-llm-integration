#!/bin/bash
########################################
# This script must be run with sufficient privileges (or via sudo)
# Ensure you run the entire script with sudo or adjust sudoers as necessary
########################################

# Ensure the script is executed on a Linux machine
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Error: This script can only be executed on a Linux machine. Aborting..."
    exit 1
fi

# Grant Permission for USB Serial Communication
if sudo chmod 666 /dev/ttyUSB0; then
    echo "Permissions updated for /dev/ttyUSB0"
fi

# Assign an IP Address to the LiDAR
if sudo ip addr add 192.168.0.100/24 dev enp89s0; then
    echo "IP address assigned to LiDAR interface"
fi

# Allow Webcam Communication with the Container
if sudo chmod 666 /dev/video0; then
    echo "Permissions updated for /dev/video0"
fi

# List all input devices so the user can identify the DualShock 4 controller.
echo "Listing all input devices with evtest:"
echo "-----------------------------------------"
sudo evtest --list-devices
echo "-----------------------------------------"

echo "Please review the above list to identify the event device for the DualShock 4 (Wireless Controller)"
echo "Then, manually run the following command with the correct event device number:"
echo "   sudo chmod 666 /dev/input/event*"