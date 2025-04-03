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
sudo chmod 666 /dev/ttyUSB0
echo "Permissions updated for /dev/ttyUSB0"

# Assign an IP Address to the LiDAR
sudo ip addr add 192.168.0.100/24 dev enp89s0
echo "IP address assigned to LiDAR interface"

# Allow Webcam Communication with the Container
sudo chmod 666 /dev/video0
echo "Permissions updated for /dev/video0"

# Identify the DualShock 4 controller event device.
echo "Searching for DualShock 4 (Wireless Controller) event device..."
found_device=""

for device in /dev/input/event*; do
    # Check if the device is Wireless Controller
    device_name=$(sudo evtest --list-devices 2>/dev/null | grep -B1 "Wireless Controller" | head -n1 | awk -F: '{print $1}')
    
    if [[ "$device_name" == *"Wireless Controller"* ]]; then
        found_device="$device"
        echo "Found DualShock 4 device: $device ($device_name)"
        break
    fi
done

if [ -n "$found_device" ]; then
    # Enable Event Access for the identified DualShock 4 Controller
    sudo chmod 666 "$found_device"
    echo "Permissions updated for $found_device"
else
    echo "DualShock 4 (Wireless Controller) device not found. Please run 'sudo evtest' manually to check device assignments"
fi
