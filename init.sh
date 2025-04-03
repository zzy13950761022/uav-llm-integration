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

# Identify the DualShock 4 controller event device
echo "Searching for DualShock 4 (Wireless Controller) event device..."
found_device=""

for device in /dev/input/event*; do
    # Retrieve the device name
    device_name=$(udevadm info --query=property --name="$device" 2>/dev/null | grep '^NAME=' | cut -d'=' -f2)
    
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