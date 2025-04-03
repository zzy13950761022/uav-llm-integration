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

# Identify the DualShock 4 controller event device.
echo "Searching for DualShock 4 (Wireless Controller) event device..."

# Use evtest to list devices and process each device block
event_device=$(sudo evtest --list-devices 2>/dev/null | \
awk 'BEGIN { RS=""; FS="\n" }
  /Wireless Controller/ && !/Touchpad/ && !/Motion Sensors/ {
    for (i=1; i<=NF; i++) {
      if ($i ~ /\/dev\/input\/(event[0-9]+)/) {
        match($i, /\/dev\/input\/(event[0-9]+)/, arr)
        print arr[1]
        exit
      }
    }
  }'
)

# Check if the event device was found
if [[ -n "$event_device" ]]; then
    device_path="/dev/input/$event_device"
    echo "Found DualShock 4 device: $device_path"
    sudo chmod 666 "$device_path"
    echo "Permissions updated for $device_path."
else
    echo "DualShock 4 (Wireless Controller) device not found. Please run 'sudo evtest' manually to check device assignments."
fi