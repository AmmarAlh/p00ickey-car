#!/bin/bash
set -e  # Exit on first error

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

if [ -f "/root/ros_ws/install/setup.bash" ]; then
    # Source the workspace if it exists
    source /root/ros_ws/install/setup.bash
else
    echo "No workspace found at /root/ros_ws/install/setup.bash"
fi

# Set networking-related ROS environment variables at runtime
export ROS_LOCALHOST_ONLY=0
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

# Hand over control to the CMD instruction
exec "$@"


