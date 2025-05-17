#!/bin/bash
set -e  # Exit on first error

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

if ! grep -q "source /opt/ros/jazzy/setup.bash" /root/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
  echo "[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash" >> /root/.bashrc
  echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc
  echo "export ROS_IP=\$(hostname -I | awk '{print \$1}')" >> /root/.bashrc
  echo "export ROS_HOSTNAME=\$ROS_IP" >> /root/.bashrc
# (Line removed as it is unnecessary for ROS2)
fi


# Hand over control to the CMD instruction
exec "$@"


