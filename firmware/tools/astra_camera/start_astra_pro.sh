#!/bin/bash
# Start Orbbec Astra Pro (depth + RGB + point cloud).
# Works with both Humble (Jetson 103) and Jazzy (107).
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  source /opt/ros/humble/setup.bash
fi
source "$HOME/ros2_ws/install/setup.bash"
exec ros2 launch astra_camera astra_pro.launch.py "$@"
