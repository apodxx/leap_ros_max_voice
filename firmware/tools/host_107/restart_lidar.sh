#!/bin/bash
export LC_ALL=C
pkill -f 'ldlidar/lib/ldlidar/ldlidar' 2>/dev/null
pkill -f 'ros2 launch ldlidar' 2>/dev/null
sleep 1
source /opt/ros/jazzy/setup.bash
source /root/lidar_sdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/ros2_app/install/local_setup.bash
export ROS_DOMAIN_ID=0
setsid nohup ros2 launch ldlidar stl27l.launch.py > /root/lidar_launch.log 2>&1 < /dev/null &
sleep 8
tail -8 /root/lidar_launch.log | tr -cd '[:print:]\n'
