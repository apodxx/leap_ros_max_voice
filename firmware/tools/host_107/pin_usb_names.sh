#!/bin/bash
export LC_ALL=C
S=/root/start_chuyun_stack.sh
L=/root/lidar_sdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/ros2_app/install/ldlidar/share/ldlidar/launch/stl27l.launch.py

# 1) 栈脚本里改用固定符号链接
sed -i 's|-p port:=/dev/ttyUSB0|-p port:=/dev/voice|' "$S"
sed -i 's|if \[ -e /dev/ttyUSB0 \]; then|if [ -e /dev/voice ]; then|' "$S"
sed -i 's|if \[ -e /dev/ttyUSB1 \]; then|if [ -e /dev/lidar ]; then|' "$S"
grep -n 'dev/voice\|dev/lidar' "$S" | head -6

# 2) 雷达 launch 也改用符号链接
sed -i "s|'/dev/ttyUSB0'|'/dev/lidar'|; s|'/dev/ttyUSB1'|'/dev/lidar'|" "$L"
grep -n "port_name" "$L"

# 3) 重启雷达
pkill -f 'ldlidar/lib/ldlidar/ldlidar' 2>/dev/null
pkill -f 'ros2 launch ldlidar' 2>/dev/null
sleep 1
source /opt/ros/jazzy/setup.bash
source /root/lidar_sdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/ros2_app/install/local_setup.bash
export ROS_DOMAIN_ID=0
setsid nohup ros2 launch ldlidar stl27l.launch.py > /root/lidar_launch.log 2>&1 < /dev/null &
sleep 8
tail -3 /root/lidar_launch.log | tr -cd '[:print:]\n'
