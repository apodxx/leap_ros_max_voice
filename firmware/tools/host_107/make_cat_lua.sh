#!/bin/bash
export LC_ALL=C
CFG=/root/chuyun_ws/install/chuyun_cartographer/share/chuyun_cartographer/config
SRC="$CFG/chuyun_laser_2d.lua"
DST="$CFG/chuyun_laser_2d_ekf.lua"

python3 - "$SRC" "$DST" <<'PY'
import re, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()
# 使用轮式里程计
s = s.replace('tracking_frame = "laser_frame"', 'tracking_frame = "base_link"')
s = s.replace('published_frame = "laser_frame"', 'published_frame = "base_link"')
s = s.replace('provide_odom_frame = true', 'provide_odom_frame = false')
s = s.replace('use_odometry = false', 'use_odometry = true')
s = s.replace('use_pose_extrapolator = true', 'use_pose_extrapolator = true')
# 放宽实时匹配窗口，容忍麦轮打滑
s = s.replace('TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1',
              'TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3')
s = s.replace('TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1',
              'TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0')
open(dst, 'w').write(s)
print("written:", dst)
PY
grep -nE 'tracking_frame|published_frame|provide_odom_frame|use_odometry|linear_search_window' "$DST"

echo "== test launch 10s =="
source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
timeout 10 ros2 launch chuyun_cartographer cartographer.launch.py configuration_basename:=chuyun_laser_2d_ekf.lua 2>&1 | head -12
