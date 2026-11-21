#!/bin/bash
export LC_ALL=C
C=/root/chuyun_ws/install/chuyun_cartographer/share/chuyun_cartographer/config/chuyun_laser_2d_ekf.lua
python3 - "$C" <<'PY'
import sys
p = sys.argv[1]
lines = open(p).read().splitlines()
fix = {
    'tracking_frame = "base_link"': 'tracking_frame = "base_link"',
    'published_frame = "base_footprint"': 'published_frame = "base_footprint"',
    'provide_odom_frame = true': 'provide_odom_frame = true',
    'use_odometry = false': 'use_odometry = false',
}
out = []
for ln in lines:
    key = ln.strip()
    if key in fix and not key.endswith(','):
        indent = ln[:len(ln) - len(ln.lstrip())]
        ln = indent + fix[key] + ','
    out.append(ln)
open(p, 'w').write("\n".join(out) + "\n")
print("commas fixed")
PY
sed -n '5,22p' "$C"

echo "== restart cartographer =="
pkill -f 'cartographer' 2>/dev/null
sleep 2
source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
setsid nohup ros2 launch chuyun_cartographer cartographer.launch.py configuration_basename:=chuyun_laser_2d_ekf.lua > /root/carto_test.log 2>&1 < /dev/null &
sleep 12
grep -aE 'FATAL|Check failed' /root/carto_test.log | head -2 || echo "no fatal"
echo "== map =="
timeout 8 ros2 topic info /map 2>/dev/null | head -3
echo "== map->odom =="
timeout 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -E 'Translation|Invalid' | head -2
