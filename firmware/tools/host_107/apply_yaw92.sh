#!/bin/bash
export LC_ALL=C
YAW_DEG=-92.0
YAW_RAD=$(python3 -c "import math;print('%.4f' % math.radians($YAW_DEG))")
echo "target yaw: $YAW_DEG deg = $YAW_RAD rad"

# 1) 更新 launch 文件（以后重启也用这个值）
L=/root/lidar_sdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/ros2_app/install/ldlidar/share/ldlidar/launch/stl27l.launch.py
python3 - "$L" "$YAW_RAD" <<'PY'
import re, sys
p, rad = sys.argv[1], sys.argv[2]
s = open(p).read()
s2 = re.sub(r"arguments=\['0','0','0\.18','0','0','[-0-9.]+','base_link','base_laser'\]",
            "arguments=['0','0','0.18','0','0','%s','base_link','base_laser']" % rad, s)
open(p, 'w').write(s2)
print("launch updated" if s2 != s else "launch unchanged")
PY

# 2) 换成单一的 TF 发布者
for pid in $(pgrep -f 'static_transform_publisher'); do
  kill -9 "$pid" 2>/dev/null
done
sleep 2
source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
setsid nohup ros2 run tf2_ros static_transform_publisher 0 0 0.18 0 0 "$YAW_RAD" base_link base_laser \
  --ros-args -r __node:=base_link_to_base_laser_stl27l > /root/tf_laser.log 2>&1 < /dev/null &
sleep 4

echo "== running tf =="
for pid in $(pgrep -f 'static_transform_publisher'); do
  tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null; echo
done

echo "== verify: wall point (lidar 92deg, 1.09m) in base_link =="
python3 - "$YAW_RAD" <<'PY'
import math, sys
yaw = float(sys.argv[1])
phi = math.radians(92.0)
r = 1.09
x = r*math.cos(phi); y = r*math.sin(phi)
xb = x*math.cos(yaw) - y*math.sin(yaw)
yb = x*math.sin(yaw) + y*math.cos(yaw)
print("base_link coords: x=%.3f y=%.3f  angle=%.1f deg" % (xb, yb, math.degrees(math.atan2(yb, xb))))
PY
