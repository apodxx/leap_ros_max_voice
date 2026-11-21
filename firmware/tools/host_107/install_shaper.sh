#!/bin/bash
export LC_ALL=C
python3 - <<'PY'
import json
p = '/root/ros2_web/ros_config.json'
d = json.load(open(p))
d.setdefault('topics', {})['cmd_vel'] = '/cmd_vel_raw'
json.dump(d, open(p, 'w'), indent=2, ensure_ascii=False)
print("web cmd_vel -> /cmd_vel_raw")
PY

# 把 shaper 加进开机启动脚本（如果还没有）
S=/root/start_chuyun_stack.sh
if ! grep -q 'cmd_vel_shaper' "$S"; then
python3 - "$S" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
block = """
# 1.8) cmd_vel 整形（限幅 + 原地转改弧线，抗打滑）
if ! pgrep -f '[c]md_vel_shaper.py' >/dev/null 2>&1; then
  echo "starting cmd_vel_shaper"
  setsid nohup python3 /root/ros2_web/cmd_vel_shaper.py > /root/cmd_vel_shaper.log 2>&1 < /dev/null &
else
  echo "cmd_vel_shaper already running"
fi
"""
marker = "# 2)"
idx = s.find(marker)
if idx < 0:
    s = s + block
else:
    s = s[:idx] + block + "\n" + s[idx:]
open(p, 'w').write(s)
print("shaper added to startup script")
PY
fi
grep -n 'cmd_vel_shaper' "$S" | head -3

echo "== start shaper now =="
source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
pkill -f '[c]md_vel_shaper.py' 2>/dev/null
setsid nohup python3 /root/ros2_web/cmd_vel_shaper.py > /root/cmd_vel_shaper.log 2>&1 < /dev/null &
sleep 3
pgrep -af '[c]md_vel_shaper.py' | head -2
tail -2 /root/cmd_vel_shaper.log | tr -cd '[:print:]\n'
