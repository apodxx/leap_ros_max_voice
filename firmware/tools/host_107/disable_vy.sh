#!/bin/bash
export LC_ALL=C
P=/root/chuyun_ws/install/chuyun_bringup/share/chuyun_bringup/config/ekf.yaml
python3 - "$P" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
old = """    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]"""
new = """    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  false, false,
                   false, false, false,
                   false, false, false]"""
if old not in s:
    print("PATTERN NOT FOUND")
    sys.exit(1)
open(p, "w").write(s.replace(old, new, 1))
print("vy fusion disabled (mecanum lateral slip is unreliable)")
PY
sed -n '/odom0_config/,/odom0_queue/p' "$P"

systemctl restart chuyun-stack.service
sleep 45
systemctl is-active chuyun-stack.service
ss -ltn | grep -E ':(3000|5173|8765)' | wc -l
