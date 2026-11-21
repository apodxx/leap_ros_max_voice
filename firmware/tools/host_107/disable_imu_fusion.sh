#!/bin/bash
export LC_ALL=C
P=/root/chuyun_ws/install/chuyun_bringup/share/chuyun_bringup/config/ekf.yaml
cp "$P" "$P.bak2" 2>/dev/null
python3 - "$P" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()
new_cfg = """    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, false]"""
s2, n = re.subn(r"    imu0_config: \[[\s\S]*?\]", new_cfg, s, count=1)
if n == 0:
    print("PATTERN NOT FOUND")
    sys.exit(1)
open(p, "w").write(s2)
print("imu fusion disabled")
PY
sed -n '/imu0_config/,/imu0_remove/p' "$P"

echo "== restart stack =="
systemctl restart chuyun-stack.service
sleep 45
systemctl is-active chuyun-stack.service
ss -ltn | grep -E ':(3000|5173|8765|8081)'
pgrep -af '[f]oxglove_bridge' | head -1
