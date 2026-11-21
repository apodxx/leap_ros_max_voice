#!/bin/bash
export LC_ALL=C
P=/root/chuyun_ws/install/chuyun_navigation2/share/chuyun_navigation2/config/mapper_params_online_async_1hz.yaml
python3 - "$P" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()
def setp(s, key, val):
    if re.search(r'^\s*%s:' % key, s, re.M):
        return re.sub(r'^(\s*)%s:.*$' % key, r'\g<1>%s: %s' % (key, val), s, count=1, flags=re.M)
    return s + "\n    %s: %s\n" % (key, val)
s = setp(s, 'minimum_travel_distance', 0.2)
s = setp(s, 'minimum_travel_heading', 0.2)
s = setp(s, 'minimum_time_interval', 0.15)
s = setp(s, 'map_update_interval', 0.5)
open(p, 'w').write(s)
print("params updated")
PY
grep -E 'minimum_travel_distance|minimum_travel_heading|minimum_time_interval|map_update_interval' "$P"
