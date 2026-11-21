#!/bin/bash
export LC_ALL=C
P=/root/chuyun_ws/install/chuyun_navigation2/share/chuyun_navigation2/config/mapper_params_online_async_1hz.yaml
cp "$P" "$P.bak2" 2>/dev/null

python3 - "$P" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()

def setp(s, key, val):
    pat = r'^(\s*)%s\s*:.*$' % re.escape(key)
    if re.search(pat, s, flags=re.M):
        return re.sub(pat, r'\g<1>%s: %s' % (key, val), s, count=1, flags=re.M)
    return s.rstrip() + "\n    %s: %s\n" % (key, val)

# 减少在打滑瞬间建节点
s = setp(s, 'minimum_travel_distance', 0.3)
s = setp(s, 'minimum_travel_heading', 0.35)
s = setp(s, 'minimum_time_interval', 0.3)
s = setp(s, 'map_update_interval', 0.5)
# 加强扫描匹配与回环
s = setp(s, 'use_scan_matching', 'true')
s = setp(s, 'do_loop_closing', 'true')
s = setp(s, 'loop_search_maximum_distance', 3.0)
s = setp(s, 'link_match_minimum_response_fine', 0.15)
s = setp(s, 'correlation_search_space_dimension', 0.5)
s = setp(s, 'correlation_search_space_resolution', 0.02)
open(p, 'w').write(s)
print("slam params tuned for slip-tolerant mapping")
PY
grep -E 'minimum_travel|minimum_time_interval|use_scan_matching|do_loop_closing|loop_search_maximum|link_match_minimum|correlation_search_space' "$P"
