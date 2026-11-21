#!/bin/bash
export LC_ALL=C
P=/root/chuyun_ws/install/chuyun_navigation2/share/chuyun_navigation2/config/mapper_params_online_async_1hz.yaml
cp "$P" "$P.bak"
sed -i 's/^\(\s*\)map_update_interval:.*/\1map_update_interval: 0.5/' "$P"
sed -i 's/^\(\s*\)transform_timeout:.*/\1transform_timeout: 0.5/' "$P"
sed -i 's/^\(\s*\)minimum_time_interval:.*/\1minimum_time_interval: 0.25/' "$P"
grep -E 'map_update_interval|transform_timeout|minimum_time_interval' "$P"
