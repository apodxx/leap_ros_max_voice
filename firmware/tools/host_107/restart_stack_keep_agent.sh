#!/bin/bash
# 重启 chuyun 栈（保留 micro-ROS agent，不重启主控板）
export LC_ALL=C
echo "== stop service =="
systemctl stop chuyun-stack.service 2>/dev/null
sleep 1

echo "== kill stack processes (keep agent) =="
for pat in '[c]huyun_bringup' '[r]obot_state_publisher' '[e]kf_node' '[e]kf_input_adapter' \
           '[w]eb_odom_throttle' '[w]eb_tf_aggregator' 'component_[c]ontainer' '[c]amera_host_stream' \
           '[s]erver.js' '[f]oxglove_bridge' '[v]ite' '[v]oice_cmd_node' '[l]dlidar' '[s]tatic_transform_publisher' \
           'slam_toolbox'; do
  pkill -f "$pat" 2>/dev/null
done
sleep 3
pkill -9 -f '[f]oxglove_bridge' 2>/dev/null
sleep 1
echo "== remaining =="
pgrep -af '[c]huyun_bringup|[s]erver.js|[l]dlidar|[v]ite' | head -5

echo "== agent (independent service) =="
systemctl is-active microros-agent.service

echo "== start service =="
systemctl start chuyun-stack.service
echo "started, waiting 40s..."
sleep 40
echo "== status =="
systemctl is-active chuyun-stack.service
ss -ltn 2>/dev/null | grep -E ':(3000|5173|8765|8081)' | awk '{print $4}'
pgrep -af '[l]dlidar|[s]erver.js' | head -4
echo "== board session =="
grep -ac 'establish_session' /root/agent.log
