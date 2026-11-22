#!/bin/bash
# 主控 micro-ROS 会话断开时的一键修复（不用断电）
# 现象：/cmd_vel 有数据但车不动；/odom 无数据；ros2 node list 里没有 leap_ros_max
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /root/chuyun_ws/install/local_setup.bash 2>/dev/null
set -u   # 必须在 source 之后：ROS 的 setup.bash 含未定义变量
export ROS_DOMAIN_ID=0
BOARD=${BOARD:-192.168.31.117}

echo "[1/3] 检查会话…"
if timeout 6 ros2 topic hz /odom 2>/dev/null | grep -q 'average rate'; then
  echo "      /odom 正常，会话是通的，无需修复"
  exit 0
fi
echo "      /odom 无数据 → 会话已断"

echo "[2/3] 重新下发 runtime-config（注意：这个接口收表单，不收 JSON）"
curl -s -m 10 -X POST "http://$BOARD/api/runtime-config" \
  -H 'Content-Type: application/x-www-form-urlencoded' \
  --data 'comm_mode=micro_ros&chassis_type=mecanum&microros_agent_ip=192.168.31.104&microros_agent_port=8888'
echo

echo "[3/3] 等待重连（最多 45 秒）…"
for i in $(seq 1 15); do
  sleep 3
  if timeout 5 ros2 topic hz /odom 2>/dev/null | grep -q 'average rate'; then
    echo "      重连成功（$((i*3))s）"
    timeout 8 ros2 topic hz /odom 2>&1 | grep 'average rate' | head -1
    exit 0
  fi
  printf '      %ds …\n' $((i*3))
done
echo "      !! 仍未重连，请给主控断电重启"
exit 1
