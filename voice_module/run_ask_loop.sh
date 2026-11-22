#!/bin/bash
# 免唤醒语音问答：先停掉 voice_cmd_node（占串口和声卡），跑完恢复
set -u
export LC_ALL=C
ROUNDS="${1:-20}"

pkill -f 'voice_cmd[_]node.py' 2>/dev/null && echo "已暂停 voice_cmd_node"
sleep 1

set +u
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/local_setup.bash
set -u

cd /root/ai_voice_module || exit 1
python3 voice_ask_loop.py --rounds "$ROUNDS"
rc=$?

setsid nohup python3 voice_cmd_node.py --ros-args \
  -p port:=/dev/ttyUSB0 -p linear_speed:=0.50 -p angular_speed:=1.50 -p hold:=0.5 \
  > /root/voice_cmd.log 2>&1 < /dev/null &
echo "已恢复 voice_cmd_node"
exit $rc
