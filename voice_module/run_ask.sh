#!/bin/bash
# 语音问答模式：暂停底盘语音控制 -> 等唤醒词 -> 抓图 -> 109 识别 -> 播报 -> 恢复语音控制
set -u
export LC_ALL=C
SECS="${1:-90}"

pkill -f 'voice_cmd[_]node.py' 2>/dev/null && echo "已暂停 voice_cmd_node"
sleep 1

set +u
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/local_setup.bash
set -u
cd /root/ai_voice_module || exit 1

python3 ask_object.py --wait --seconds "$SECS"
rc=$?

# 恢复底盘语音控制
setsid nohup python3 voice_cmd_node.py --ros-args \
  -p port:=/dev/ttyUSB0 -p linear_speed:=0.50 -p angular_speed:=1.50 -p hold:=0.5 \
  > /root/voice_cmd.log 2>&1 < /dev/null &
echo "已恢复 voice_cmd_node"
exit $rc
