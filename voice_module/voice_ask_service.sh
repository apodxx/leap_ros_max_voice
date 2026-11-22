#!/bin/bash
# 常驻语音听写：识别“识别前方物体”后抓图 -> 109 描述 -> 播报
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/local_setup.bash
cd /root/ai_voice_module || exit 1
exec python3 -u voice_ask_loop.py --rounds 0
