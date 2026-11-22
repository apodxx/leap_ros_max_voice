#!/bin/bash
# Start the chuyun ROS 2 stack and the Ros_Vue_arm web console.
# Idempotent: already-running parts are left untouched.

source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
source /root/ros2_ws/install/local_setup.bash
source /root/micro_ros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0

# 1) micro-ROS agent (UDP 8888)
if ! pgrep -f '[m]icro_ros_agent.*udp4' >/dev/null 2>&1; then
  echo "starting micro-ROS agent"
  setsid nohup ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4 \
    > /root/agent.log 2>&1 < /dev/null &
else
  echo "micro-ROS agent already running"
fi
sleep 3

# 1.5) 主控只在 micro-ROS 任务重启时才会重新连 agent，没连上就踢一下
sleep 4
if ! grep -q 'session established' /root/agent.log 2>/dev/null; then
  AGENT_IP=$(hostname -I | awk '{print $1}')
  echo "no micro-ROS session yet, kicking board (agent $AGENT_IP)"
  for ip in 192.168.0.104 192.168.0.100; do
    curl -s -m 3 -X POST -H 'Content-Type: application/x-www-form-urlencoded' \
      --data "comm_mode=micro_ros&chassis_type=mecanum&microros_agent_ip=$AGENT_IP&microros_agent_port=8888" \
      "http://$ip/api/runtime-config" >/dev/null 2>&1 && break
  done
  sleep 4
fi


# 1.8) cmd_vel 整形（限幅 + 原地转改弧线，抗打滑）
if ! pgrep -f '[c]md_vel_shaper.py' >/dev/null 2>&1; then
  echo "starting cmd_vel_shaper"
  setsid nohup python3 /root/ros2_web/cmd_vel_shaper.py > /root/cmd_vel_shaper.log 2>&1 < /dev/null &
else
  echo "cmd_vel_shaper already running"
fi

# 1.9) scan normalizer: karto/slam_toolbox requires a CONSTANT point count
# 地图中继：导航模式下 /map 只发一次（latched），网页用 volatile 订阅收不到
if ! pgrep -f '[m]ap_relay.py' >/dev/null 2>&1; then
  setsid nohup python3 /root/ros2_web/map_relay.py > /root/map_relay.log 2>&1 &
fi

if ! pgrep -f '[s]can_normalizer.py' >/dev/null 2>&1; then
  echo "starting scan_normalizer"
  setsid nohup python3 /root/ros2_web/scan_normalizer.py \
    > /root/scan_normalizer.log 2>&1 < /dev/null &
else
  echo "scan_normalizer already running"
fi

# 2) chassis bringup: robot_state_publisher, joint_state_publisher, EKF
if ! pgrep -f '[c]huyun_bringup.launch.py' >/dev/null 2>&1; then
  echo "starting chuyun_bringup"
  setsid nohup ros2 launch chuyun_bringup chuyun_bringup.launch.py \
    > /root/chuyun_bringup.log 2>&1 < /dev/null &
else
  echo "chuyun_bringup already running"
fi

# 2.5) Astra camera + host MJPEG stream (for the web console "host camera" tab)
if ! pgrep -f 'component_[c]ontainer' >/dev/null 2>&1; then
  echo "starting astra camera"
  setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; exec ros2 launch astra_camera astra_pro.launch.py' \
    > /tmp/astra107.log 2>&1 < /dev/null &
  sleep 10
else
  echo "astra camera already running"
fi
if ! pgrep -f 'camera_host_[s]tream.py' >/dev/null 2>&1; then
  echo "starting host camera stream (8081)"
  setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/local_setup.bash; exec python3 -u /root/ros2_web/camera_host_stream.py --port 8081' \
    > /root/camera_host_stream.log 2>&1 < /dev/null &
  sleep 3
else
  echo "host camera stream already running"
fi

# 3) web telemetry helpers
for node in web_odom_throttle web_tf_aggregator; do
  if ! pgrep -f "[r]os2 run chuyun_bringup $node" >/dev/null 2>&1; then
    echo "starting $node"
    setsid nohup ros2 run chuyun_bringup "$node" \
      > "/root/${node}.log" 2>&1 < /dev/null &
  else
    echo "$node already running"
  fi
done
sleep 3

# 4) web console: backend 3000 + foxglove 8765 + frontend 5173
if ! pgrep -f '[s]erver.js' >/dev/null 2>&1; then
  echo "starting Ros_Vue_arm console"
  cd /root/ros2_web || exit 1
  setsid nohup bash start_console.sh > /root/ros2_web_console.log 2>&1 < /dev/null &
else
  echo "Ros_Vue_arm console already running"
fi

# 5) AI voice module: slow /cmd_vel nudges, auto-stop after hold
if [ -e /dev/voice ]; then
  if ! pgrep -f '[v]oice_cmd_node.py' >/dev/null 2>&1; then
    echo "starting voice_cmd_node"
    cd /root/ai_voice_module || exit 1
    # 必须走 /cmd_vel_raw：直发 /cmd_vel 会绕过符号翻转与最小速度补偿（0.42 保底）
    setsid nohup python3 voice_cmd_node.py --ros-args \
      -r /cmd_vel:=/cmd_vel_raw \
      -p port:=/dev/voice -p linear_speed:=0.50 -p angular_speed:=0.80 -p hold:=1.5 \
      > /root/voice_cmd.log 2>&1 < /dev/null &
  else
    echo "voice_cmd_node already running"
  fi
else
  echo "voice module /dev/voice not present; skip voice control (see udev rule 1-1.4.1)"
fi


# 6) LDROBOT laser lidar (STL27L @ /dev/ttyUSB1 -> /scan)
if [ -e /dev/lidar ]; then
  if ! pgrep -f '[l]dlidar' >/dev/null 2>&1; then
    echo "starting ldlidar"
    setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash; source /root/lidar_sdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/ros2_app/install/local_setup.bash; exec ros2 launch ldlidar stl27l.launch.py' \
      > /root/lidar_launch.log 2>&1 < /dev/null &
    sleep 3
  else
    echo "ldlidar already running"
  fi
else
  echo "lidar /dev/ttyUSB1 not present; skip"
fi

echo "start command issued"
