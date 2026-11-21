#!/bin/bash
export LC_ALL=C
cat > /etc/systemd/system/microros-agent.service <<'EOF'
[Unit]
Description=micro-ROS agent (UDP 8888)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment=HOME=/root
Environment=ROS_LOG_DIR=/root/.ros/log
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /root/micro_ros_ws/install/local_setup.bash && exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4'
StandardOutput=append:/root/agent.log
StandardError=append:/root/agent.log
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
# 停掉栈里起的那个 agent
pkill -f '[r]os2 run micro_ros_agent' 2>/dev/null
pkill -f 'micro_ros_agent.*udp4' 2>/dev/null
sleep 2
systemctl enable miroros-agent 2>/dev/null
systemctl enable microros-agent.service 2>/dev/null
systemctl start microros-agent.service
sleep 4
systemctl is-active microros-agent.service
ss -lun | grep 8888
