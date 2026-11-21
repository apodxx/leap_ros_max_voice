#!/bin/bash
export LC_ALL=C
cat > /root/watch_console.sh <<'EOF'
#!/bin/bash
# 控制台自愈（改进版）：
# - 只在 chuyun-stack 处于 active 状态时才检查
# - 端口缺失需持续 60 秒才重启，避免打断启动过程
STATE=/tmp/chuyun_watch.state
now=$(date +%s)
state=$(systemctl is-active chuyun-stack 2>/dev/null)
[ "$state" = "active" ] || { rm -f "$STATE"; exit 0; }

if ss -ltn 2>/dev/null | grep -q ':8765 ' && ss -ltn 2>/dev/null | grep -q ':3000 '; then
  rm -f "$STATE"
  exit 0
fi

if [ -f "$STATE" ]; then
  first=$(cat "$STATE" 2>/dev/null || echo "$now")
  if [ $((now - first)) -gt 60 ]; then
    logger -t chuyun-watch "console down >60s, restarting stack"
    rm -f "$STATE"
    systemctl restart chuyun-stack.service
  fi
else
  echo "$now" > "$STATE"
fi
EOF
chmod +x /root/watch_console.sh

cat > /etc/systemd/system/chuyun-watch.service <<'EOF'
[Unit]
Description=Chuyun web console watchdog
After=network-online.target

[Service]
Type=oneshot
ExecStart=/bin/bash /root/watch_console.sh
EOF

systemctl daemon-reload
systemctl enable --now chuyun-watch.timer
sleep 2
systemctl is-active chuyun-watch.timer
ss -ltn | grep -E ':(3000|5173|8765)' | head -3
