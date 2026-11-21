#!/bin/bash
export LC_ALL=C
# 杀掉我手动起的 vite preview，让 stack 自己管理
pkill -f '[v]ite preview' 2>/dev/null
pkill -f 'npm exec vite' 2>/dev/null
sleep 3
echo "== 5173 freed? =="
ss -ltn | grep 5173 || echo "free"
systemctl restart chuyun-stack.service
echo "restarting, wait 45s..."
sleep 45
systemctl is-active chuyun-stack.service
echo "== ports =="
ss -ltn | grep -E ':(3000|5173|8765|8081)'
echo "== processes =="
pgrep -af '[f]oxglove_bridge' | head -2
pgrep -af 'node server.js' | head -2
pgrep -af '[v]ite preview' | head -2
