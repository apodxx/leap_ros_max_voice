#!/bin/bash
export LC_ALL=C
# 先看两个 tty 的物理路径
for d in /dev/ttyUSB0 /dev/ttyUSB1; do
  echo "== $d"
  udevadm info -q property -n "$d" 2>/dev/null | grep -E '^DEVPATH='
done

cat > /etc/udev/rules.d/99-robot-usb.rules <<'EOF'
# 激光雷达（USB 口 1-1.3）
SUBSYSTEM=="tty", KERNELS=="1-1.3", SYMLINK+="lidar", MODE="0666"
# 语音模块（USB 口 1-1.1.1）
SUBSYSTEM=="tty", KERNELS=="1-1.1.1", SYMLINK+="voice", MODE="0666"
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
sleep 2
echo "== symlinks =="
ls -l /dev/lidar /dev/voice 2>/dev/null
