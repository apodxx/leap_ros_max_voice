# host_misc — 上位机系统配置（部署到 RK3566）

这里的目录结构**与设备上的绝对路径一致**，便于直接对照/拷贝：

| 本仓库路径 | 设备上的位置 | 说明 |
|---|---|---|
| `root/start_chuyun_stack.sh` | `/root/start_chuyun_stack.sh` | `chuyun-stack.service` 的 ExecStart，幂等启动整栈 |
| `root/restart_stack.sh` | `/root/restart_stack.sh` | 保留 micro-ROS agent 的重启方式（主控不掉线） |
| `etc/udev/rules.d/99-robot-usb.rules` | `/etc/udev/rules.d/99-robot-usb.rules` | 固定 `/dev/lidar`、`/dev/voice` |
| `root/lidar_sdk/.../stl27l.launch.py` | 同路径（雷达 SDK 安装目录） | 已修复的雷达静态 TF（yaw −92°） |
| `deprecated/imu_bias.py` | — | 已废弃的 Python 零偏校正节点，逻辑已下沉到 C++ |

## 部署

```bash
install -m 755 root/start_chuyun_stack.sh /root/
install -m 644 etc/udev/rules.d/99-robot-usb.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger --subsystem-match=tty
systemctl restart chuyun-stack
```

## systemd 单元（参考）

```ini
# /etc/systemd/system/chuyun-stack.service
[Unit]
Description=Chuyun robot stack (micro-ROS agent + chuyun_ws + ros2_web)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=HOME=/root
Environment=ROS_LOG_DIR=/root/.ros/log
ExecStart=/bin/bash /root/start_chuyun_stack.sh
TimeoutStartSec=180

[Install]
WantedBy=multi-user.target
```

> micro-ROS agent 是**独立服务**（`microros-agent.service`），不要和主栈一起重启，
> 否则主控会掉线重连。

## 已知的启动期问题

`robot_state_publisher` 在开机高负载时偶发死锁（`futex_do_wait`），
表现为 TF 树断裂、`odom→base_laser` 无法解析、SLAM 收不到扫描。
处理办法：`systemctl restart chuyun-stack`。
建议给 `chuyun-watch.sh` 增加 TF 健康检查（当前只检查 3000/8765 端口）。
详见 `docs/调试踩坑记录.md` 坑 4。
