# leap_ros_max — 四轮麦轮小车（ESP32-S3 + RK3566 + ROS 2 Jazzy + Web 控制台）

四轮麦克纳姆轮移动底盘的完整实现：ESP32-S3 固件（micro-ROS）、RK3566 上位机
ROS 2 工作空间（EKF / SLAM / 导航）、以及一个基于 Foxglove WebSocket 的浏览器控制台。

本仓库在原始工程基础上做了**大量建图与操控的修复**，完整记录见
[`docs/调试踩坑记录.md`](docs/调试踩坑记录.md)，逐文件改动见
[`docs/CHANGELOG.md`](docs/CHANGELOG.md)。

---

## 1. 系统总览

```
┌──────────────────┐   WiFi (micro-ROS UDP 8888)   ┌──────────────────────────────┐
│ ESP32-S3 主控     │ <───────────────────────────> │ RK3566 上位机 192.168.31.104  │
│ 192.168.31.117    │   /odom /imu /scan /battery   │ micro-ROS agent              │
│ 电机 / 编码器     │   /cmd_vel                    │ EKF → SLAM → Web 控制台       │
│ 麦轮运动学        │                               │ IMU 零偏校正 / 扫描归一化     │
└──────────────────┘                               └──────────────────────────────┘
        ▲                                                        ▲
        │ USB 烧录                                    LDROBOT STL-27L (/scan 10 Hz)
   ESP-IDF 5.2.7                                            浏览器 http://<上位机>:5173
```

### 硬件

| 部件 | 型号 / 参数 |
|---|---|
| 主控 | ESP32-S3，micro-ROS over UDP |
| 上位机 | RK3566 / Armbian 24.04 / ROS 2 Jazzy，4 核 |
| 电机 | MG370 1:32 减速，GMR 编码器 **500 ppr**（×4 正交 × 32 = 64000 计数/轮转） |
| 底盘 | 四轮麦轮，轮径 80 mm，轮距 340 mm，轴距 295 mm |
| 激光雷达 | LDROBOT STL-27L，USB-CH340，921600 bps，10 Hz，约 2170 点/圈 |
| IMU | LSM6DS3（I2C），固件内做互补滤波 |

### 关键坐标变换

```
map ──slam_toolbox──> odom ──EKF──> base_footprint ──URDF──> base_link ──静态──> base_laser
                                                                              (0, 0, 0.18) / yaw -92°
```

---

## 2. 仓库结构

| 路径 | 说明 |
|---|---|
| `firmware/` | ESP32-S3 固件（ESP-IDF 工程）：`main/`、`components/`、`managed_components/`、`doc/`、`tools/` |
| `host_ws/src/chuyun_bringup/` | EKF、TF 适配器、Web 遥测聚合节点（C++） |
| `host_ws/src/chuyun_navigation2/` | slam_toolbox 建图参数与 launch |
| `host_ws/src/chuyun_description/` | URDF 与 mesh |
| `host_ws/src/leap_interfaces/` | 自定义服务 `/set_speed_pid`、`/get_speed_pid` |
| `web/` | 浏览器控制台（Vue 3 + Vite + Foxglove WebSocket），含后端 `server.js` 与 Python 辅助节点 |
| `host_misc/` | 上位机启动脚本、udev 规则、雷达 launch（含已修复的静态 TF） |
| `docs/` | 踩坑记录、改动清单、架构说明 |

> 未纳入仓库的大件：`leap_ros/`（ROS 工作空间副本，91 MB）、`底层驱动/`（厂商 STM32 参考驱动，65 MB）、
> `firmware_bin/`（烧录产物）、OpenNI SDK、各 `build/` 目录。这些不是本次改动的范围。

---

## 3. 上位机部署

```bash
# 依赖：ROS 2 Jazzy + slam_toolbox + robot_localization + foxglove_bridge + rosbag2
colcon build --symlink-install        # 在 host_ws 下
systemctl enable --now microros-agent # micro-ROS agent（独立服务，UDP 8888）
systemctl enable --now chuyun-stack   # 主栈：bringup + web + 扫描归一化 + 雷达

# Web 控制台
cd web && npm install && npm run build
# 访问 http://<上位机IP>:5173
```

`chuyun-stack.service` 执行 `host_misc/root/start_chuyun_stack.sh`，它是幂等的，
依次确保：micro-ROS agent → 扫描归一化 → IMU 零偏校正 → 雷达桥接 → 底盘 bringup →
相机（可选）→ Web 控制台。**建图任务由网页按钮触发**（`/api/run_task/mapping`），不在服务里。

### udev 固定串口名

```bash
# /etc/udev/rules.d/99-robot-usb.rules  （见 host_misc）
SUBSYSTEM=="tty", KERNELS=="1-1.2",     SYMLINK+="lidar", MODE="0666"
SUBSYSTEM=="tty", KERNELS=="1-1.3",     SYMLINK+="lidar", MODE="0666"
SUBSYSTEM=="tty", KERNELS=="1-1.1.1",   SYMLINK+="voice", MODE="0666"
```

---

## 4. 固件编译与烧录

```powershell
idf.py set-target esp32s3
idf.py build
python -m esptool --chip esp32s3 -p COM4 -b 460800 write_flash -fm dio -ff 80m `
  0x10000 build\leap_ros_max.bin
```

主控的 Web 配网页在 `http://192.168.31.117/`，可切换通信方式（micro-ROS / UART）、
底盘类型（麦轮 / 差速）、**micro-ROS Agent 的 IP 和端口**。切换网段后必须在这里改 Agent IP，
否则主控永远连不上上位机。

---

## 5. Web 控制台功能

- 实时地图 / 激光 / 位姿 / 电池（Foxglove WebSocket，端口 8765）
- 键盘控制：`W/S` 前进后退、`A/D` 左右转、`Q/E` 左右移
- **虚拟摇杆**：手机触屏可用，无需键盘
- 建图 / 导航任务启停、地图保存、参数设置
- 速度可调（设置面板，持久化到 localStorage）

话题链路（指令）：`网页 → /cmd_vel_raw → cmd_vel_shaper.py → /cmd_vel → 主控`
整形节点负责限幅、坐标轴符号转换、以及把"原地转"改写成"弧线转"以规避麦轮打滑。

---

## 6. 已知限制

1. **麦轮在光滑地面打滑**：编码器测不到轮地相对滑动，平移里程计仍有随机误差；
2. **窄走廊退化**：约 1 m 宽、沿轴向无特征时，2D 激光无法约束前后位置（数学退化）；
3. **雷达支架必须刚性固定**：手掰雷达能看出激光相对地图滑动时，任何算法都救不了；
4. 陀螺仪零偏依赖运行期估计，长时间连续行驶仍会缓慢漂移。

详细的现象、证据与处理方式见 [`docs/调试踩坑记录.md`](docs/调试踩坑记录.md)。

---

## 7. 常见问题速查

### "发了命令但车不动"

按顺序查（详见 `docs/调试踩坑记录.md` 坑 20/21/22）：

```bash
# ① 板子↔上位机的 micro-ROS 会话还活着吗？
ros2 topic hz /odom            # 没数据 = 会话断了
sudo bash /root/fix_microros.sh   # 一键重连（不用断电）

# ② 主控还在订阅 /cmd_vel 吗？
ros2 topic info /cmd_vel       # Subscription count 应为 1

# ③ 命令的符号和大小对吗？
ros2 topic echo /cmd_vel --once     # 幅值应 >= 0.42 m/s（静摩擦死区）

# ④ 命令到板子了吗？
curl http://192.168.31.117/api/status | grep target
```

### "网页白屏 / 打不开"

```bash
curl -s http://127.0.0.1:5173/ | wc -c    # 0 字节 = 构建产物被清零（掉电导致）
cd /root/ros2_web && npm run build
```

### "导航模式下地图不显示"

`lite_navigation` 只在启动时发一次 `/map`（latched），网页是 volatile 订阅，收不到。
`map_relay.py` 会以 1 Hz 重播（已加入开机自启）。

### "车在地图上的位置不对"

点网页上的 **「一键自动定位」**（全局扫描匹配，约 7 秒）。
每次重新「开始导航」后都需要定位一次——里程计重启后从零开始。

### "语音模块没反应"

```bash
ls -l /dev/voice                    # 不存在 → udev 端口变了（坑 18）
python3 -c "import serial,binascii; s=serial.Serial('/dev/voice',115200,timeout=0.2); print(binascii.hexlify(s.read(64)))"
# 说唤醒词时应出现 aa550300fb
```

---

## 8. 动力调参记录

固件 PID 存在 NVS 里，**可以不改固件在线调**：

```bash
ros2 service call /set_speed_pid leap_interfaces/srv/SetSpeedPid '{kp: 12.0, ki: 1.0, kd: 0.0}'
```

| 指标 | 调之前 (7.5/0/0) | 调之后 (12/1/0) |
|---|---|---|
| 起步门槛 | 0.40 m/s | **0.20 m/s** |
| 命令 0.40 的实际速度 | 95 mm/s | **282 mm/s** |
| 命令→实际效率 | 35% | **65%** |

原因：原 PID 是纯 P，轮子打滑时编码器读数接近目标值，误差变小 → 输出变小 → 更转不动。
`ki` 把这部分稳态误差累积起来，`kp` 提高让同样的误差产生更大推力。

