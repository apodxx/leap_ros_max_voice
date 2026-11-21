# leap_ros_max_mcu_driver

四轮麦克纳姆轮小车：ESP32-S3 底盘控制器（micro-ROS）+ RK3566 上位机（ROS 2 Jazzy / SLAM / Web 控制台）。

```
┌──────────────┐  WiFi (micro-ROS UDP 8888)   ┌─────────────────────────────┐
│ ESP32-S3     │ <──────────────────────────> │ RK3566 (192.168.0.107)      │
│ 电机/编码器   │   /odom /imu /scan /battery  │ micro-ROS agent             │
│ 麦轮运动学    │   /cmd_vel                   │ EKF + slam_toolbox          │
└──────────────┘                              │ Web 控制台 (vite/foxglove)   │
        ▲                                     └─────────────────────────────┘
        │ USB(仅烧录)                                     ▲
   LDROBOT STL-27L ──/scan(10Hz)──────────────────────────┘
```

## 目录

| 路径 | 说明 |
|---|---|
| `main/` | ESP32 固件（应用层） |
| `components/` | 固件组件（电机、编码器、PID、micro-ROS 封装等） |
| `底层驱动/` | 厂家 STM32 参考驱动（运动学、电机、编码器） |
| `extra_ros_packages/leap_interfaces` | 自定义服务（`/set_speed_pid`、`/get_speed_pid`） |
| `doc/` | 协议、参数适配、**调试记录** |
| `tools/` | 诊断脚本、上位机修改脚本、相机/语音工具 |

## 关键文档

- [`doc/slam_debug_log.md`](doc/slam_debug_log.md) —— **SLAM 建图问题完整排查记录**（现象、实测数据、根因、修复、遗留问题）
- [`doc/firmware_tuning.md`](doc/firmware_tuning.md) —— 固件参数改动与 IDF 5.2.7 构建适配
- [`doc/四驱麦轮底盘参数适配.md`](doc/四驱麦轮底盘参数适配.md) —— 麦轮参数
- [`doc/protocols.md`](doc/protocols.md) —— 通信协议
- [`tools/diagnostics_107/README.md`](tools/diagnostics_107/README.md) —— 诊断脚本用法
- [`tools/host_107/README.md`](tools/host_107/README.md) —— 上位机环境修改脚本

## 当前底盘参数（已按实测标定）

```c
kMotorSigns[4]            = {-1, -1, 1, 1};
kEncoderSigns[4]          = {-1, -1, 1, 1};
kMecanumWheelDiameterMm   = 80.0f;    // 实测 8 cm
kMecanumWheelbaseMm       = 295.0f;   // 实测 29.5 cm
kMecanumTrackWidthMm      = 340.0f;   // 实测 34 cm
kEncoderCountsPerWheelRev = 68000.0f;
```

雷达安装角（`base_link → base_laser`）：

```
0 0 0.18 0 0 -1.6057   # 即 yaw = -92°，实测标定
```

## 快速开始

### 固件（Windows + ESP-IDF 5.2.7）

```powershell
idf.py set-target esp32s3
idf.py build
python -m esptool --chip esp32s3 -p COM4 -b 460800 write_flash -fm dio -ff 80m 0x10000 build\leap_ros_max.bin
```

### 上位机（107）

```bash
systemctl status chuyun-stack        # ROS + Web
systemctl status microros-agent      # micro-ROS agent（独立服务）
# 网页： http://192.168.0.107:5173/
# 重启（保留 agent，主控板不掉线）：
bash /root/restart_stack.sh
```

## 已知问题

- **麦轮原地转向打滑**：地砖上轮子空转，里程计高估转角（实测 24° vs 实际 0.5°），会导致建图重影。
  目前措施：`cmd_vel_shaper.py` 限幅并把原地转改成弧线；根治需修复 IMU 陀螺仪。
- IMU 角速度符号错误，已从 EKF 融合中移除。
- `gmapping` 在 ARM 上段错误，不可用。
