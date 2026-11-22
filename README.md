# leap_ros_max — 四轮麦轮移动机器人（ESP32-S3 + RK3566 + ROS 2 Jazzy + Web）

一套完整的四轮麦克纳姆轮移动机器人实现，包含四个部分：

| 部分 | 目录 | 说明 |
|---|---|---|
| **① 主控固件** | [`firmware/`](firmware/) | ESP32-S3，micro-ROS 客户端，运动学 / PID / 里程计 / IMU |
| **② 上位机 ROS 2** | [`host_ws/src/`](host_ws/src/) | EKF 融合、SLAM 建图、轻量导航、TF 与遥测适配（C++） |
| **③ Web 控制台** | [`web/`](web/) | 浏览器端地图/激光/控制/导航，Node 后端 + Vue 前端 |
| **④ 语音模块** | [`voice_module/`](voice_module/) | CI1302 离线语音识别 → `/cmd_vel`，以及 TTS 播报 |

硬件明细（型号、接口、端口映射、电源、TF、话题表）见
**[`docs/硬件清单.md`](docs/硬件清单.md)**。

**整车硬件成本约 ¥734**（不含电池），大量使用二手件。明细与采购链接见硬件清单。

调试过程中踩过的 26 个坑（症状 / 证据 / 根因 / 修复）见
**[`docs/调试踩坑记录.md`](docs/调试踩坑记录.md)**，逐文件改动见
**[`docs/CHANGELOG.md`](docs/CHANGELOG.md)**。

---

## 硬件概览

```
                    上位机 Panther（黑豹）X2 (RK3566, Armbian/Ubuntu 24.04, ROS 2 Jazzy)
                    192.168.31.104   :5173 网页 :3000 API :8765 foxglove :8888 micro-ROS
                                    │
        ┌───────────────┬───────────┴──────────┬────────────────┐
   USB 激光雷达      USB 语音模块          USB 声卡          WiFi
   STL-27L           CI1302               JMTek             │
   921600bps         115200bps            TTS 播报           │
                                                            ▼
                                   主控 leap_ros_pro 控制板 (ESP32-S3) 192.168.31.117
                                   4× AT8236 + 4× MG370 1:32 (GMR 500ppr)
                                   LSM6DS3 IMU / 电池 ADC / 超声波 / 灯效
```

| 部件 | 型号 |
|---|---|
| 主控 | **leap_ros_pro 控制板**（ESP32-S3，4 MiB Flash，micro-ROS over UDP），¥198 |
| 上位机 | **Panther（黑豹）X2**（RK3566，4 核，3.8 GiB），Armbian 26.08 / Ubuntu 24.04，ROS 2 Jazzy，¥150 |
| 电机 + 底盘 | **趣轮 MG370 1:32** ×4 + GMR 编码器 500 ppr（64000 计数/轮转），¥180 |
| 驱动 | AT8236 ×4 |
| 底盘 | 四轮麦轮，轮径 80 mm，轮距 340 mm，轴距 295 mm |
| 雷达 | **LDROBOT STL-27L**，10 Hz，约 2170 点/圈，¥86（二手） |
| 语音 | **创乐博 AI 大模型语音交互模块**（CI1302），唤醒词「小创小创」，¥120 |
| 电源 | **辰克 5600 mAh 3S 锂电池**，标称 11.1 V |

---

## 仓库结构

```
leap_ros_max/
├── firmware/                 ① ESP32-S3 固件（ESP-IDF 工程）
│   ├── main/                 控制器/运动学/任务/配置
│   ├── components/           驱动（AT8236、编码器、LSM6DS3、WS2812…）
│   ├── managed_components/   micro-ROS 等托管组件
│   └── doc/                  协议、调参、调试日志
│
├── host_ws/src/              ② 上位机 ROS 2 工作空间
│   ├── chuyun_bringup/       EKF、TF 适配器、Web 遥测聚合（C++）
│   ├── chuyun_navigation2/   slam_toolbox 建图 + 轻量导航（lite_navigation_node）
│   ├── chuyun_description/   URDF 与 mesh
│   └── leap_interfaces/      自定义服务 /set_speed_pid、/get_speed_pid
│
├── web/                      ③ Web 控制台
│   ├── src/                  Vue 3 前端（App.vue、组件）
│   ├── server.js             Express 后端 API
│   ├── scan_normalizer.py    雷达点数归一化（/scan → /scan_fixed）
│   ├── cmd_vel_shaper.py     限幅 + 符号翻转 + 最小速度补偿
│   ├── map_relay.py          地图 latched → 1 Hz 重播
│   ├── auto_localize.py      全局扫描匹配定位（7 秒）
│   └── speak.py              中文 TTS 播报
│
├── voice_module/             ④ 语音模块
│   ├── ai_voice_module.py    串口协议封装（AA 55 cmd data FB）
│   ├── voice_cmd_node.py     ROS 节点：语音 → /cmd_vel_raw
│   └── voice_ask*.py         语音问答 / 物体识别联动
│
├── host_misc/                系统配置（按设备路径组织）
│   ├── root/start_chuyun_stack.sh
│   ├── root/fix_microros.sh  一键重连主控
│   ├── etc/udev/rules.d/99-robot-usb.rules
│   └── root/lidar_sdk/.../stl27l.launch.py
│
└── docs/
    ├── 硬件清单.md
    ├── 调试踩坑记录.md       26 条，含实测数据
    └── CHANGELOG.md          逐文件改动
```

---

## ① 主控固件

**职责**：把 ROS 的 `Twist` 变成四个轮子的 PWM，并把编码器和 IMU 变成 `/odom`、`/imu`。

| 模块 | 文件 |
|---|---|
| 运动学（麦轮/差速） | `main/control/mecanum_kinematics.cpp`、`differential_kinematics.cpp` |
| 速度闭环 PID + 里程计 | `main/control/motion_controller.cpp` |
| 控制主循环 | `main/tasks/control/motion_task.cpp` |
| micro-ROS 收发 | `main/tasks/network/microros/*.cpp` |
| 板载配网 HTTP | `main/tasks/network/provision/provision_portal.cpp` |
| 参数 | `main/config/motion_config.h`、`pid_config.cpp` |

**关键参数**（`motion_config.h`）：

```c
kMaxPwm              = 1000    // PWM 满量程
kMotorPwmDeadband    = 200     // 低速推力下限（建议提到 450）
kMaxAccelRpmPerS     = 500     // 加速度限制
kControlLoopDtS      = 0.02    // 50 Hz 控制环
kEncoderCountsPerWheelRev = 68000
```

**PID 在线可调**（存 NVS，不用烧录）：

```bash
ros2 service call /set_speed_pid leap_interfaces/srv/SetSpeedPid '{kp: 12.0, ki: 1.0, kd: 0.0}'
ros2 service call /get_speed_pid leap_interfaces/srv/GetSpeedPid '{}'
```

**编译烧录**（ESP-IDF 5.2.x）：

```bash
idf.py set-target esp32s3
idf.py build
python -m esptool --chip esp32s3 -p COM16 -b 460800 write_flash -fm dio -ff 80m \
  0x10000 build/leap_ros_max.bin
```

---

## ② 上位机 ROS 2

### 数据流

```
/scan ──scan_normalizer──> /scan_fixed ──> slam_toolbox / lite_navigation
                       （定长 2100 点、无 NaN）

/odom ──┐
        ├─ ekf_input_adapter ─> /odom/ekf_input ─┐
/imu  ──┘   （符号+比例标定、去零偏）             ├─> robot_localization ─> odom→base_footprint
                                                │
                          /imu/ekf_input ───────┘
```

### 关键节点（均在本仓库 `host_ws/src/chuyun_bringup/src/`）

| 节点 | 作用 |
|---|---|
| `ekf_input_adapter` | 陀螺运行期零偏估计、轮速比例标定、主控坐标系→ROS 坐标系转换 |
| `web_tf_aggregator` | 把 `/tf` + `/tf_static` 聚合成 `/web/tf`（**网页依赖静态变换**） |
| `web_odom_throttle` | 降频转发 `/odometry/filtered` |

### 建图

```bash
# 通过网页按钮，或：
curl -X POST http://127.0.0.1:3000/api/run_task/mapping
```

`slam_toolbox`（Karto）参数要点：

```yaml
scan_topic: /scan_fixed       # 必须用归一化后的扫描
max_laser_range: 4.5          # 太大 → 自由空间"扇面"淹没墙体
minimum_travel_distance: 0.5
correlation_search_space_dimension: 0.5   # 走廊退化时不要放宽
```

### 导航（Lite）

`lite_navigation_node`：A* 全局规划 + 局部采样规划 + 相关匹配定位，
不依赖 Nav2，资源占用低。

```yaml
max_vx: 1.10        # 实测命令 < 0.40 时轮子推不动，必须给足
max_wz: 1.30
goal_position_tolerance: 0.10
goal_yaw_tolerance: 3.15          # 只看位置，不管朝向
```

节点已 remap `('/cmd_vel', '/cmd_vel_raw')`，与网页/语音走同一条链路。

---

## ③ Web 控制台

### 功能

- 实时地图 / 激光 / 位姿 / 电池 / 路径（Foxglove WebSocket）
- **键盘控制**：`W/S` 前进后退、`A/D` 左右转、`Q/E` 左右移
- **虚拟摇杆**：手机触屏可用
- 建图 / 导航任务启停、地图保存、多点导航
- **一键自动定位**（全局扫描匹配，约 7 秒）
- **多点导航到达播报**（"到达 1 号点"）
- 摄像头流、机械臂面板、语音助手配置

### 启动

```bash
npm install && npm run build
# 由 chuyun-stack 服务托管：server.js(3000) + vite preview(5173) + foxglove_bridge(8765)
```

### 辅助节点

| 脚本 | 作用 |
|---|---|
| `scan_normalizer.py` | `/scan` → `/scan_fixed`，固定 2100 点、NaN 替换为量程 |
| `cmd_vel_shaper.py` | **限幅 + 主控符号翻转 + 最小有效速度补偿 + 原地转改弧线** |
| `map_relay.py` | 导航模式下 `/map` 只发一次（latched），以 1 Hz 重播给 volatile 订阅者 |
| `auto_localize.py` | 全局扫描匹配求位姿，发布 `/initialpose` |
| `speak.py` | edge-tts 在线合成（兜底 espeak-ng），播放到 USB 声卡 |

> **所有速度发布者都必须发 `/cmd_vel_raw`**，发 `/cmd_vel` 会绕过限幅、符号翻转和最小速度补偿。

---

## ④ 语音模块

| 环节 | 实现 |
|---|---|
| 识别 | CI1302 离线识别，串口 `AA 55 <cmd> <data> FB` |
| 唤醒 | 「小创小创」→ `AA 55 03 00 FB` |
| 控制 | `voice_cmd_node.py` 解析帧 → 发布 `/cmd_vel_raw`（持续 `hold` 秒） |
| 播报 | `speak.py`（edge-tts / espeak-ng）→ USB 声卡 |

```bash
python3 voice_cmd_node.py --ros-args \
  -r /cmd_vel:=/cmd_vel_raw \
  -p port:=/dev/voice -p linear_speed:=0.50 -p angular_speed:=0.80 -p hold:=1.5
```

---

## 部署

```bash
# 1. 工作空间
cd host_ws && colcon build --symlink-install

# 2. Web
cd web && npm install && npm run build

# 3. 系统配置
install -m 755 ../host_misc/root/start_chuyun_stack.sh /root/
install -m 755 ../host_misc/root/fix_microros.sh /root/
install -m 644 ../host_misc/etc/udev/rules.d/99-robot-usb.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger --subsystem-match=tty

# 4. 服务
systemctl enable --now microros-agent   # micro-ROS agent（独立服务，重启主栈时不动它）
systemctl enable --now chuyun-stack     # 主栈：bringup + web + 归一化 + 雷达 + 语音 + 中继
```

访问 `http://<上位机IP>:5173/`。

**建图 / 导航任务由网页按钮触发**，不在服务里自动启动。

---

## 常见问题速查

### "发了命令但车不动"

```bash
ros2 topic hz /odom              # ① 没数据 = 主控会话断了
sudo bash /root/fix_microros.sh  #    一键重连（不用断电）
ros2 topic info /cmd_vel         # ② Subscription count 应为 1
ros2 topic echo /cmd_vel --once  # ③ 符号对不对、幅值够不够（≥0.42）
curl http://192.168.31.117/api/status | grep target   # ④ 命令到板子了吗
```

### "网页白屏"

```bash
curl -s http://127.0.0.1:5173/ | wc -c   # 0 = 构建产物被掉电清零
cd /root/ros2_web && npm run build
```

### "导航模式下地图不显示"

`lite_navigation` 只在启动时发一次 `/map`（latched），`map_relay.py` 已负责重播。

### "车在地图上的位置不对"

点网页上的 **「一键自动定位」**。每次重新「开始导航」后都要定位一次。

### "语音模块没反应"

```bash
ls -l /dev/voice     # 不存在 → udev 端口变了（规则里 1-1.1.1 / 1-1.4.1 都写了）
python3 -c "import serial,binascii; s=serial.Serial('/dev/voice',115200,timeout=0.2); print(binascii.hexlify(s.read(64)))"
# 说唤醒词时应出现 aa550300fb
```

---

## 已知限制

1. **麦轮打滑**：编码器测不到轮地相对滑动，平移里程计仍有随机误差；
   当前命令→实际速度效率约 **65%**（PID 调参前只有 35%）。
2. **窄走廊退化**：约 1 m 宽、轴向无特征时，2D 激光无法约束前后位置（数学退化）。
3. **雷达支架必须刚性固定**：手掰雷达能看出激光相对地图滑动时，任何算法都救不了。
4. **低速死区**：命令低于 0.20 m/s（调参后）轮子不转；已用 `MIN_LINEAR=0.42` 兜底。
5. **主控会话不会自愈**：上位机重启后主控不会自动重连，需要 `fix_microros.sh`。
6. 陀螺零偏依赖运行期估计，长时间连续行驶仍会缓慢漂移。

---

## 调试记录索引

`docs/调试踩坑记录.md` 共 26 条，按主题分组：

| 编号 | 主题 |
|---|---|
| 1–5 | 坐标系与数据质量：静态 TF 装错轴、雷达点数抖动、NaN、TF 树断裂、残留进程 |
| 6–10 | 里程计与 EKF：门限冻结 yaw、融合绝对位置、网页缺静态 TF、轴符号相反、轮速比例 |
| 11–14 | 稳定性与性能：陀螺零偏、Python 节点烧 CPU、走廊退化、搜索窗放开适得其反 |
| 15–17 | 硬件：支架未固定、`max_laser_range` 过大、前端按键映射 |
| 18–26 | 语音、micro-ROS 会话、静摩擦死区、latched QoS、掉电白屏、自动定位、多点导航 |
