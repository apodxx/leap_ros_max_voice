# SLAM 建图问题排查记录（2026-11）

本文记录四轮麦轮小车「激光扫描不贴墙 / 车头朝向错误 / 地图乱开新路」问题的完整排查过程、实测数据、根因与解决方案。

---

## 1. 系统组成

| 角色 | 硬件 | 地址 | 说明 |
|---|---|---|---|
| 底盘控制器 | ESP32-S3（Maturo） | 192.168.0.102（曾为 .100/.104） | micro-ROS over WiFi(UDP 8888)，发布 `/odom` `/imu` `/scan`(仅注册) `/battery_state`，订阅 `/cmd_vel` |
| 上位机 | RK3566 / 4 核 / 4GB（Armbian Ubuntu 24.04） | 192.168.0.107 | ROS 2 Jazzy + micro-ROS agent + SLAM + Web 控制台 |
| 激光雷达 | LDROBOT STL-27L（USB 转 CH340，921600） | `/dev/lidar` | 10 Hz，约 2170 点/圈 |
| 深度相机 | Orbbec Astra Pro（UVC） | — | 建图时建议停掉，占用约 1.5 个核 |
| 语音模块 | CI1302（CH340，115200） | `/dev/voice` | — |

软件链路：

```
STL-27L --/scan(10Hz)--> \
ESP32 --/odom(30~55Hz)--> EKF --/odometry/filtered--> slam_toolbox --/map--> Web(foxglove_bridge:8765)
ESP32 --/imu--> EKF(仅角速度，现已关闭)
```

---

## 2. 现象

1. 激光扫描红点与实际墙体不贴合；
2. 地图不沿行进路线延伸，而是「开辟」出一条错误路线；
3. 网页上蓝色三角形（车头）方向与实际行进方向差约 90°；
4. 原地转向时地图重影最严重。

---

## 3. 排查过程、证据与根因

### 3.1 电机/编码器方向（已解决）

**现象**：前进时右后轮忽正忽反、车走不直、转向不动。

**测量**：直接驱动每个轮子 +PWM / −PWM 并读编码器计数（诊断固件 `motor_diag`）。

```
+60% PWM 时编码器计数方向：
  W0(FL) 负   W1(FR) 负   W2(BL) 正   W3(BR) 负
```

**结论**：`+PWM → 计数为负` 表示该轮编码器方向与电机相反，速度环变成正反馈 → 自激振荡。

**修复**（`main/config/motion_config.h`）：

```c
kMotorSigns[4]   = {-1, -1, 1, 1};
kEncoderSigns[4] = {-1, -1, 1, 1};   // BR 那一路最初漏改，是振荡元凶
```

**验证**：前进 0.5 m/s 指令 1.5 s → 位移 0.214 m，侧偏 3 mm，车头偏 −0.8°。

### 3.2 里程计几何与编码器比例（已按实测修正）

```c
kMecanumWheelDiameterMm  = 80.0f;    // 实测 8 cm
kMecanumWheelbaseMm      = 295.0f;   // 实测前后轮中心距 29.5 cm
kMecanumTrackWidthMm     = 340.0f;   // 实测左右轮中心距 34 cm
kEncoderCountsPerWheelRev= 68000.0f; // 参考厂家 STM32 驱动 ENCODER_PPR
```

> 厂家 STM32 参考（`底层驱动/Drivers/User/Inc/mecanum_kinematics.h`）：
> `ROBOT_WHEEL_BASE 0.30f`、`ROBOT_TRACK_WIDTH 0.35f`、`ROBOT_WHEEL_RADIUS 0.04f`、`ENCODER_PPR 68000`。

### 3.3 micro-ROS 断线后无法自动重连（已解决）

**现象**：107 上 agent 一重启，主控板就掉线，必须断电重启主控。

**原因**：UDP 无连接，主控板只会向旧会话发包，固件没有重连逻辑。

**修复**（`main/tasks/network/microros/microros_runtime.cpp`）：在主循环加入 ping 看门狗。

```c
// 每 3 秒 ping 一次 agent，失败就销毁实体，下一轮自动重建会话
if (use_udp) {
    static uint32_t last_ping_ms = 0;
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now_ms - last_ping_ms > 3000U) {
        last_ping_ms = now_ms;
        if (rmw_uros_ping_agent(300, 1) != RMW_RET_OK) {
            destroy_ros_entities();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
    }
}
```

**验证**：重启 agent 服务后 2 秒内自动重连成功（会话数 2→3）。

**配套**：把 agent 独立成 systemd 服务 `microros-agent.service`，重启网页/ROS 节点时不再连带重启 agent。

### 3.4 串口设备名漂移（已解决）

**现象**：雷达时好时坏，日志 `ldlidar communication is abnormal`。

**原因**：USB 重新枚举后，雷达与语音模块的 `ttyUSB0/ttyUSB1` 互换，而配置里写死了 `ttyUSB1`。

**修复**：udev 规则按物理 USB 路径固定名字。

```udev
# /etc/udev/rules.d/99-robot-usb.rules
SUBSYSTEM=="tty", KERNELS=="1-1.3",     SYMLINK+="lidar", MODE="0666"
SUBSYSTEM=="tty", KERNELS=="1-1.1.1",   SYMLINK+="voice", MODE="0666"
```

雷达 launch、语音节点、启动脚本全部改用 `/dev/lidar`、`/dev/voice`。

### 3.5 IMU 角速度符号错误（已解决，已从融合中移除）

**测量**（指令 +1.5 rad/s 转 1.5 s）：

```
里程计 wz = +0.456   IMU wz = -0.431   ← 符号相反
里程计 wz = -0.472   IMU wz = -0.452   ← 两个方向都负
```

**结论**：IMU 陀螺仪 Z 轴角速度不可靠，EKF 融合后朝向被带偏。

**修复**：`ekf.yaml` 中把 `imu0_config` 全部置 false（暂不使用 IMU），后续若要用需先修固件里的陀螺仪轴/符号。

### 3.6 麦轮横向速度 vy 污染位姿（已解决）

麦轮横移靠打滑实现，`/odom` 的 `vy` 极不可靠，送入 EKF 会让位姿横漂。

**修复**：`ekf.yaml` 的 `odom0_config` 关闭 `vy` 融合，只保留 `x, y, yaw, vx`。

### 3.7 雷达安装角（关键，已按实测定标）

**问题**：雷达的 0° 方向与车头方向不一致，TF 里却按 0° 处理。

**定标方法**（可靠、可复现）：

1. 车头**正对**一面墙，距离约 1 m，摆正；
2. 临时把 `base_link → base_laser` 的 TF 置为 yaw=0（并**清掉重复的 static_transform_publisher**）；
3. 用 `/scan` 找最平的一段墙（40° 窗口内标准差最小）：

```
正前方墙：雷达坐标系 +92.0°，距离 1.09 m，std 0.018
⇒ 雷达安装角 = -92.0°（-1.6057 rad）
```

4. 写入 TF 并验证：把该墙点换算到 `base_link` 得到 `(1.090, 0.000)`，角度 0.0° ✅

```
static_transform_publisher 0 0 0.18 0 0 -1.6057 base_link base_laser
```

> 教训：之前多次用「面对墙」测角但**同时存在两个 static_transform_publisher**（一个 0° 一个 82°），测量结果互相矛盾（278° / 91° / 179°）。**定标前必须先清理重复 TF 发布者**。

### 3.8 原地转向打滑（当前主要遗留问题）

**测量方法**：让车原地转约 0.5 rad，用雷达自相关（720 线）算「世界旋转角」，与里程计对比。

```
里程计报告转了 +23.9°
雷达实测车实际只转了 -0.5°
```

**结论**：轮子在地砖上**打滑空转**，里程计严重高估转角。SLAM 拿错误姿态拼扫描 → 墙不贴、地图乱开新路、转向重影。

**已采取措施**：

1. 新增 `cmd_vel_shaper.py`（网页 → `/cmd_vel_raw` → 整形 → `/cmd_vel`）：
   - 限幅 `MAX_VX=0.45 m/s`、`MAX_WZ=0.90 rad/s`；
   - 原地转时附加 `ARC_VX=0.12 m/s` 前进量，自动变成弧线转弯；
2. slam_toolbox 参数调优（见 3.9）。

**根治方案（待做）**：修好 IMU 陀螺仪，用陀螺测真实转角（不受打滑影响）；或在粗糙地面/地毯上建图。

### 3.9 slam_toolbox 参数

`mapper_params_online_async_1hz.yaml` 关键项：

```yaml
map_update_interval: 0.5
minimum_time_interval: 0.3
minimum_travel_distance: 0.3
minimum_travel_heading: 0.35
transform_timeout: 0.5
use_scan_matching: true
do_loop_closing: true
loop_search_maximum_distance: 3.0
link_match_minimum_response_fine: 0.15
correlation_search_space_dimension: 0.5
correlation_search_space_resolution: 0.02
```

### 3.10 Web 控制台显示问题（已解决）

| 问题 | 原因 | 修复 |
|---|---|---|
| 地图不随尺寸变化刷新 | 只在首次 autoFit | 地图尺寸/原点变化时重新 autoFit（考虑视图旋转） |
| 车与地图错位几米 | 地图 1 Hz、车位姿 30 Hz，两者原点不同步 | 绘制时按「渲染原点 − 当前原点」平移图像 |
| 地图卡片无法缩放/居中 | 视图旋转后 autoFit 仍按未旋转尺寸算 | autoFit 按旋转后外接尺寸计算 |
| 想旋转显示 | — | 按 `R` 键每次旋转视图 90°（纯显示） |

### 3.11 服务稳定性（已解决）

| 问题 | 原因 | 修复 |
|---|---|---|
| 重启网页/ROS 后主控掉线 | agent 被一起重启 | agent 独立 systemd 服务 |
| 点「停止建图」整个网页掉线 | 控制台进程一起被终止 | 加 `chuyun-watch.timer` 自愈 |
| 看门狗把启动过程打断（一直 activating） | 栈启动需 45 s，看门狗 30 s 就重启 | 改为「持续 60 s 不可用才重启」，且仅当 unit 处于 active |
| 两个 SLAM 实例抢同一节点/轨迹 | 手动启动 + 网页启动并存 | 统一只用网页启动 |

### 3.12 环境与资源

- 4 核 CPU，Astra 相机容器约 146% CPU → 建图时停掉相机；
- `/scan` 10 Hz、`/odom` 13 ms 延迟、`/scan` 4 ms 延迟，**不是时间同步问题**；
- 雷达静止重复性 1 cm，**雷达数据本身正常**；
- 所处环境较拥挤（四周 0.4～1.5 m 内均有物体），长墙特征少，不利于扫描匹配。

---

## 4. 当前参数汇总

### 固件（`main/config/`）

```c
kMotorSigns[4]            = {-1, -1, 1, 1};
kEncoderSigns[4]          = {-1, -1, 1, 1};
kMecanumWheelDiameterMm   = 80.0f;
kMecanumWheelbaseMm       = 295.0f;
kMecanumTrackWidthMm      = 340.0f;
kEncoderCountsPerWheelRev = 68000.0f;
kPublisherCount           = 6;   // 与现成 libmicroros 的 RMW_UXRCE_MAX_PUBLISHERS 对齐
```

### 上位机 TF

```
map → odom → base_footprint → base_link → base_laser
                              ↑ 静态: 0 0 0.18 0 0 -1.6057
```

- `map → odom`：slam_toolbox 发布
- `odom → base_footprint`：EKF 发布（`publish_tf: true`）

---

## 5. 遗留问题与后续方案

| 问题 | 影响 | 方案 |
|---|---|---|
| **原地转向打滑** | 转角被高估 20~40 倍，地图重影 | ① 建图只用弧线（已实现整形节点）；② 修 IMU 陀螺仪用真实角速度（推荐） |
| IMU 角速度符号/轴错误 | 无法用于融合 | 检查 LSM6DS3 轴映射与符号，在固件里修正后重新启用 `imu0_config` 的 yaw rate |
| `Cartographer` 与 EKF 的 TF 冲突 | 两套都发 odom，不能共存 | 若用 Cartographer：关闭 EKF 的 `publish_tf`，由 Cartographer 提供 `odom → base_footprint` |
| `gmapping` 段错误 | 无法使用 | ARM 上构建问题，暂不修复 |
| 环境特征少 | 扫描匹配易失败 | 在空旷、有长墙的场地建图 |
| 电池 11.5 V 时动力不足 | 8 s 仅走 34 cm，比例失真 | 建图前充满电 |

---

## 6. 复现与验证脚本

见 `tools/diagnostics_107/`：

| 脚本 | 用途 |
|---|---|
| `wall_only.py` | 车正对墙时测雷达安装角（需 TF yaw 先置 0） |
| `rot_truth.py` | 原地转，用雷达自相关测真实转角 vs 里程计 |
| `straight_drift.py` | 直行 2 m，测里程计直线度与 yaw 漂移 |
| `wheel_signs.py` | 前进时四个轮子的反馈速度符号 |
| `dir_matrix.py` | 六方向（前/后/左移/右移/左转/右转）验证 |
| `map_cell_check.py` | 检查车位姿是否落在地图范围内、所在栅格占用值 |
| `scan_probe.py` | `/scan` 频率、点数、量程统计 |
| `repeat_check.py` | 雷达静止重复性 |
| `cmd_vel_shaper.py` | `/cmd_vel_raw → /cmd_vel` 限幅 + 弧线转弯 |

---

## 7. 经验教训

1. **先验证底盘里程计，再调 SLAM**——里程计错的时候，任何 SLAM 参数都救不回来；
2. **同一 TF 只能有一个发布者**——重复的 `static_transform_publisher` 会让定标结果自相矛盾；
3. **USB 串口必须用 udev 固定名字**——重新枚举会导致设备互换；
4. **micro-ROS 不会自动重连**——需要固件侧 ping 看门狗 + agent 常驻；
5. **麦轮原地转极易打滑**——建图尽量用弧线，或用陀螺仪作为角度源。
