# 改动清单（相对原始工程）

按文件列出本次所有改动，以及"为什么改"。踩坑的现象/证据见
[`调试踩坑记录.md`](调试踩坑记录.md)。

---

## 1. ESP32 固件（`firmware/`）

**本次未改动固件。** 所有问题都在上位机侧规避（符号、零偏、雷达外参）。
但建议后续改此处：

| 位置 | 建议 |
|---|---|
| `components/lsm6ds3_driver/lsm6ds3_driver.cpp` `Init()` | `CalibrateGyro()` 应改为"检测到静止才标定"，或运行期重零偏（见坑 11） |
| `main/config/*` `kEncoderCountsPerWheelRev` | 当前 68000，手册理论值 64000，差 6.25%；建议配合轮径实测重新标定（见坑 10） |
| `main/control/mecanum_kinematics.cpp` | 运动学/电机方向与 ROS 整体相反，本次在上位机侧取反规避（见坑 9） |

---

## 2. `host_ws/src/chuyun_bringup`

### 2.1 `config/ekf.yaml`

| 参数 | 原值 | 现值 | 原因 |
|---|---|---|---|
| `odom0_config` | `x, y, yaw, vx` 全 true | **只有 `vx, vy`** | 轮式 yaw 严重失真（原地转 137° 只报 1.5°），绝对位置被它积分成错误轨迹（坑 7） |
| `imu0_config` | 全 false（IMU 被完全禁用） | **只有 `vyaw` = true** | 陀螺是唯一不受打滑影响的转角源（坑 7） |
| `imu0_twist_rejection_threshold` | （默认 10） | **1.0e9** | 门限把 IMU 全部拒掉导致 yaw 冻结（坑 6） |
| `odom0_twist_rejection_threshold` | （默认 10） | **1.0e9** | 同上 |
| `odom0_pose_rejection_threshold` | （默认 10） | **1.0e9** | 同上 |
| `dynamic_process_noise_covariance` | false | **true** | 让过程噪声自适应，避免协方差被一次跳变带崩 |

### 2.2 `launch/chuyun_bringup.launch.py`

```diff
 ekf_input_adapter_node = Node(
     package='chuyun_bringup',
     executable='ekf_input_adapter',
     name='ekf_input_adapter',
-    output='screen'
+    # adapter removes the gyro zero-rate bias and applies the calibrated
+    # wheel-speed scale (measured 3.25 m -> 3.317 m, k = 0.980)
+    parameters=[{'odom_linear_scale': 0.980, 'odom_axis_sign': -1.0}],
+    output='screen'
 )
```
（曾短暂加过 `remappings=[('/imu','/imu_biasfree')]`，把零偏校正放到 Python 节点里，
后因该节点烧 43% CPU 而下沉到 C++ 适配器，映射已移除 —— 见坑 12。）

### 2.3 `src/ekf_input_adapter.cpp`（主要改动之一）

新增三项能力，全部在 C++ 里完成，不增加进程：

```cpp
// 1) 运行期陀螺零偏估计（仅静止时更新，无 /odom 时拒绝更新 = fail safe）
imu_bias_standstill_sec = 2.0   // 静止判定窗口
imu_bias_alpha          = 0.05  // EMA 系数

// 2) 轮速比例标定
odom_linear_scale = 0.980       // 实测 3.25 m 报 3.317 m，+2.05%

// 3) 坐标轴符号转换（主控约定与 ROS 相反）
odom_axis_sign = -1.0
```

`/odom` 回调里先做符号与比例换算，再补协方差：

```cpp
const double lin = odom_linear_scale_ * odom_axis_sign_;
message.twist.twist.linear.x *= lin;
message.twist.twist.linear.y *= lin;
message.twist.twist.angular.z *= odom_axis_sign_;
message.pose.pose.position.x *= lin;
message.pose.pose.position.y *= lin;
```

`/imu` 回调里在静止时更新零偏，然后把零偏从角速度里减掉再发布到 `/imu/ekf_input`。

### 2.4 `src/web_tf_aggregator.cpp`（关键修复）

**问题**：只订阅 `/tf`，导致 `base_link→base_laser`（−92° 安装角）和
`base_footprint→base_link` 这两个**静态变换**从来没送到浏览器，网页激光整体偏 92°（坑 8）。

**改动**：

```diff
+    static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
+      "/tf_static", rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
+      [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
+        for (const auto & transform : message->transforms) {
+          const auto key = std::make_pair(
+            transform.header.frame_id, transform.child_frame_id);
+          static_transforms_[key] = transform;
+        }
+      });
```

快照发布时**动态变换按老化时间过滤，静态变换始终包含并重新打时间戳**：

```cpp
for (const auto & entry : static_transforms_) {
  auto transform = entry.second;
  transform.header.stamp = current_time;   // 避免被下游按年龄过滤掉
  snapshot.transforms.push_back(transform);
}
```

---

## 3. `host_ws/src/chuyun_navigation2`

### `config/mapper_params_online_async_1hz.yaml`

| 参数 | 原值 | 现值 | 原因 |
|---|---|---|---|
| `scan_topic` | `/scan` | **`/scan_fixed`** | 原始扫描点数每帧变化，karto 会 abort（坑 2/3） |
| `max_laser_range` | 12.0 | **4.5** | 无回波射线把自由空间刷到 12 m，地图被扇面淹没（坑 16） |
| `minimum_travel_distance` | 0.3 | **0.5** | 更少的节点 = 更少在退化方向上翻车的机会（坑 14） |
| `minimum_travel_heading` | 0.35 | **0.5** | 同上 |
| `minimum_time_interval` | 0.3 | **0.5** | 同上 |
| `map_update_interval` | 0.5 | **1.0** | 降低 CPU 压力 |
| `correlation_search_space_dimension` | 0.5 | **0.5（曾试 1.0，已回退）** | 放宽搜索窗在走廊退化方向反而更糊（坑 14） |
| `correlation_search_space_resolution` | 0.02 | **0.01** | 回到 slam_toolbox 默认档 |
| `link_match_minimum_response_fine` | 0.15 | **0.15（曾试 0.1，已回退）** | 同上 |

---

## 4. `web/`（浏览器控制台）

### 4.1 新增 `scan_normalizer.py`

订阅 `/scan`，按固定角度栅格重采样成 **2100 点**后发布 `/scan_fixed`：

- 固定点数、固定 `angle_increment = 2π/2100`（解决坑 2）；
- 非有限/非正读数替换为 `range_max`（解决坑 3）；
- 输出 `range_max` 收紧到 12.0（与 slam 的 `max_laser_range` 配套）；
- 由 `start_chuyun_stack.sh` 开机自启。

### 4.2 `cmd_vel_shaper.py`

```diff
-MAX_VX = 0.45
-MAX_WZ = 0.90
+MAX_VX = 0.85
+MAX_WZ = 1.20
+
+# 主控固件的 x/y/ω 约定与 ROS 整体相反（实测 +vx 会后退、+wz 会右转）。
+AXIS_SIGN = -1.0
...
-        out.linear.x = vx
-        out.linear.y = vy
-        out.angular.z = wz
+        # ROS 坐标系先算好（含上面的弧线整形），再整体转换到主控坐标系
+        out.linear.x = AXIS_SIGN * vx
+        out.linear.y = AXIS_SIGN * vy
+        out.angular.z = AXIS_SIGN * wz
```

原有的"原地转 → 弧线"整形保留（`ARC_VX = 0.12 m/s`，抗麦轮打滑）。

### 4.3 `src/App.vue`

- 按键重映射：`A/D` → 左右转，`Q/E` → 左右移（原来相反）；
- 新增 `ROT_SIGN`、`STRAFE_RATIO`、`maxStrafeVelocity()`；
- 新增 `onJoystickVelocity()`，把虚拟摇杆的输入接入同一条 `publishVelocity` 链路；
- 默认速度：线速度 0.55 → **0.70 m/s**，角速度 2.0 → **2.6 rad/s**；
- 速度设置持久化 key：`ros2-web-motion-speeds-v3` → **`v5`**（避免旧值覆盖新默认）。

```js
// 主控的 x/y/ω 符号整体与 ROS 相反，统一在 cmd_vel_shaper.py 里取反，
// 这里保持标准 ROS 方向即可（不要再取反，否则会转两次）。
const ROT_SIGN = 1
const STRAFE_RATIO = 0.65

function onJoystickVelocity(payload) {
  if (!isConnected.value) return
  const linearX = Number(payload?.linearX) || 0
  const angularZ = Number(payload?.angularZ) || 0
  publishVelocity(linearX, 0, ROT_SIGN * angularZ)
}
```

### 4.4 `src/components/JoystickControl.vue`

- 从"已存在但未挂载"变为**实际挂载**到左栏（键盘控制下方）；
- 最大线速度 0.6 → **0.70 m/s**，最大角速度 2.0 → **2.6 rad/s**；
- 支持触屏（`touchstart/touchmove/touchend`），10 Hz 下发，松手立即发零速。

### 4.5 `src/components/KeyboardControl.vue`

按键提示文字更新为新映射：`Q 左移 / W 前进 / E 右移 / A 左转 / S 后退 / D 右转`。

### 4.6 `imu_bias.py`（已废弃）

最初的 Python 零偏校正节点，因占用 43% CPU 且跟不上消息率，
逻辑已下沉到 `ekf_input_adapter.cpp`（坑 12）。保留在 `host_misc/deprecated/` 供参考。

---

## 5. `host_misc/`（上位机系统配置）

### 5.1 `root/start_chuyun_stack.sh`

在原有流程中插入两步（都为幂等判断）：

```bash
# 1.9) scan normalizer
if ! pgrep -f '[s]can_normalizer.py' >/dev/null 2>&1; then
  setsid nohup python3 /root/ros2_web/scan_normalizer.py > /root/scan_normalizer.log 2>&1 &
fi
```

（曾插入过 imu_bias 的启动段，随节点废弃一并移除。）

### 5.2 `etc/udev/rules.d/99-robot-usb.rules`

```diff
-SUBSYSTEM=="tty", KERNELS=="1-1.3", SYMLINK+="lidar", MODE="0666"
+SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="lidar", MODE="0666"
+SUBSYSTEM=="tty", KERNELS=="1-1.3", SYMLINK+="lidar", MODE="0666"
 SUBSYSTEM=="tty", KERNELS=="1-1.1.1", SYMLINK+="voice", MODE="0666"
```

雷达 udev 规则原本写死在 USB 口 `1-1.3`，实际设备枚举在 `1-1.2`，
导致 `/dev/lidar` 从未生成、雷达节点直接跳过。两个口都保留以兼容重新插拔。

### 5.3 `root/lidar_sdk/.../stl27l.launch.py`

```diff
-arguments=['0','0','0.18','0','0','-1.6057','base_link','base_laser']
+# ROS 2 位置参数顺序是 x y z yaw pitch roll —— 原写法把 -92° 装进了 roll
+arguments=['--x','0','--y','0','--z','0.18','--yaw','-1.6057',
+           '--pitch','0','--roll','0','--frame-id','base_link','--child-frame-id','base_laser']
```

---

## 6. 未改动但值得注意

| 文件 | 说明 |
|---|---|
| `foxglove_bridge.params.yaml` | `topic_whitelist` 已含 `/map`、`/scan`、`/web/tf` 等，无需改 |
| `ros_config.json` | 其中 `"cmd_vel": "/cmd_vel_raw"` 是网页指令进入整形节点的入口，**不要改成 `/cmd_vel`**，否则绕过限幅与符号转换 |
| `ekf_input_adapter` 的协方差 | 轮式 x/y 用 0.01（σ=0.1 m），IMU 角速度 0.0025（σ=0.05 rad/s） |

---

## 7. 验证用的度量方法（可复现）

本次调试建立了几套可量化的检查，脚本思路如下（都在上位机上跑）：

| 指标 | 方法 | 健康值 |
|---|---|---|
| 激光贴图率 | 取一帧 `/scan_fixed`，按 TF 投到 map，统计落在占用栅格 ±1 格内的比例 | 静止 >99%，移动 >50% |
| `map→odom` 修正量 | `tf2_echo map odom` | 越小越好，健康 <0.2 m / <2° |
| EKF yaw 是否冻结 | 录 bag，逐窗口对比 EKF yaw 变化 vs 陀螺积分 vs 雷达互相关 | 三者一致（比值 1.00±0.05） |
| 轮速比例 | 量 3~4 m 直线，对比 `∫vx dt` 与真实距离 | 误差 <2% |
| 地图自洽性 | 旋转偏移扫描：把当前扫描绕机器人旋转 −180°~+180°，看最佳贴合角 | 最佳角应在 0° 附近且有尖峰 |

> 注意：`/map` 是 `transient_local`，订阅时必须用对应 QoS，否则会以为"没有地图"。

---

# 续：语音控制、自动定位与动力调参（2026-11-22 下午）

## 新增文件

| 文件 | 位置 | 作用 |
|---|---|---|
| `auto_localize.py` | `web/` | 全局扫描匹配定位，7 秒出结果，发布 `/initialpose` |
| `map_relay.py` | `web/` | 把 latched 的 `/map` 以 1 Hz 重播，供 volatile 订阅者（网页）使用 |
| `fix_microros.sh` | `host_misc/root/` | 主控 micro-ROS 会话断开时一键重连（无需断电） |

## `web/cmd_vel_shaper.py`

```diff
-MAX_VX = 0.45
-MAX_WZ = 0.90
+MAX_VX = 1.30
+MAX_WZ = 1.40
+
+AXIS_SIGN = -1.0     # 主控 x/y/ω 与 ROS 相反
+
+# 实测：命令 < 0.40 m/s 时轮子完全不转（静摩擦 + PID 纯 P）
+MIN_LINEAR = 0.42    # 非零线速度命令幅值补到这个值（保留方向）
+MIN_ANGULAR = 0.55
```

新增"最小有效速度"逻辑：线速度幅值非零但低于 `MIN_LINEAR` 时按比例放大，
避免控制器在接近目标时发出"推不动车"的小速度。

## `web/server.js`

新增 `POST /api/auto_localize`：spawn `auto_localize.py`，把 JSON 结果回传前端。

## `web/src/App.vue`

- 线速度默认 0.45 → **1.10 m/s**，角速度 2.0 → 2.6
- 按键重映射：`A/D` = 左右转，`Q/E` = 左右移
- 新增虚拟摇杆接入 `onJoystickVelocity`
- 速度设置持久化 key：v3 → **v7**

## `web/src/components/NavigationControl.vue`

新增 **「一键自动定位」** 按钮：调用 `/api/auto_localize`，回显得分与坐标。

## `web/src/components/JoystickControl.vue`

`maxLinearSpeed` 0.6 → **1.10**，`maxAngularSpeed` 2.0 → 2.6。

## `host_ws/src/chuyun_navigation2/launch/lite_navigation.launch.py`

```diff
 Node(
     package='chuyun_navigation2',
     executable='lite_navigation_node',
     name='lite_navigation',
+    # 主控的 x/y/ω 与 ROS 相反，符号翻转在 cmd_vel_shaper 里统一处理，
+    # 导航必须走 /cmd_vel_raw 才能享受同样的限幅与符号转换。
+    remappings=[('/cmd_vel', '/cmd_vel_raw')],
     ...
```

## `host_ws/src/chuyun_navigation2/config/lite_navigation_omni.yaml`

| 参数 | 原值 | 现值 | 原因 |
|---|---|---|---|
| `max_vx` | 0.18 | **1.10** | 0.18 落在静摩擦死区里，永远推不动车 |
| `max_vy` | 0.16 | 0.70 | 同上（横移） |
| `max_reverse_speed` | 0.08 | 0.70 | 同上（倒车） |
| `max_wz` | 0.70 | 1.30 | 转向 |
| `max_xy_speed` | 0.18 | 1.10 | 合成速度上限 |
| `max_ax` / `max_ay` / `max_aw` | 0.35 / 0.30 / 1.00 | 1.60 / 1.30 / 2.50 | 不放开加速度就到不了这个速度 |
| `goal_yaw_tolerance` | 0.12 | **3.15** | 多点导航时不想每点都转身 |

## `host_misc/etc/udev/rules.d/99-robot-usb.rules`

```diff
 # Voice module CI1302 (CH340, 115200) on USB port 1-1.1.1
 SUBSYSTEM=="tty", KERNELS=="1-1.1.1", SYMLINK+="voice", MODE="0666"
+# 语音模块这次插在 1-1.4.1，两条规则都保留
+SUBSYSTEM=="tty", KERNELS=="1-1.4.1", SYMLINK+="voice", MODE="0666"
```

## `host_misc/root/start_chuyun_stack.sh`

新增两段（都带幂等判断）：

```bash
# 地图中继：导航模式下 /map 只发一次（latched），网页用 volatile 订阅收不到
if ! pgrep -f '[m]ap_relay.py' >/dev/null 2>&1; then
  setsid nohup python3 /root/ros2_web/map_relay.py > /root/map_relay.log 2>&1 &
fi
```

语音块补上 remap（**关键**）并调参：

```diff
-    setsid nohup python3 voice_cmd_node.py --ros-args \
-      -p port:=/dev/voice -p linear_speed:=0.50 -p angular_speed:=1.50 -p hold:=0.5 \
+    setsid nohup python3 voice_cmd_node.py --ros-args \
+      -r /cmd_vel:=/cmd_vel_raw \
+      -p port:=/dev/voice -p linear_speed:=0.50 -p angular_speed:=0.80 -p hold:=1.5 \
```

## 固件参数（**通过 ROS 服务在线修改，未重新烧录**）

```bash
ros2 service call /set_speed_pid leap_interfaces/srv/SetSpeedPid \
  '{kp: 12.0, ki: 1.0, kd: 0.0}'
```

原值 `kp=7.5, ki=0, kd=0`（纯 P，打滑后永远补不上稳态误差）。
参数存在 NVS（键 `speed_v2`），断电/重启不丢。

**效果**：起步门槛 0.40 → 0.20 m/s，命令→实际速度效率 35% → 65%。

## 待做（需要烧录固件）

`main/config/motion_config.h`：

```c
kMotorPwmDeadband = 200  →  450      // 低速时给更大推力
kMaxAccelRpmPerS  = 500  →  800
```

编译：`D:\esp\v5.2.6` 的 ESP-IDF，`idf.py build`；
烧录：`python -m esptool --chip esp32s3 -p COM16 -b 460800 write_flash -fm dio -ff 80m 0x10000 build\leap_ros_max.bin`

