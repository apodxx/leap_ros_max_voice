# 诊断脚本（在上位机 107 上运行）

这些脚本用于定位「激光不贴墙 / 朝向错误 / 地图乱开新路」类问题。全部在 RK3566（192.168.0.107）上运行。

## 运行前

```bash
source /opt/ros/jazzy/setup.bash
source /root/chuyun_ws/install/local_setup.bash
source /root/ros2_ws/install/local_setup.bash
source /root/micro_ros_ws/install/local_setup.bash     # 需要发 /cmd_vel 时
export ROS_DOMAIN_ID=0
```

## 脚本清单

| 脚本 | 用途 | 典型输出 |
|---|---|---|
| `scan_probe.py` | `/scan` 频率、点数、量程范围 | `messages in 10s: 87`、`valid points 2074 min 0.32 max 1.49` |
| `repeat_check.py` | 雷达静止重复性（判断雷达好坏） | `static repeat: mean abs diff = 0.011 m` |
| `sector_check.py` | 按 30° 扇区统计距离分布（判断是否被遮挡） | 各扇区 `mean/std/min/max` |
| `wall_only.py` | **雷达安装角定标**：车正对墙、TF yaw 先置 0，输出墙的角度 | `flattest wall: center +92.0 deg` ⇒ TF yaw = −92° |
| `rot_truth.py` | 原地转，用雷达自相关测真实转角，对比里程计（判断打滑） | `odom dyaw=+23.9`，`lidar world-rotation=-0.5` |
| `straight_drift.py` | 直行 2 m，测里程计直线度与 yaw 漂移 | `distance=0.34 m, yaw drift=-0.1 deg` |
| `fused_yaw_check.py` | 对比 `/odom` 与 `/odometry/filtered` 的转角（判断 EKF 是否被 IMU 污染） | `odom dyaw=+49.1, filtered dyaw=+50.3` |
| `wheel_signs.py` | 前进时四个轮子的反馈速度符号 | `FL +74 FR −86 BL −80 BR +127` |
| `dir_matrix.py` | 六方向验证（前/后/左移/右移/左转/右转） | 每个指令对应的 `dx dy dyaw` |
| `map_cell_check.py` | 车位姿是否在地图内、所在栅格占用值 | `cell (52, 1) occupancy -1` |
| `cmd_vel_shaper.py` | `/cmd_vel_raw → /cmd_vel`：限幅 + 原地转改弧线 | 常驻节点 |

## 关键用法

### 雷达安装角定标（最重要）

```bash
# 1) 清掉所有重复的静态 TF，临时置 0
for pid in $(pgrep -f static_transform_publisher); do kill -9 $pid; done
ros2 run tf2_ros static_transform_publisher 0 0 0.18 0 0 0 base_link base_laser &

# 2) 把车头正对墙（约 1 m），运行：
python3 wall_only.py
# 输出：flattest wall: center +92.0 deg  ⇒  TF yaw 设为 -92°

# 3) 设好后验证：该墙点在 base_link 下应为 (x>0, y≈0)
```

### 打滑检测

```bash
python3 rot_truth.py
# odom dyaw 远大于 lidar world-rotation ⇒ 轮子打滑，里程计不可信
```
