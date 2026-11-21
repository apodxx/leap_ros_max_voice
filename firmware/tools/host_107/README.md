# 上位机（107 / RK3566）修改脚本

本目录保存对 107 上 ROS 2 / Web 控制台的环境修改脚本，便于重装系统后一键恢复，也便于追溯每项改动的目的。

## 目录与作用

| 脚本 | 作用 |
|---|---|
| `udev_fix.sh` | 创建 `/etc/udev/rules.d/99-robot-usb.rules`：`/dev/lidar`、`/dev/voice` 固定串口名 |
| `pin_usb_names.sh` | 把启动脚本、雷达 launch 里的 `ttyUSB*` 全部改成 `/dev/lidar`、`/dev/voice` |
| `agent_service.sh` | 把 micro-ROS agent 独立成 `microros-agent.service`（开机自启、崩溃重启） |
| `restart_stack_keep_agent.sh` | 重启 ROS/网页但**保留 agent**（内容同 `/root/restart_stack.sh`） |
| `fix_watchdog2.sh` | 安装改进版控制台看门狗 `chuyun-watch.timer`（持续 60 s 不可用才重启，避免打断启动） |
| `restart_lidar.sh` | 单独重启雷达节点（带正确环境变量） |
| `disable_imu_fusion.sh` | EKF 关闭 IMU 角速度融合（IMU 符号错误时避免污染朝向） |
| `disable_vy.sh` | EKF 关闭麦轮 `vy` 横向速度融合 |
| `tune_slam*.sh` | slam_toolbox 参数调优（刷新率、TF 超时、最小位移/转角、回环） |
| `apply_yaw92.sh` | 把雷达安装角写入 launch 与运行中的 TF（−92°） |
| `install_shaper.sh` | 安装 `cmd_vel_shaper.py`、把网页 `cmd_vel` 话题改为 `/cmd_vel_raw`，并加入开机脚本 |
| `make_cat_lua.sh` / `fix_carto_comma.sh` | 生成/修复 Cartographer 的 `chuyun_laser_2d_ekf.lua` |
| `fix_map_offset.sh` | 修复网页地图与车位姿因原点不同步造成的「错位几米」 |
| `fix_autofit2.sh` | 修复视图旋转后 autoFit 不生效（重置视图按钮无效） |
| `add_rot_key2.sh` | 网页地图支持按 `R` 键旋转视图 90° |
| `reset_viewrot.sh` | 把视图旋转默认值恢复为 0 |

## 关键配置文件落点（107）

```
/root/chuyun_ws/install/chuyun_navigation2/share/chuyun_navigation2/config/mapper_params_online_async_1hz.yaml
/root/chuyun_ws/install/chuyun_bringup/share/chuyun_bringup/config/ekf.yaml
/root/chuyun_ws/install/chuyun_cartographer/share/chuyun_cartographer/config/chuyun_laser_2d_ekf.lua
/root/lidar_sdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/ros2_app/install/ldlidar/share/ldlidar/launch/stl27l.launch.py
/root/ros2_web/ros_config.json          # 网页话题映射（cmd_vel -> /cmd_vel_raw）
/root/ros2_web/cmd_vel_shaper.py        # 限幅 + 弧线转弯
/root/start_chuyun_stack.sh             # 开机启动脚本
/etc/udev/rules.d/99-robot-usb.rules
/etc/systemd/system/microros-agent.service
/etc/systemd/system/chuyun-watch.{service,timer}
```

> 注意：`ros2_web` 的前端改动（`src/components/SlamMap.vue`、`src/App.vue`）需要 `npm run build` 重新构建后 `vite preview` 才会生效。
