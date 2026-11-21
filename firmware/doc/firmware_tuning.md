# 固件改动记录（ESP32-S3 / micro-ROS）

工程：`D:\leap_ros_max_mcu_driver-main`（本仓库）
构建环境：ESP-IDF **v5.2.7**（Windows，`D:\esp\v5.2.7\esp-idf`）

---

## 1. 运动参数（`main/config/motion_config.h`）

| 参数 | 原值 | 现值 | 依据 |
|---|---|---|---|
| `kMecanumWheelDiameterMm` | 80 | 80 | 实测轮径 8 cm |
| `kMecanumWheelbaseMm` | 300 | **295** | 实测前后轮中心距 29.5 cm |
| `kMecanumTrackWidthMm` | 350 | **340** | 实测左右轮中心距 34 cm |
| `kEncoderCountsPerWheelRev` | 68000 | **68000** | 与厂家 STM32 驱动 `ENCODER_PPR` 一致 |
| `kMotorSigns[4]` | {-1,-1,1,1} | **{-1,-1,1,1}** | 正转方向与车头一致 |
| `kEncoderSigns[4]` | {-1,1,-1,1} | **{-1,-1,1,1}** | 实测 `+PWM → 计数为正`；**BR 那一路最关键** |

> `kEncoderSigns` 若某一路反了，该轮速度环变成正反馈 → 轮子忽正忽反、车走不直、转向失效。

## 2. ROS 发布者数量（`main/config/ros_config.h`）

```c
static constexpr size_t kPublisherCount = 6;   // 原 5
```

原因：现成可用的 `libmicroros.a` 是以 `RMW_UXRCE_MAX_PUBLISHERS=6` 编译的，固件里的
`static_assert(RMW_UXRCE_MAX_PUBLISHERS == ros_cfg::kPublisherCount, ...)` 会检查相等。

## 3. micro-ROS 自动重连（`main/tasks/network/microros/microros_runtime.cpp`）

在 `microros_runtime_loop()` 的 `spin_once()` 之后加入看门狗：

```c
if (use_udp) {
    static uint32_t last_ping_ms = 0;
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now_ms - last_ping_ms > 3000U) {
        last_ping_ms = now_ms;
        if (rmw_uros_ping_agent(300, 1) != RMW_RET_OK) {
            ESP_LOGW(kTag, "agent ping failed; recreating micro-ROS session");
            destroy_ros_entities();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
    }
}
```

效果：agent 重启 / 107 整机重启后，主控板 2 秒左右自动重连，**不需要再断电重启主控板**。

## 4. 构建环境适配（ESP-IDF 5.2.7）

原工程基于 IDF 6.x + 组件管理器，在 5.2.7 下需要：

1. `main/CMakeLists.txt`：`esp_driver_tsens` → `driver`；各组件 `REQUIRES` 去掉 `esp_driver_pcnt/ledc/gpio`（5.2 里这些还在 `driver` 中）；
2. 组件全部本地化（不走组件管理器）：
   - `managed_components/espressif__led_strip` → `components/led_strip`
   - `managed_components/micro_ros_espidf_component` → `components/micro_ros_espidf_component`
   - `managed_components/espressif__mdns` → `components/mdns`
3. 构建时设置 `IDF_COMPONENT_MANAGER=0`；
4. **直接复用已编译好的 `libmicroros.a`**（来自 `managed_components/micro_ros_espidf_component/`，约 11.9 MB）：
   在组件 `CMakeLists.txt` 中把 `execute_process(... libmicroros.mk ...)` 换成
   `message(STATUS "Using prebuilt libmicroros.a")`，避免在 Windows 上执行 colcon 构建。

## 5. 烧录

```powershell
# 应用分区
python -m esptool --chip esp32s3 -p COM4 -b 460800 --before default_reset --after hard_reset \
  write_flash -fm dio -ff 80m 0x10000 build\leap_ros_max.bin

# 整片恢复（4MB 备份）
python -m esptool --chip esp32s3 -p COM4 -b 460800 write_flash 0x0 firmware_backup_4mb.bin
```

> 烧应用分区不会动 NVS（WiFi 配置、agent IP 都保留）。

## 6. 验证要点（烧录后）

1. 上电 15 s 内 `/battery_state` 有数据、`/odom` 归零；
2. `ros2 topic pub /cmd_vel` 发前进 → 车走直线（1.5 m 内不偏）；
3. 发左移/右移 → 横移方向正确；
4. 发原地转 → 能转（注意地面打滑）；
5. 重启 agent 服务 → 主控 3 s 内自动重连。
