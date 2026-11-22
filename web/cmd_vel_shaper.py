#!/usr/bin/env python3
"""cmd_vel 整形

三件事：
1) 限幅
2) 主控坐标系转换（AXIS_SIGN，主控 x/y/ω 与 ROS 相反）
3) 最小有效速度 + 原地转改弧线

最小有效速度的来历（实测，见 docs）：
    命令 0.10 / 0.20 / 0.30 m/s  ->  轮子完全不转（静摩擦 + PWM 死区）
    命令 0.40 m/s                ->  轮子 -58~-119 mm/s
    命令 0.55 m/s                ->  轮子 -216~-248 mm/s
导航控制器在接近目标时会把速度降到 0.1~0.2，正好落进这个死区，
于是"发了速度但不走"。这里把非零的线性命令补到门槛以上。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MAX_VX = 1.10          # m/s
MAX_WZ = 1.40          # rad/s
AXIS_SIGN = -1.0       # 主控坐标轴与 ROS 相反，统一在这里翻转

ARC_VX = 0.12          # 原地转时附加的前进速度
INPLACE_VX_THRESHOLD = 0.02
INPLACE_WZ_THRESHOLD = 0.05

# 实测起步门槛：低于这个幅值电机推不动车
MIN_LINEAR = 0.42      # m/s（线速度幅值下限）
MIN_ANGULAR = 0.55     # rad/s（纯转向时的角速度下限）


class Shaper(Node):
    def __init__(self):
        super().__init__("cmd_vel_shaper")
        self.sub = self.create_subscription(Twist, "/cmd_vel_raw", self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            "cmd_vel shaper: /cmd_vel_raw -> /cmd_vel  "
            f"(MAX_VX={MAX_VX}, MAX_WZ={MAX_WZ}, AXIS_SIGN={AXIS_SIGN}, "
            f"MIN_LINEAR={MIN_LINEAR})")

    def on_cmd(self, msg):
        vx = max(-MAX_VX, min(MAX_VX, msg.linear.x))
        vy = max(-MAX_VX, min(MAX_VX, msg.linear.y))
        wz = max(-MAX_WZ, min(MAX_WZ, msg.angular.z))

        lin_mag = math.hypot(vx, vy)
        in_place = lin_mag < INPLACE_VX_THRESHOLD and abs(wz) > INPLACE_WZ_THRESHOLD

        if in_place:
            # 原地转 -> 弧线，规避麦轮原地打滑（这一路不补最小速度）
            vx = ARC_VX if wz > 0 else -ARC_VX
        elif lin_mag > 1e-3 and lin_mag < MIN_LINEAR:
            # 方向保留，幅值补到能起步
            scale = MIN_LINEAR / lin_mag
            vx *= scale
            vy *= scale
            vx = max(-MAX_VX, min(MAX_VX, vx))
            vy = max(-MAX_VX, min(MAX_VX, vy))

        if not in_place and abs(wz) > 1e-3 and abs(wz) < MIN_ANGULAR:
            wz = math.copysign(MIN_ANGULAR, wz)

        out = Twist()
        out.linear.x = AXIS_SIGN * vx
        out.linear.y = AXIS_SIGN * vy
        out.angular.z = AXIS_SIGN * wz
        self.pub.publish(out)


def main():
    rclpy.init()
    n = Shaper()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
