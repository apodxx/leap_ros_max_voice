#!/usr/bin/env python3
"""cmd_vel 整形：限幅 + 原地转时自动加一点前进，形成弧线（抗打滑）"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MAX_VX = 0.85          # m/s（低于 0.2 会进电机死区不动）
# 主控固件的 x/y/ω 约定与 ROS 整体相反（实测 +vx 会后退、+wz 会右转）。
# 所有指令在下发前统一乘这个系数；里程计反馈在 ekf_input_adapter 里同步取反。
AXIS_SIGN = -1.0
MAX_WZ = 1.20          # rad/s
ARC_VX = 0.12          # 原地转时附加的前进速度
INPLACE_VX_THRESHOLD = 0.02
INPLACE_WZ_THRESHOLD = 0.05


class Shaper(Node):
    def __init__(self):
        super().__init__("cmd_vel_shaper")
        self.sub = self.create_subscription(Twist, "/cmd_vel_raw", self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("cmd_vel shaper: /cmd_vel_raw -> /cmd_vel")

    def on_cmd(self, msg):
        out = Twist()
        vx = max(-MAX_VX, min(MAX_VX, msg.linear.x))
        vy = max(-MAX_VX, min(MAX_VX, msg.linear.y))
        wz = max(-MAX_WZ, min(MAX_WZ, msg.angular.z))

        # 原地转 -> 改成弧线
        if abs(vx) < INPLACE_VX_THRESHOLD and abs(wz) > INPLACE_WZ_THRESHOLD:
            vx = ARC_VX if wz > 0 else -ARC_VX

        # ROS 坐标系先算好（含上面的弧线整形），再整体转换到主控坐标系
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
