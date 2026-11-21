#!/usr/bin/env python3
"""cmd_vel 整形：限幅 + 原地转时自动加一点前进，形成弧线（抗打滑）"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MAX_VX = 0.25          # m/s
MAX_WZ = 0.70          # rad/s
ARC_VX = 0.10          # 原地转时附加的前进速度
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

        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
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
