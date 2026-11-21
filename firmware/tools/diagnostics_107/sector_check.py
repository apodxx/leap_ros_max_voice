#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Chk(Node):
    def __init__(self):
        super().__init__("near_band")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = None
        self.create_subscription(LaserScan, "/scan", self.cb, qos)

    def cb(self, msg):
        if self.scan is None:
            self.scan = msg


def main():
    rclpy.init()
    n = Chk()
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.scan is not None:
            break
    m = n.scan
    r = np.array([x if (m.range_min < x < m.range_max) else np.nan for x in m.ranges])
    a = m.angle_min + np.arange(len(r)) * m.angle_increment
    print("total points:", len(r), "valid:", int((~np.isnan(r)).sum()))
    # 按 30° 扇区统计
    deg = np.degrees(a)
    for s in range(0, 360, 30):
        mask = (deg >= s) & (deg < s + 30) & (~np.isnan(r))
        if mask.sum():
            vals = r[mask]
            print("%3d-%3d deg: n=%3d mean=%.2f std=%.2f min=%.2f max=%.2f" %
                  (s, s + 30, mask.sum(), vals.mean(), vals.std(), vals.min(), vals.max()))
        else:
            print("%3d-%3d deg: none" % (s, s + 30))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
