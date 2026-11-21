#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

BINS = 720


class Rep(Node):
    def __init__(self):
        super().__init__("repeat_check")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = None
        self.count = 0
        self.create_subscription(LaserScan, "/scan", self.cb, qos)

    def cb(self, msg):
        n = len(msg.ranges)
        arr = np.full(BINS, np.nan)
        for i in range(BINS):
            r = msg.ranges[int(i * n / BINS)]
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                arr[i] = r
        self.scan = arr
        self.count += 1

    def grab(self):
        self.scan = None
        for _ in range(60):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None:
                return self.scan.copy()
        return None


def compare(a, b):
    v = ~np.isnan(a) & ~np.isnan(b)
    if v.sum() < 100:
        return None
    return float(np.mean(np.abs(a[v] - b[v]))), int(v.sum())


def main():
    rclpy.init()
    n = Rep()
    t0 = time.time()
    while n.count < 3 and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    a = n.grab()
    time.sleep(3.0)
    b = n.grab()
    if a is None or b is None:
        print("no scan")
        return
    r = compare(a, b)
    print("static repeat: mean abs diff = %.3f m over %d bins" % r)
    # 统计距离分布
    v = ~np.isnan(a)
    vals = a[v]
    print("bins valid: %d/%d" % (v.sum(), BINS))
    for lo, hi in ((0, 0.3), (0.3, 1), (1, 3), (3, 6), (6, 30)):
        c = int(((vals >= lo) & (vals < hi)).sum())
        print("  %4.1f-%4.1f m: %d" % (lo, hi, c))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
