#!/usr/bin/env python3
"""TF=0 时，直接在雷达坐标系里找最平的一段墙（=车头正对方向）"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__("wall_only")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.msgs = []
        self.create_subscription(LaserScan, "/scan", self.cb, qos)

    def cb(self, msg):
        if len(self.msgs) < 3:
            self.msgs.append(msg)


def main():
    rclpy.init()
    n = Probe()
    t0 = 0
    import time
    t0 = time.time()
    while len(n.msgs) < 3 and time.time() - t0 < 12:
        rclpy.spin_once(n, timeout_sec=0.2)

    if not n.msgs:
        print("no scan received")
        return

    m = n.msgs[-1]
    rs = np.array(m.ranges, dtype=float)
    ang = m.angle_min + np.arange(len(rs)) * m.angle_increment
    valid = np.isfinite(rs) & (rs > 0.15) & (rs < 12.0)
    rs = np.where(valid, rs, np.nan)
    print("scan points:", len(rs), "valid:", int((~np.isnan(rs)).sum()))

    best = None
    for s in np.arange(0, 2 * math.pi, math.radians(1)):
        mask = (ang >= s) & (ang < s + math.radians(40)) & ~np.isnan(rs)
        if mask.sum() < 30:
            continue
        vals = rs[mask]
        mu, sd = vals.mean(), vals.std()
        if 0.4 < mu < 3.0 and (best is None or sd < best[2]):
            best = (math.degrees(s + math.radians(20)), mu, sd, int(mask.sum()))

    if not best:
        print("no flat wall found in 0.4-3m")
        return

    center, mu, sd, cnt = best
    if center > 180:
        center -= 360
    print("flattest wall: center %+.1f deg, dist %.2f m, std %.3f, n=%d" % (center, mu, sd, cnt))
    print("=> 建议把雷达 TF 的 yaw 设为 %+.1f deg" % (-center))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
