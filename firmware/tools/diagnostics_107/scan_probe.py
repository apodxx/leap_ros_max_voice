#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class Probe(Node):
    def __init__(self):
        super().__init__("scan_probe")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos)
        self.count = 0
        self.last = None

    def cb(self, msg):
        self.count += 1
        self.last = msg

def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    print("messages in 10s:", n.count)
    m = n.last
    if m:
        print("frame:", m.header.frame_id, "points:", len(m.ranges))
        valid = [r for r in m.ranges if m.range_min < r < m.range_max]
        print("angle_min/max: %.2f/%.2f  range_min/max: %.2f/%.2f" % (m.angle_min, m.angle_max, m.range_min, m.range_max))
        print("valid points:", len(valid), "min %.2f max %.2f" % ((min(valid), max(valid)) if valid else (0, 0)))
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
