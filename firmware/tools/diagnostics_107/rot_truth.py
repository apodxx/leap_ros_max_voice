#!/usr/bin/env python3
"""原地转 ~90°，用雷达自相关测真实转角，和里程计对比"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

BINS = 720


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("rot_truth")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = None
        self.odom_yaw = None
        self.create_subscription(LaserScan, "/scan", self.cb, qos)
        self.create_subscription(Odometry, "/odom", self.cb_odom, qos)

    def cb(self, msg):
        n = len(msg.ranges)
        arr = np.full(BINS, np.nan)
        for i in range(BINS):
            r = msg.ranges[int(i * n / BINS)]
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                arr[i] = r
        self.scan = arr

    def cb_odom(self, msg):
        self.odom_yaw = quat_yaw(msg.pose.pose.orientation)

    def grab(self):
        self.scan = None
        for _ in range(60):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None:
                return self.scan.copy()
        return None

    def spin(self, s=0.2):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, wz, seconds):
        m = Twist()
        m.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
        z = Twist()
        for _ in range(12):
            self.pub.publish(z)
            time.sleep(0.03)


def best_shift(a, b):
    best = (0, 1e9)
    for s in range(BINS):
        bb = np.roll(b, s)
        v = ~np.isnan(a) & ~np.isnan(bb)
        if v.sum() < 200:
            continue
        cost = float(np.mean(np.abs(a[v] - bb[v])))
        if cost < best[1]:
            best = (s, cost)
    return best


def main():
    rclpy.init()
    n = Probe()
    n.spin(2.5)
    a = n.grab()
    y0 = n.odom_yaw
    n.drive(1.5, 1.3)     # 约 0.5~0.7 rad
    n.spin(0.8)
    b = n.grab()
    y1 = n.odom_yaw
    if a is None or b is None:
        print("no scan")
        return
    shift, cost = best_shift(a, b)
    deg = shift * 360.0 / BINS
    if deg > 180:
        deg -= 360
    odom_deg = math.degrees(y1 - y0)
    print("odom dyaw = %+.1f deg" % odom_deg)
    print("lidar world-rotation = %+.1f deg (cost %.3f)" % (deg, cost))
    print("=> 车实际转 = %+.1f deg" % (-deg))
    print("=> 比例 = %.2f" % ((-deg) / odom_deg) if abs(odom_deg) > 1 else "=> 比例 n/a")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
