#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Check(Node):
    def __init__(self):
        super().__init__("fused_yaw_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_yaw = None
        self.filt_yaw = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom_yaw", yaw_from_quat(m.pose.pose.orientation)), qos)
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: setattr(self, "filt_yaw", yaw_from_quat(m.pose.pose.orientation)), qos)

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, wz, seconds):
        msg = Twist()
        msg.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
        zero = Twist()
        for _ in range(15):
            self.pub.publish(zero)
            time.sleep(0.03)


def main():
    rclpy.init()
    n = Check()
    n.spin_for(2.5)
    for wz, dur in ((1.5, 2.0), (-1.5, 2.0)):
        n.spin_for(0.3)
        o0, f0 = n.odom_yaw, n.filt_yaw
        n.drive(wz, dur)
        n.spin_for(0.4)
        o1, f1 = n.odom_yaw, n.filt_yaw
        print("cmd %+.1f x %.0fs: odom dyaw=%+.1f deg, filtered dyaw=%+.1f deg" % (
            wz, dur,
            math.degrees(norm(o1 - o0)) if None not in (o0, o1) else float("nan"),
            math.degrees(norm(f1 - f0)) if None not in (f0, f1) else float("nan")))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
