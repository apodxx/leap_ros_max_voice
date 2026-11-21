#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class T(Node):
    def __init__(self):
        super().__init__("straight_drift")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), qos)

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, vx, seconds):
        msg = Twist()
        msg.linear.x = vx
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)


def main():
    rclpy.init()
    n = T()
    n.spin_for(2.0)
    m = n.odom
    p0 = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
    print("start: x=%.3f y=%.3f yaw=%.1f" % p0, flush=True)
    n.drive(0.35, 8.0)
    n.spin_for(0.6)
    m = n.odom
    p1 = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    print("end:   x=%.3f y=%.3f yaw=%.1f" % p1, flush=True)
    print("distance=%.2f m, yaw drift=%+.1f deg, heading=%.1f deg" % (
        d, p1[2] - p0[2], math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
