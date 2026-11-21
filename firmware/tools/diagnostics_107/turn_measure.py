#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TurnTest(Node):
    def __init__(self):
        super().__init__("turn_measure")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.yaw = None

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def turn(self, wz, seconds):
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
    n = TurnTest()
    n.spin_for(2.0)
    for name, wz in (("left(+1.0)", 1.0), ("right(-1.0)", -1.0)):
        n.spin_for(0.3)
        before = n.yaw
        n.turn(wz, 1.5)
        n.spin_for(0.4)
        after = n.yaw
        if before is not None and after is not None:
            d = math.degrees(after - before)
            while d > 180:
                d -= 360
            while d < -180:
                d += 360
            print("%s dyaw=%+.1f deg" % (name, d))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
