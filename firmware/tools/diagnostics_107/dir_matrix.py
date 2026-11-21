#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DirCheck(Node):
    def __init__(self):
        super().__init__("dir_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.pose = None

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.pose = (p.position.x, p.position.y, yaw)

    def spin_for(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, vx, vy, wz, seconds):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(10):
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)


def main():
    rclpy.init()
    n = DirCheck()
    n.spin_for(2.0)  # discovery

    tests = [
        ("forward", 0.40, 0.0, 0.0),
        ("back", -0.40, 0.0, 0.0),
        ("strafe_left", 0.0, 0.40, 0.0),
        ("strafe_right", 0.0, -0.40, 0.0),
        ("turn_left", 0.0, 0.0, 0.6),
        ("turn_right", 0.0, 0.0, -0.6),
    ]
    print("cmd            dx      dy      dyaw")
    for name, vx, vy, wz in tests:
        n.spin_for(0.3)
        before = n.pose
        n.drive(vx, vy, wz, 0.6)
        n.spin_for(0.4)
        after = n.pose
        if before and after:
            dx = after[0] - before[0]
            dy = after[1] - before[1]
            dyaw = math.degrees(after[2] - before[2])
            print("%-13s %+7.3f %+7.3f %+8.1f" % (name, dx, dy, dyaw))
        else:
            print(name, "no odom")
        time.sleep(0.4)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
