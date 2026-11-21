#!/usr/bin/env python3
"""Remove the IMU's zero-rate bias so the gyro can drive EKF yaw rate.

Why: the firmware calibrates the gyro bias once at boot. If the chassis is
disturbed during those ~3 s, a large offset is baked in for the whole run
(measured: 16 deg/s after a dirty boot vs 0.78 deg/s after a clean one).
Even 0.78 deg/s integrates to ~47 deg/min, which ruins mapping.

This node re-estimates the bias whenever the wheels report standstill, so the
gyro's yaw rate is trustworthy for robot_localization.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuBias(Node):
    def __init__(self):
        super().__init__('imu_bias')
        self.declare_parameter('in_topic', '/imu')
        self.declare_parameter('out_topic', '/imu_biasfree')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('standstill_sec', 2.0)
        self.declare_parameter('alpha', 0.05)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.standstill_sec = float(self.get_parameter('standstill_sec').value)
        self.alpha = float(self.get_parameter('alpha').value)

        # bias in rad/s
        self.bias = [0.0, 0.0, 0.0]
        self.have_bias = False
        self.last_motion = self.get_clock().now()
        self.last_odom = None
        self.n_updates = 0

        out_qos = QoSProfile(depth=20)
        out_qos.reliability = ReliabilityPolicy.RELIABLE
        out_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(Imu, self.out_topic, out_qos)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, self.in_topic, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f'{self.in_topic} -> {self.out_topic}, standstill={self.standstill_sec}s '
            f'from {self.odom_topic}')

    def on_odom(self, msg):
        self.last_odom = self.get_clock().now()
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        w = abs(msg.twist.twist.angular.z)
        if v > 0.01 or w > 0.02:
            self.last_motion = self.get_clock().now()

    def on_imu(self, msg):
        now = self.get_clock().now()
        still = (now - self.last_motion).nanoseconds * 1e-9 > self.standstill_sec

        # Fail safe: without fresh wheel odometry we cannot know whether the
        # robot is moving, so never touch the bias estimate.
        odom_fresh = (self.last_odom is not None and
                      (now - self.last_odom).nanoseconds * 1e-9 < 1.0)

        a = msg.angular_velocity
        raw = (a.x, a.y, a.z)

        if still and odom_fresh:
            if not self.have_bias:
                self.bias = list(raw)
                self.have_bias = True
            else:
                for i in range(3):
                    self.bias[i] += self.alpha * (raw[i] - self.bias[i])
            self.n_updates += 1
            if self.n_updates % 100 == 0:
                self.get_logger().info(
                    'bias(deg/s) = %+0.3f %+0.3f %+0.3f' %
                    tuple(math.degrees(b) for b in self.bias))

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity.x = a.x - self.bias[0]
        out.angular_velocity.y = a.y - self.bias[1]
        out.angular_velocity.z = a.z - self.bias[2]
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuBias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
