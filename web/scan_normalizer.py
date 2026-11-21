#!/usr/bin/env python3
"""Normalize /scan to a FIXED number of range readings.

Why: the LDROBOT STL-27L driver emits a slightly different point count on every
rotation (observed 2156..2172). karto / slam_toolbox sizes its scan buffers from
the first scan and aborts on any other size:

    LaserRangeScan contains 2169 range readings, expected 2168
    Mapper FATAL ERROR - unable to get pointer in probability search!

This node resamples every scan onto a fixed angular grid (nearest source sample,
no holes) and republishes it, so slam_toolbox always sees the same geometry.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TWO_PI = 2.0 * math.pi


class ScanNormalizer(Node):
    def __init__(self):
        super().__init__('scan_normalizer')
        self.declare_parameter('in_topic', '/scan')
        self.declare_parameter('out_topic', '/scan_fixed')
        self.declare_parameter('points', 2100)
        # Must stay in sync with slam_toolbox's max_laser_range. The driver
        # advertises range_max=30, which makes karto raster far-out points and
        # blow up its correlation grid.
        self.declare_parameter('max_range', 12.0)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.n = int(self.get_parameter('points').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.inc = TWO_PI / self.n
        self.out_angles = np.arange(self.n, dtype=np.float64) * self.inc

        self.pub = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, in_topic, self.callback, qos_profile_sensor_data)

        self.count = 0
        self.dropped = 0
        self.sizes = set()
        self.get_logger().info(
            f'normalizing {in_topic} -> {out_topic}: fixed {self.n} points, '
            f'inc={self.inc:.6f} rad')

    def callback(self, msg: LaserScan) -> None:
        n_in = len(msg.ranges)
        if n_in < self.n // 2:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = np.mod(msg.angle_min + np.arange(n_in) * msg.angle_increment, TWO_PI)

        order = np.argsort(angles, kind='stable')
        s_ang = angles[order]
        s_rng = ranges[order]

        # nearest source sample for each fixed output angle -> never leaves a hole
        pos = np.searchsorted(s_ang, self.out_angles)
        pos = np.clip(pos, 1, n_in - 1)
        left = s_ang[pos - 1]
        right = s_ang[pos]
        take_left = (self.out_angles - left) <= (right - self.out_angles)
        idx = np.where(take_left, pos - 1, pos)

        # karto treats NaN as "valid" (all its comparisons are false) and then
        # computes a garbage grid index -> "Mapper FATAL ERROR - unable to get
        # pointer in probability search" + SIGABRT. No-return echoes must be
        # replaced by a finite out-of-range value so karto marks them INVALID.
        no_return = self.max_range
        out_ranges = s_rng[idx]
        bad = ~np.isfinite(out_ranges) | (out_ranges <= 0.0)
        self.dropped += int(bad.sum())
        out_ranges = np.where(bad, no_return, out_ranges)
        out_ranges = np.clip(out_ranges, msg.range_min, no_return)

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.angle_min = 0.0
        out.angle_max = self.inc * (self.n - 1)
        out.angle_increment = self.inc
        out.scan_time = msg.scan_time if msg.scan_time > 0.0 else 0.1
        out.time_increment = out.scan_time / self.n
        out.range_min = msg.range_min
        out.range_max = no_return
        out.ranges = out_ranges.tolist()
        out.intensities = []
        self.pub.publish(out)

        self.count += 1
        self.sizes.add(n_in)
        if self.count % 200 == 0:
            self.get_logger().info(
                f'published {self.count} scans, source sizes {sorted(self.sizes)}, '
                f'nan/zero readings replaced: {self.dropped}')


def main() -> None:
    rclpy.init()
    node = ScanNormalizer()
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
