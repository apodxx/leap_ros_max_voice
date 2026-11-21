#!/usr/bin/env python3
import math

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class Check(Node):
    def __init__(self):
        super().__init__("map_robot_check")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_map(self, msg):
        self.map = msg

    def run(self):
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.25)
            if self.map is not None:
                break
        if self.map is None:
            print("no map")
            return
        pose = None
        last = None
        for _ in range(40):
            for frame in ("base_footprint", "base_link"):
                try:
                    tf = self.buf.lookup_transform("map", frame, rclpy.time.Time())
                    pose = (tf.transform.translation.x, tf.transform.translation.y, frame)
                    break
                except Exception as e:
                    last = e
            if pose is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.3)
        if pose is None:
            print("no map->base transform:", last)
            return
        x, y, frame = pose
        info = self.map.info
        ox, oy = info.origin.position.x, info.origin.position.y
        res = info.resolution
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        print("robot in %s frame: %.3f, %.3f" % (frame, x, y))
        print("map %dx%d res %.3f origin (%.2f, %.2f)" % (info.width, info.height, res, ox, oy))
        print("cell (%d, %d)" % (cx, cy))
        if 0 <= cx < info.width and 0 <= cy < info.height:
            val = self.map.data[cy * info.width + cx]
            print("occupancy at robot cell:", val)
            # 3x3 neighbourhood
            vals = []
            for dy in (-1, 0, 1):
                row = []
                for dx in (-1, 0, 1):
                    yy, xx = cy + dy, cx + dx
                    if 0 <= xx < info.width and 0 <= yy < info.height:
                        row.append(self.map.data[yy * info.width + xx])
                    else:
                        row.append(None)
                vals.append(row)
            for r in vals:
                print(r)
        else:
            print("robot outside map bounds")


def main():
    rclpy.init()
    n = Check()
    n.run()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
