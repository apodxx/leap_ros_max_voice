#!/usr/bin/env python3
"""地图中继

问题：导航模式下 lite_navigation 只把地图发布一次（TRANSIENT_LOCAL / latched）。
      foxglove_bridge 用 VOLATILE QoS 订阅，启动那一刻没赶上，就永远收不到，
      网页显示"无地图模式"。（已实测：VOLATILE 收到 0 条，TRANSIENT_LOCAL 收到 1 条）

做法：以 TRANSIENT_LOCAL 订阅 /map，收到后在 /map 上以 1 Hz 重播，
      让 VOLATILE 订阅者（foxglove / 网页）随时能拿到。
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


class MapRelay(Node):
    def __init__(self):
        super().__init__("map_relay")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._msg = None
        self._published = 0
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self._pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("map_relay: 以 1 Hz 重播 /map，供 volatile 订阅者使用")

    def _on_map(self, msg):
        if self._msg is None:
            self.get_logger().info(
                f"收到地图 {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m")
        self._msg = msg

    def _tick(self):
        if self._msg is None:
            return
        self._pub.publish(self._msg)
        self._published += 1


def main():
    rclpy.init()
    node = MapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
