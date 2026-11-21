#!/usr/bin/env python3
import json
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BOARD = "http://192.168.0.102/api/status"


def poll(stop, out):
    while not stop.is_set():
        try:
            d = json.loads(urllib.request.urlopen(BOARD, timeout=1.5).read().decode())
            m = d["motion"]
            out.append((m["target_vx"], m["vx"], m["vel_fl"], m["vel_fr"], m["vel_bl"], m["vel_br"]))
        except Exception as e:
            out.append(("ERR",))
        time.sleep(0.2)


def main():
    rclpy.init()
    node = Node("probe_move")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    rows = []
    stop = threading.Event()
    t = threading.Thread(target=poll, args=(stop, rows), daemon=True)
    t.start()
    time.sleep(2.0)
    msg = Twist()
    msg.linear.x = 0.4
    t0 = time.time()
    while time.time() - t0 < 1.5:
        pub.publish(msg)
        time.sleep(0.05)
    zero = Twist()
    for _ in range(15):
        pub.publish(zero)
        time.sleep(0.03)
    stop.set()
    t.join(timeout=2)
    for r in rows[-12:]:
        print(r)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
