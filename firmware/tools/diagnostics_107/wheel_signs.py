#!/usr/bin/env python3
import json
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BOARD = "http://192.168.0.100/api/status"


def poll(stop, out):
    while not stop.is_set():
        try:
            d = json.loads(urllib.request.urlopen(BOARD, timeout=1.5).read().decode())
            m = d["motion"]
            out.append((m["target_vx"], m["vel_fl"], m["vel_fr"], m["vel_bl"], m["vel_br"]))
        except Exception:
            pass
        time.sleep(0.2)


def main():
    rclpy.init()
    node = Node("wheel_signs")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    rows = []
    stop = threading.Event()
    t = threading.Thread(target=poll, args=(stop, rows), daemon=True)
    t.start()
    time.sleep(2.0)

    msg = Twist()
    msg.linear.x = 0.40
    t0 = time.time()
    while time.time() - t0 < 1.8:
        pub.publish(msg)
        time.sleep(0.05)
    zero = Twist()
    for _ in range(20):
        pub.publish(zero)
        time.sleep(0.05)
    stop.set()
    t.join(timeout=2)

    active = [r for r in rows if r[0] != 0]
    print("samples:", len(active))
    if active:
        for i, name in enumerate(("FL", "FR", "BL", "BR"), start=1):
            vals = [r[i] for r in active]
            vals.sort()
            print("%s: median %.1f  min %.1f  max %.1f" % (name, vals[len(vals)//2], vals[0], vals[-1]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
