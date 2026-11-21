#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 107 上 Astra 相机的 ROS 图像发布成 MJPEG，供网页中控台直接显示。

    GET /stream         multipart/x-mixed-replace（浏览器 <img src> 直接可用）
    GET /snapshot.jpg   单帧
    GET /health

启动（需要先 source ROS 环境）：
    python3 camera_host_stream.py --port 8081
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

LATEST = {"jpg": None, "stamp": 0.0}
LOCK = threading.Lock()


class Grabber(Node):
    def __init__(self, topic):
        super().__init__("camera_host_stream")
        self.bridge = CvBridge()
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if ok:
                with LOCK:
                    LATEST["jpg"] = buf.tobytes()
                    LATEST["stamp"] = time.time()
        except Exception:
            pass


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path.startswith("/health"):
            with LOCK:
                age = time.time() - LATEST["stamp"] if LATEST["stamp"] else -1
            body = ('{"ok": %s, "age": %.1f}' % ("true" if LATEST["jpg"] else "false", age)).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/snapshot"):
            with LOCK:
                jpg = LATEST["jpg"]
            if not jpg:
                self.send_error(503, "no frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg)
            return

        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    with LOCK:
                        jpg = LATEST["jpg"]
                    if jpg:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.066)
            except (BrokenPipeError, ConnectionResetError):
                return
            return

        self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--topic", default="/camera/color/image_raw")
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args.topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("camera host stream on 0.0.0.0:%d (topic %s)" % (args.port, args.topic), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
