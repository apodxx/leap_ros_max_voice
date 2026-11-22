#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""107 端：语音问「当前物体是什么」→ 抓相机画面 → 109 YOLO26 识别 → 喇叭播报。

用法：
  # 等唤醒词（说“小创小创”）
  python3 ask_object.py --wait
  # 立刻拍一张识别（调试）
  python3 ask_object.py --once
  # 用现成图片调试（不抓相机）
  python3 ask_object.py --image /tmp/test.jpg
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

WAKE = b"\xaa\x55\x03\x00\xfb"
SAY = b"\xaa\x55\xff\x01\xfb"
COCO_ZH = None  # 由服务端返回中文名


def speak(text, device, voice, no_tts=False):
    print("[TTS] %s" % text, flush=True)
    if no_tts:
        return
    mp3 = "/tmp/ask_object_tts.mp3"
    last = None
    for _ in range(3):
        try:
            subprocess.run(["edge-tts", "--voice", voice, "--text", text,
                            "--write-media", mp3], check=True, timeout=30,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["mpg123", "-q", "-a", device, mp3], timeout=40,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
        except Exception as exc:  # noqa: BLE001
            last = exc
            time.sleep(0.6)
    print("[TTS] edge-tts failed (%s), fallback to espeak-ng" % last, flush=True)
    try:
        subprocess.run(["espeak-ng", "-v", "cmn", "-s", "170", text], timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:  # noqa: BLE001
        print("[TTS] fallback failed: %s" % exc, flush=True)


def grab_frame(topic, out_path, timeout=8.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    rclpy.init()
    node = Node("ask_object_grab")
    bridge = CvBridge()
    got = {"img": None}

    def cb(msg):
        if got["img"] is None:
            got["img"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    end = time.time() + timeout
    while time.time() < end and got["img"] is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if got["img"] is None:
        raise RuntimeError("没有收到相机画面（%s），确认相机节点在跑" % topic)
    cv2.imwrite(out_path, got["img"], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return out_path


def detect(image_path, server):
    with open(image_path, "rb") as f:
        data = f.read()
    req = urllib.request.Request(server, data=data, method="POST",
                                 headers={"Content-Type": "application/octet-stream"})
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode("utf-8"))


def sentence(res):
    if not res.get("ok"):
        return "识别服务出错：%s" % res.get("error", "未知错误")
    objs = res.get("objects", [])
    if not objs:
        return "我没有看清楚。把物体放到画面中间，离半米到两米，再试一次。"
    objs = sorted(objs, key=lambda o: -o["conf"])[:3]
    parts = []
    for o in objs:
        name = o.get("zh") or o.get("en")
        pos = o.get("pos", "")
        parts.append("%s有%s" % (pos, name) if pos else name)
    return "画面里%s。" % "，".join(parts)


def wait_wake(port, seconds):
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.rts = False
    ser.dtr = False
    ser.open()
    ser.reset_input_buffer()
    print("[SERIAL] 等待唤醒词（小创小创），%d 秒内有效" % seconds, flush=True)
    buf = bytearray()
    end = time.time() + seconds
    try:
        while time.time() < end:
            data = ser.read(64)
            if not data:
                continue
            buf.extend(data)
            while True:
                idx = buf.find(b"\xaa\x55")
                if idx < 0:
                    del buf[:-1]
                    break
                if idx:
                    del buf[:idx]
                if len(buf) < 5:
                    break
                frame = bytes(buf[:5])
                if frame[4] != 0xFB:
                    del buf[:1]
                    continue
                del buf[:5]
                print("[SERIAL] %s" % frame.hex(" "), flush=True)
                if frame == WAKE:
                    return True
        return False
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", action="store_true", help="等唤醒词后触发")
    ap.add_argument("--once", action="store_true", help="立刻触发一次")
    ap.add_argument("--image", help="直接用图片，不抓相机")
    ap.add_argument("--seconds", type=float, default=60.0, help="等待唤醒词的时长")
    ap.add_argument("--server", default="http://192.168.0.109:8090/detect")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--device", default="plughw:CARD=Device")
    ap.add_argument("--voice", default="zh-CN-XiaoxiaoNeural")
    ap.add_argument("--no-tts", action="store_true")
    ap.add_argument("--keep", action="store_true", help="保留抓到的图片")
    args = ap.parse_args()

    if not (args.wait or args.once or args.image):
        ap.error("需要 --wait / --once / --image 之一")

    if args.wait:
        if not wait_wake(args.port, args.seconds):
            print("超时，没有听到唤醒词", flush=True)
            return 1
        speak("好的，让我看看。", args.device, args.voice, args.no_tts)

    if args.image:
        path = args.image
    else:
        path = "/tmp/ask_object_frame.jpg"
        print("[CAM] grabbing %s" % args.topic, flush=True)
        grab_frame(args.topic, path)

    t0 = time.time()
    res = detect(path, args.server)
    print("[109] %s (%.2fs)" % (json.dumps(res, ensure_ascii=False), time.time() - t0), flush=True)
    text = sentence(res)
    speak(text, args.device, args.voice, args.no_tts)
    if not args.keep and not args.image:
        try:
            os.remove(path)
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
