#!/usr/bin/env python3
"""在已保存的地图上做全局扫描匹配，求出机器人位姿并发布 /initialpose。

用法：python3 auto_localize.py [地图名，默认 home]
输出：一行 JSON  {"ok":true,"score":..,"x":..,"y":..,"yaw_deg":..}
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MAPS_DIR = '/root/chuyun_ws/src/chuyun_navigation2/maps'
SCAN_TOPIC = '/scan_fixed'


def read_pgm(path):
    data = open(path, 'rb').read()
    idx, fields = 0, []
    while len(fields) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        fields.append(data[start:idx])
    w, h = int(fields[1]), int(fields[2])
    idx += 1
    if fields[0] == b'P5':
        buf = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)
    else:
        buf = np.array(data[idx:].split(), dtype=np.int32)[:w * h].reshape(h, w).astype(np.uint8)
    return buf


def read_yaml(path):
    out = {}
    for line in open(path, encoding='utf-8'):
        line = line.split('#')[0].strip()
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        out[k.strip()] = v.strip()
    return out


class Locator(Node):
    def __init__(self):
        super().__init__('auto_localize')
        self.scan = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=False)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def _cb(self, msg):
        self.scan = msg


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'home'
    t_start = time.time()
    try:
        meta = read_yaml(f'{MAPS_DIR}/{name}.yaml')
        img = read_pgm(f'{MAPS_DIR}/{meta["image"]}')
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': f'地图读取失败: {exc}'}))
        return

    img = np.flipud(img)
    height, width = img.shape
    res = float(meta.get('resolution', 0.05))
    ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    occ = img < 100
    if occ.sum() < 50:
        print(json.dumps({'ok': False, 'error': '地图里没有占用栅格'}))
        return

    # 膨胀 2 格（10 cm）作为匹配容差
    score_map = occ.copy()
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            if dx * dx + dy * dy <= 4:
                score_map |= np.roll(np.roll(occ, dy, 0), dx, 1)

    rclpy.init()
    node = Locator()
    ex = MultiThreadedExecutor(4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    deadline = time.time() + 20
    while time.time() < deadline and node.scan is None:
        time.sleep(0.2)
    if node.scan is None:
        print(json.dumps({'ok': False, 'error': f'{SCAN_TOPIC} 无数据'}))
        return

    scan = node.scan
    tf_msg, deadline = None, time.time() + 15
    while time.time() < deadline:
        try:
            tf_msg = node.buf.lookup_transform('base_footprint', scan.header.frame_id, rclpy.time.Time())
            break
        except Exception:
            time.sleep(0.4)
    if tf_msg is None:
        print(json.dumps({'ok': False, 'error': 'TF base_footprint->激光 不可用'}))
        return

    rq = tf_msg.transform.rotation
    yaw_tf = math.atan2(2 * (rq.w * rq.z + rq.x * rq.y), 1 - 2 * (rq.y * rq.y + rq.z * rq.z))
    rr = np.asarray(scan.ranges, dtype=np.float64)
    ang = scan.angle_min + np.arange(rr.size) * scan.angle_increment
    good = np.isfinite(rr) & (rr > 0.15) & (rr < 8.0)
    rr, ang = rr[good], ang[good]
    if rr.size < 60:
        print(json.dumps({'ok': False, 'error': '有效激光点太少'}))
        return
    px = tf_msg.transform.translation.x + rr * np.cos(ang + yaw_tf)
    py = tf_msg.transform.translation.y + rr * np.sin(ang + yaw_tf)

    # 降采样到 60 点（精度够，速度快 2.5 倍）
    sel = np.linspace(0, px.size - 1, 60).astype(int)
    qx, qy = px[sel], py[sel]

    def score(x, y, th):
        c, s = math.cos(th), math.sin(th)
        wx = x + qx * c - qy * s
        wy = y + qx * s + qy * c
        cx = np.round((wx - ox) / res).astype(int)
        cy = np.round((wy - oy) / res).astype(int)
        ok = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
        if ok.sum() < len(cx) * 0.5:
            return -1.0
        return score_map[cy[ok], cx[ok]].mean()

    # ---- 粗搜：0.25 m 网格 + 7.5 度步长，按朝向向量化 ----
    step = 0.25
    gx = np.arange(0.0, width * res - step / 2, step)
    gy = np.arange(0.0, height * res - step / 2, step)
    GX, GY = np.meshgrid(ox + gx, oy + gy)
    GXf, GYf = GX.ravel(), GY.ravel()
    best = (-1.0, 0.0, 0.0, 0.0)
    for k in range(48):
        th = k * math.pi / 24
        c, s = math.cos(th), math.sin(th)
        wx = GXf[:, None] + qx[None, :] * c - qy[None, :] * s
        wy = GYf[:, None] + qx[None, :] * s + qy[None, :] * c
        cx = np.round((wx - ox) / res).astype(np.int32)
        cy = np.round((wy - oy) / res).astype(np.int32)
        ok = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
        valid = ok.sum(axis=1)
        vals = np.where(ok, score_map[np.clip(cy, 0, height - 1), np.clip(cx, 0, width - 1)], 0)
        hits = vals.sum(axis=1)
        ratio = np.where(valid > qx.size * 0.5, hits / np.maximum(valid, 1), -1.0)
        i = int(np.argmax(ratio))
        if ratio[i] > best[0]:
            best = (float(ratio[i]), float(GXf[i]), float(GYf[i]), th)

    bs, bx, by, bt = float(best[0]), float(best[1]), float(best[2]), float(best[3])
    # ---- 细化 ----
    for sp, span in ((0.05, 0.30), (0.02, 0.06)):
        for dx in np.arange(-span, span + 1e-9, sp):
            for dy in np.arange(-span, span + 1e-9, sp):
                for dt in np.arange(-0.13, 0.131, 0.0325):
                    v = score(bx + dx, by + dy, bt + dt)
                    if v > bs:
                        bs, bx, by, bt = float(v), float(bx + dx), float(by + dy), float(bt + dt)

    result = {'ok': bool(float(bs) >= 0.55), 'score': round(float(bs), 4), 'x': round(float(bx), 3),
              'y': round(float(by), 3), 'yaw_deg': round(float(math.degrees(bt)), 1),
              'elapsed_s': round(time.time() - t_start, 1)}
    if not result['ok']:
        result['error'] = '匹配得分偏低，请人工设置初始位姿'
        print(json.dumps(result))
        return

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = bx
    msg.pose.pose.position.y = by
    msg.pose.pose.orientation.z = math.sin(bt / 2)
    msg.pose.pose.orientation.w = math.cos(bt / 2)
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.07
    for _ in range(5):
        msg.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(msg)
        time.sleep(0.15)
    print(json.dumps(result))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # 保证永远输出合法 JSON
        print(json.dumps({'ok': False, 'error': str(exc)}))
