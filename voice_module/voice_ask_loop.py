#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音问答（免唤醒）：循环录音 3.5s -> 109 ASR -> 命中关键词就抓图识别并播报。

    python3 voice_ask_loop.py --rounds 20
"""
import argparse
import json
import subprocess
import sys
import time
import urllib.request

from ask_object import detect, grab_frame, sentence, speak

KEYWORDS = ("识别", "物体", "前方", "看看", "是什么", "前面")


def record_wav(seconds, device, path):
    subprocess.run(
        ["arecord", "-D", device, "-f", "S16_LE", "-r", "16000", "-c", "1",
         "-d", str(int(round(seconds))), path],
        check=True, timeout=seconds + 8,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    return path


def asr_call(path, url):
    with open(path, "rb") as f:
        data = f.read()
    req = urllib.request.Request(url, data=data, method="POST")
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.loads(resp.read().decode("utf-8"))


def describe(path, url):
    with open(path, "rb") as f:
        data = f.read()
    req = urllib.request.Request(url, data=data, method="POST",
                                 headers={"Content-Type": "image/jpeg"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=20)
    ap.add_argument("--record-seconds", type=float, default=5.0)
    ap.add_argument("--asr-url", default="http://192.168.0.109:8091/asr")
    ap.add_argument("--server", default="http://192.168.0.109:8090/detect")
    ap.add_argument("--describe-url", default="http://192.168.0.109:8090/describe")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--device", default="plughw:CARD=Device")
    ap.add_argument("--voice", default="zh-CN-XiaoxiaoNeural")
    ap.add_argument("--no-tts", action="store_true")
    args = ap.parse_args()

    print('循环听写中（%s 轮，每轮 %.1fs）…' % (args.rounds if args.rounds > 0 else '无限', args.record_seconds), flush=True)
    # 预热相机和 VLM，避免第一次识别等模型加载
    try:
        grab_frame(args.topic, '/tmp/warm.jpg')
        describe('/tmp/warm.jpg', args.describe_url)
        print('预热完成', flush=True)
    except Exception as exc:
        print('预热失败: %s' % exc, flush=True)
    hit = False
    prev_text = ''
    i = 0
    while args.rounds <= 0 or i < args.rounds:
        try:
            wav = record_wav(args.record_seconds, args.device, "/tmp/loop_q.wav")
            res = asr_call(wav, args.asr_url)
            text = (res.get("text") or "").strip()
        except Exception as exc:  # noqa: BLE001
            print("[%d] ASR 失败 %s" % (i, exc), flush=True)
            i += 1
            continue
        print('[%d] %s' % (i, text), flush=True)
        joined = (prev_text + text).strip()
        prev_text = text
        if joined and any(k in joined for k in KEYWORDS):
            hit = True
            speak('好，让我看看。', args.device, args.voice, args.no_tts)
            break
        i += 1

    if not hit:
        print("没有听到“识别前方物体”", flush=True)
        return 1

    path = "/tmp/loop_frame.jpg"
    grab_frame(args.topic, path)
    t0 = time.time()
    try:
        res = describe(path, args.describe_url)
        text = (res.get("text") or "").strip()
        print("[VLM] %s (%.2fs)" % (text, time.time() - t0), flush=True)
        if not res.get("ok") or not text:
            raise RuntimeError(res.get("error") or "empty")
        speak("前方是%s。" % text, args.device, args.voice, args.no_tts)
        return 0
    except Exception as exc:  # noqa: BLE001
        print("[VLM] 失败 %s，回退 YOLO" % exc, flush=True)

    result = detect(path, args.server)
    print("[109] %s (%.2fs)" % (json.dumps(result, ensure_ascii=False), time.time() - t0), flush=True)
    speak(sentence(result), args.device, args.voice, args.no_tts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
