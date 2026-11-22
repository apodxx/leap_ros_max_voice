#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音问答（带 ASR 意图判断）：

  “小创小创” 唤醒 -> 录音 3 秒 -> 109 SenseVoice 转写
  -> 命中“识别/物体/前方…” 才抓图 -> 109 YOLO26 识别 -> 播报结果

用法：
  python3 voice_ask.py --wait          # 等唤醒词
  python3 voice_ask.py --once          # 直接抓图识别（调试）
"""
import argparse
import json
import subprocess
import sys
import time
import urllib.request

from ask_object import detect, grab_frame, sentence, speak, wait_wake

KEYWORDS = ("识别", "物体", "前方", "看看", "是什么", "什么", "前面", "瞅", "认一认")


def record_wav(seconds, device, path):
    subprocess.run(
        ["arecord", "-D", device, "-f", "S16_LE", "-r", "16000", "-c", "1",
         "-d", str(seconds), path],
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", action="store_true")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--seconds", type=float, default=90.0, help="等待唤醒词时长")
    ap.add_argument("--record-seconds", type=float, default=3.0)
    ap.add_argument("--asr-url", default="http://192.168.0.109:8091/asr")
    ap.add_argument("--server", default="http://192.168.0.109:8090/detect")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--device", default="plughw:CARD=Device")
    ap.add_argument("--voice", default="zh-CN-XiaoxiaoNeural")
    ap.add_argument("--no-tts", action="store_true")
    args = ap.parse_args()

    if not (args.wait or args.once):
        ap.error("需要 --wait 或 --once")

    if args.wait:
        if not wait_wake(args.port, args.seconds):
            print("超时：没有听到唤醒词", flush=True)
            return 1
        speak("请说。", args.device, args.voice, args.no_tts)
        wav = record_wav(args.record_seconds, args.device, "/tmp/voice_ask_q.wav")
        try:
            result = asr_call(wav, args.asr_url)
            text = (result.get("text") or "").strip()
            print("[ASR] %s (%s)" % (text, result.get("ms")), flush=True)
        except Exception as exc:  # noqa: BLE001
            print("[ASR] 失败: %s" % exc, flush=True)
            text = ""

        if text and not any(k in text for k in KEYWORDS):
            speak("我没听清，可以说：识别前方物体。", args.device, args.voice, args.no_tts)
            return 0

    path = "/tmp/voice_ask_frame.jpg"
    print("[CAM] grabbing %s" % args.topic, flush=True)
    grab_frame(args.topic, path)

    t0 = time.time()
    res = detect(path, args.server)
    print("[109] %s (%.2fs)" % (json.dumps(res, ensure_ascii=False), time.time() - t0), flush=True)
    speak(sentence(res), args.device, args.voice, args.no_tts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
