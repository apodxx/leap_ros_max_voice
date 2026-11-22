#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点：创乐博 AI 语音模块 -> /cmd_vel，并支持“识别前方物体”。

  ros2 run <pkg> voice_cmd_node --ros-args -p port:=/dev/ttyUSB0

参数：
  port            串口，默认自动查找
  linear_speed    前进/后退线速度 m/s，默认 0.15
  angular_speed   转向角速度 rad/s，默认 0.5
  hold            每条指令持续秒数，默认 1.2
  asr_url         109 的 SenseVoice 接口
  detect_url      109 的 YOLO 识别接口
  camera_url      107 主机摄像头快照
话题：
  /cmd_vel        geometry_msgs/Twist
  /voice_command  std_msgs/String（wake / sleep / forward / back / left / right / cmdXX）

唤醒后 3 秒内如果听到“识别前方物体”，会抓一帧主机摄像头 -> 109 YOLO -> 喇叭播报。
"""
import json
import subprocess
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ai_voice_module import VoiceModule

ASK_KEYWORDS = ("识别", "物体", "前方", "看看", "是什么", "前面")


class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__("voice_cmd_node")
        self.declare_parameter("port", "")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("hold", 1.2)
        self.declare_parameter("asr_url", "http://192.168.0.109:8091/asr")
        self.declare_parameter("detect_url", "http://192.168.0.109:8090/detect")
        self.declare_parameter("describe_url", "http://192.168.0.109:8090/describe")
        self.declare_parameter("camera_url", "http://127.0.0.1:8081/snapshot.jpg")
        self.declare_parameter("audio_device", "plughw:CARD=Device")

        port = self.get_parameter("port").value or None
        self.linear = float(self.get_parameter("linear_speed").value)
        self.angular = float(self.get_parameter("angular_speed").value)
        self.hold = float(self.get_parameter("hold").value)
        self.asr_url = self.get_parameter("asr_url").value
        self.detect_url = self.get_parameter("detect_url").value
        self.describe_url = self.get_parameter("describe_url").value
        self.camera_url = self.get_parameter("camera_url").value
        self.audio_device = self.get_parameter("audio_device").value

        self.module = VoiceModule(port)
        self.get_logger().info(f"语音模块已连接：{self.module.port} @115200")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.evt_pub = self.create_publisher(String, "/voice_command", 10)

        self.stop_at = 0.0
        self.ask_thread = None
        self.create_timer(0.02, self.poll_serial)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info("等待唤醒词 / 指令 ……")

    def publish_twist(self, lin: float, ang: float) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    def publish_event(self, name: str) -> None:
        msg = String()
        msg.data = name
        self.evt_pub.publish(msg)

    def handle_event(self, cmd: int, data: int) -> None:
        if cmd == 0x03:
            self.publish_event("wake")
            self.get_logger().info("唤醒")
            self.start_ask()
            return
        if cmd == 0x02:
            self.publish_event("sleep")
            self.get_logger().info("休眠")
            return
        if cmd != 0x00:
            return

        table = {
            0x01: ("forward", self.linear, 0.0),
            0x02: ("back", -self.linear, 0.0),
            0x03: ("left", 0.0, self.angular),
            0x04: ("right", 0.0, -self.angular),
            0x09: ("stop", 0.0, 0.0),
        }
        if data not in table:
            self.publish_event(f"cmd{data:02X}")
            self.get_logger().info(f"收到未收录命令 0x{data:02X}")
            return

        name, lin, ang = table[data]
        self.publish_event(name)
        self.publish_twist(lin, ang)
        self.stop_at = 0.0 if name == "stop" else time.time() + self.hold
        self.get_logger().info(f"{name}: linear={lin:.2f} angular={ang:.2f}")

    # ---- 语音问答 ---------------------------------------------------
    def start_ask(self) -> None:
        if self.ask_thread is not None and self.ask_thread.is_alive():
            return
        self.ask_thread = threading.Thread(target=self.ask_flow, daemon=True)
        self.ask_thread.start()

    def ask_flow(self) -> None:
        time.sleep(0.4)
        wav = "/tmp/ask_cmd.wav"
        try:
            subprocess.run(
                ["arecord", "-D", self.audio_device, "-f", "S16_LE", "-r", "16000",
                 "-c", "1", "-d", "3", wav],
                check=True, timeout=10,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"录音失败: {exc}")
            return

        text = self.call_asr(wav)
        self.get_logger().info(f"唤醒后语音: {text!r}")
        if not text or not any(k in text for k in ASK_KEYWORDS):
            return

        try:
            image = urllib.request.urlopen(self.camera_url, timeout=5).read()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"抓图失败: {exc}")
            return

        # 先用 VLM 描述（比 YOLO 准），失败再回退 YOLO 检测
        text = self.call_describe(image)
        if text:
            self.get_logger().info(f"识别结果: 前方是{text}")
            self.speak("前方是%s。" % text)
            return

        try:
            req = urllib.request.Request(
                self.detect_url, data=image, method="POST",
                headers={"Content-Type": "image/jpeg"},
            )
            with urllib.request.urlopen(req, timeout=15) as resp:
                result = json.loads(resp.read().decode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"识别失败: {exc}")
            return

        sentence = self.make_sentence(result)
        self.get_logger().info(f"识别结果: {sentence}")
        self.speak(sentence)

    def call_describe(self, image: bytes) -> str:
        try:
            req = urllib.request.Request(
                self.describe_url, data=image, method="POST",
                headers={"Content-Type": "image/jpeg"},
            )
            with urllib.request.urlopen(req, timeout=20) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            if data.get("ok"):
                return (data.get("text") or "").strip()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VLM 描述失败: {exc}")
        return ""

    def call_asr(self, wav: str) -> str:
        try:
            with open(wav, "rb") as f:
                data = f.read()
            req = urllib.request.Request(self.asr_url, data=data, method="POST")
            with urllib.request.urlopen(req, timeout=20) as resp:
                return (json.loads(resp.read().decode("utf-8")).get("text") or "").strip()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"ASR 失败: {exc}")
            return ""

    @staticmethod
    def make_sentence(result: dict) -> str:
        if not result.get("ok"):
            return "识别服务出错了。"
        objects = result.get("objects") or []
        if not objects:
            return "我没有看清楚，画面里没有识别到物体。"
        names = []
        for obj in sorted(objects, key=lambda o: -o.get("conf", 0))[:3]:
            name = obj.get("zh") or obj.get("en")
            pos = obj.get("pos") or ""
            item = f"{pos}有{name}" if pos else name
            if item not in names:
                names.append(item)
        return "画面里" + "，".join(names) + "。"

    def speak(self, text: str) -> None:
        self.get_logger().info(f"播报: {text}")
        mp3 = "/tmp/ask_reply.mp3"
        for _ in range(2):
            try:
                subprocess.run(["edge-tts", "--voice", "zh-CN-XiaoxiaoNeural",
                                "--text", text, "--write-media", mp3],
                               check=True, timeout=30,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.run(["mpg123", "-q", "-a", self.audio_device, mp3],
                               timeout=40,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
        try:
            subprocess.run(["espeak-ng", "-v", "cmn", "-s", "170", text], timeout=30,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TTS 失败: {exc}")

    # ---- 串口 -------------------------------------------------------
    def poll_serial(self) -> None:
        for _ in range(8):
            ev = self.module.read_event()
            if ev is None:
                break
            self.handle_event(*ev)

    def watchdog(self) -> None:
        if self.stop_at and time.time() >= self.stop_at:
            self.publish_twist(0.0, 0.0)
            self.stop_at = 0.0

    def destroy_node(self):
        self.publish_twist(0.0, 0.0)
        self.module.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
