#!/usr/bin/env python3
"""中文语音播报：edge-tts（在线，自然音）优先，失败回退 espeak-ng（离线）。

用法：python3 speak.py "到达1号点"
      python3 speak.py --voice zh-CN-YunxiNeural "到达1号点"
"""
import asyncio
import hashlib
import os
import subprocess
import sys

CACHE_DIR = '/root/.cache/tts'
DEFAULT_VOICE = 'zh-CN-XiaoxiaoNeural'
MP3_PLAYER = 'mpg123'


def cache_path(text, voice):
    key = hashlib.md5(f'{voice}|{text}'.encode('utf-8')).hexdigest()[:16]
    os.makedirs(CACHE_DIR, exist_ok=True)
    return os.path.join(CACHE_DIR, f'{key}.mp3')


def synth_edge(text, voice, path):
    import edge_tts

    async def run():
        comm = edge_tts.Communicate(text, voice)
        await asyncio.wait_for(comm.save(path), timeout=12)

    asyncio.run(run())


def play_mp3(path):
    subprocess.run([MP3_PLAYER, '-q', path], check=False, timeout=30,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def play_espeak(text):
    wav = '/tmp/tts_espeak.wav'
    subprocess.run(['espeak-ng', '-v', 'cmn', '-s', '150', '-w', wav, text],
                   check=False, timeout=20,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if os.path.exists(wav):
        subprocess.run(['aplay', '-q', wav], check=False, timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    args = [a for a in sys.argv[1:]]
    voice = DEFAULT_VOICE
    if args and args[0] == '--voice' and len(args) >= 3:
        voice = args[1]
        args = args[2:]
    text = ' '.join(args).strip()
    if not text:
        print('usage: speak.py [--voice NAME] <text>', file=sys.stderr)
        return 2

    path = cache_path(text, voice)
    if not os.path.exists(path):
        try:
            synth_edge(text, voice, path)
        except Exception as exc:  # 断网 / 服务不可用时走离线兜底
            print(f'edge-tts 失败（{type(exc).__name__}），回退 espeak-ng', file=sys.stderr)
            play_espeak(text)
            return 0

    play_mp3(path)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'speak 异常: {exc}', file=sys.stderr)
        sys.exit(1)
