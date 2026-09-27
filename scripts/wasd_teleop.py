#!/usr/bin/env python3
"""wasd_teleop.py — 终端 WASD 键盘控制底盘（纯标准库，无需 pip 装包）

用法：
    python3 scripts/wasd_teleop.py                     # 默认 0.2 m/s / 0.8 rad/s
    python3 scripts/wasd_teleop.py --speed 0.3 --turn 1.0
    python3 scripts/wasd_teleop.py --url http://192.168.8.x:18765

按键（按住即动，松开约 0.5s 后自停；空格立即急停）：
    W 前进   S 后退   A 左转   D 右转   空格 急停   Ctrl+C / q 退出

原理：终端 cbreak 模式读按键；按住时终端自动重复按键维持速度指令，
以 20Hz 持续 POST /execute set_base_velocity，松开后停止发送，
服务端超过 timeout（默认 0.5s）收不到更新自动刹停。
"""
import argparse
import json
import select
import sys
import termios
import time
import tty
import urllib.request


def send(url, vx, wz, timeout=1.0):
    payload = json.dumps({"action": "set_base_velocity",
                          "parameters": {"vx": round(vx, 3),
                                         "wz": round(wz, 3)}}).encode()
    req = urllib.request.Request(
        url + "/execute", data=payload,
        headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=timeout).read()


def main():
    ap = argparse.ArgumentParser(description="WASD 键盘控制 RouteManager 底盘")
    ap.add_argument("--url", default="http://127.0.0.1:18765")
    ap.add_argument("--speed", type=float, default=0.2, help="线速度 m/s")
    ap.add_argument("--turn", type=float, default=0.8, help="角速度 rad/s")
    ap.add_argument("--hz", type=float, default=20.0, help="发送频率")
    args = ap.parse_args()

    print(f"底盘遥控 → {args.url}   线速度 {args.speed} m/s   角速度 {args.turn} rad/s")
    print("W 前进  S 后退  A 左转  D 右转  空格 急停  Ctrl+C 退出")
    print("（按住键移动，松开自动停；若方向反了改 config.json base_teleop 符号）")

    vel = {"w": (args.speed, 0.0), "s": (-args.speed, 0.0),
           "a": (0.0, args.turn), "d": (0.0, -args.turn)}
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    last_key, last_time = None, 0.0
    hold = 0.5          # 该时间内没有新按键（含自动重复）视为松开
    period = 1.0 / args.hz
    next_t = time.time()
    try:
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k == " ":                       # 急停
                    last_key, last_time = None, 0.0
                    send(args.url, 0.0, 0.0)
                elif k in vel:
                    last_key, last_time = k, time.time()
                elif k in ("q", "\x1b"):           # q / ESC 退出
                    break
            moving = last_key is not None and (time.time() - last_time) < hold
            if moving:
                vx, wz = vel[last_key]
            else:
                vx, wz = 0.0, 0.0
                if last_key is not None and not moving:
                    last_key = None                # 已刹停，清除按键
            try:
                send(args.url, vx, wz)
                state = f"{vx:+.2f} m/s  {wz:+.2f} rad/s"
            except Exception as e:
                state = f"发送失败: {e}"
            print(f"\r{state:<36}", end="", flush=True)
            next_t += period
            nap = next_t - time.time()
            time.sleep(nap if nap > 0 else 0)
            if nap <= 0:
                next_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            send(args.url, 0.0, 0.0)
        except Exception:
            pass
        print("\n已退出，底盘已刹停")


if __name__ == "__main__":
    main()
