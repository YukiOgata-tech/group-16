#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Weighted-Center based line follower (fixed version for older gpiozero)
- pwm=True を削除して旧バージョン互換
- 機能はオリジナルと同一
"""

import time
import argparse
from statistics import fmean
from gpiozero import MCP3004, Robot

# ===== 調整パラメータ =====
SENSOR_CHANNELS = [0, 1, 2, 3]
WHITE_IS_HIGH = True
THRESH = 0.55
SMOOTH_N = 4

# 速度係数
BASE_SPEED = 0.45
STEER_GAIN = 0.8
DEADBAND = 0.05
LOSS_COUNT_LIMIT = 6

RAMP_TIME = 0.6  # ソフトスタート時間[s]

# モータピン（IN/INモード）
PIN_AIN1, PIN_AIN2 = 6, 5
PIN_BIN1, PIN_BIN2 = 26, 27
# ===========================


def normalize(raw, white_is_high=True):
    return float(raw) if white_is_high else 1.0 - float(raw)


def moving_average(buf, new_val, n):
    """固定長の移動平均。n<=1なら新値をそのまま返す"""
    if n <= 1:
        return new_val
    buf.append(new_val)
    if len(buf) > n:
        buf.pop(0)
    return fmean(buf)


def compute_center(values):
    """センサ配列から重心位置を求める (-1..1)"""
    n = len(values)
    if n == 1:
        return 0.0
    weights = [(i - (n - 1) / 2) / ((n - 1) / 2) for i in range(n)]
    s = sum(values) + 1e-9
    center = sum(w * v for w, v in zip(weights, values)) / s
    if abs(center) < DEADBAND:
        center = 0.0
    return max(-1.0, min(1.0, center))


def mix_to_motors(base_speed, steer, gain):
    """中央偏差 steer(-1..1) を左右モータ速度に変換"""
    diff = gain * steer
    left = base_speed - diff
    right = base_speed + diff
    return max(-1, min(1, left)), max(-1, min(1, right))


def ramp(robot, target_left, target_right, t_ramp=0.5, steps=12):
    """ソフトスタート"""
    for k in range(1, steps + 1):
        a = k / steps
        robot.value = (target_left * a, target_right * a)
        time.sleep(max(0.0, t_ramp / steps))


def main():
    parser = argparse.ArgumentParser(description="Line follower (fixed)")
    parser.add_argument("--monitor", action="store_true",
                        help="モニタのみ（モータを回さない）")
    parser.add_argument("--speed", type=float, default=BASE_SPEED,
                        help="基準速度 (0..1)")
    parser.add_argument("--gain", type=float, default=STEER_GAIN,
                        help="操舵ゲイン")
    parser.add_argument("--hz", type=float, default=25,
                        help="更新周期[Hz]")
    args = parser.parse_args()

    sensors = [MCP3004(channel=ch) for ch in SENSOR_CHANNELS]

    # pwm=True を削除して旧版対応
    robot = Robot(left=(PIN_AIN1, PIN_AIN2),
                  right=(PIN_BIN1, PIN_BIN2))

    if not args.monitor:
        ramp(robot, 0.0, 0.0, t_ramp=0.2)

    smooth_bufs = [[] for _ in sensors]
    loss_cnt = 0
    period = 1.0 / max(1.0, args.hz)

    print("=== RUNNING ===  Ctrl+C で停止")
    print(f"WHITE_IS_HIGH={WHITE_IS_HIGH}  THRESH={THRESH}  "
          f"SMOOTH_N={SMOOTH_N}  SPEED={args.speed}  GAIN={args.gain}")

    try:
        while True:
            raw_vals = [s.value for s in sensors]
            vals = [normalize(v, WHITE_IS_HIGH) for v in raw_vals]
            vals = [moving_average(smooth_bufs[i], vals[i], SMOOTH_N)
                    for i in range(len(vals))]

            is_dark_like = [(v < THRESH) for v in vals]
            line_present = any(is_dark_like)
            center = compute_center(vals)

            if args.monitor or not line_present:
                left, right = 0.0, 0.0
                loss_cnt += 1
            else:
                loss_cnt = 0
                left, right = mix_to_motors(args.speed, center, args.gain)

            if loss_cnt >= LOSS_COUNT_LIMIT:
                left, right = 0.0, 0.0

            robot.value = (left, right)
            vv = ", ".join(f"{v:0.2f}" for v in vals)
            print(f"vals=[{vv}]  center={center:+0.2f}  out=({left:+0.2f},{right:+0.2f})  "
                  f"{'LOST' if loss_cnt else ''}")
            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("stopped.")


if __name__ == "__main__":
    main()
