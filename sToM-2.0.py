"""
Weighted-Center based line follower (MCP3004 + DRV8835 + gpiozero).
- 4chのフォトリフレクタを読みつつ、ライン位置の“重心”を求めて左右モータを連続制御
- センサ値を常時コンソール出力
- ソフトスタート、喪失時ストップ、デッドバンド付き
"""

import time
import argparse
from statistics import fmean
from gpiozero import MCP3004, Robot

# 調整パラメータ 
SENSOR_CHANNELS = [0, 1, 2, 3]
WHITE_IS_HIGH = True
THRESH = 0.55

SMOOTH_N = 4

# 速度係数
BASE_SPEED = 0.45        # 直進時の基準速度 0..1
STEER_GAIN = 0.8         
DEADBAND = 0.05          

LOSS_COUNT_LIMIT = 6     # 連続フレーム数

# スタート遅延用
RAMP_TIME = 0.6          # 立ち上げ時間[s]

# モータピン（IN/INモード）
PIN_AIN1 = 6
PIN_AIN2 = 5
PIN_BIN1 = 26
PIN_BIN2 = 27


def normalize(raw, white_is_high=True):
    if white_is_high:
        return float(raw)
    return 1.0 - float(raw)


def moving_average(buf, new_val, n):
    """固定長の移動平均。n<=1なら新値をそのまま返す"""
    if n <= 1:
        return new_val
    buf.append(new_val)
    if len(buf) > n:
        buf.pop(0)
    return fmean(buf)


def compute_center(values):
    """
    0..1の正規化済みセンサ値配列(左→右)を受け取り、
    重心位置を -1..+1 にマップして返す。
    -1: 左側にライン、+1: 右側にライン、0: 中央
    """
    n = len(values)
    # 位置重み（-1 .. +1 の等間隔）
    if n == 1:
        return 0.0
    weights = [ (i - (n-1)/2) / ((n-1)/2) for i in range(n) ]
    s = sum(values) + 1e-9
    center = sum(w*v for w, v in zip(weights, values)) / s
    # デッドバンド適用
    if abs(center) < DEADBAND:
        center = 0.0
    return max(-1.0, min(1.0, center))


def mix_to_motors(base_speed, steer, gain):
    """
    中央偏差 steer(-1..1) を左右モータ速度(-1..1)に変換。
    base_speed: 前進の基準、gain: 差動の強さ
    """
    diff = gain * steer
    left = base_speed - diff
    right = base_speed + diff
    # クリップ
    left = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))
    return left, right


def ramp(robot, target_left, target_right, t_ramp=0.5, steps=12):
    """現在速度からtargetまで滑らかに遷移"""
    for k in range(1, steps+1):
        a = k/steps
        robot.value = (target_left*a, target_right*a)
        time.sleep(max(0.0, t_ramp/steps))


def main():
    parser = argparse.ArgumentParser(
        description="Line follower (weighted center method)")
    parser.add_argument("--monitor", action="store_true",
                        help="モニタのみ（モータを回さない）")
    parser.add_argument("--speed", type=float, default=BASE_SPEED,
                        help=f"基準速度 0..1 (default {BASE_SPEED})")
    parser.add_argument("--gain", type=float, default=STEER_GAIN,
                        help=f"操舵ゲイン (default {STEER_GAIN})")
    parser.add_argument("--hz", type=float, default=25,
                        help="更新周期[Hz]")
    args = parser.parse_args()

    # デバイス初期化
    sensors = [MCP3004(channel=ch) for ch in SENSOR_CHANNELS]
    robot = Robot(left=(PIN_AIN1, PIN_AIN2),
                  right=(PIN_BIN1, PIN_BIN2),
                  pwm=True)

    # ソフトスタート：安全のためいきなり全開にしない
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
            # 1) センサ読み取り & 正規化
            raw_vals = [s.value for s in sensors]
            vals = [normalize(v, WHITE_IS_HIGH) for v in raw_vals]
            # 平滑化
            vals = [moving_average(smooth_bufs[i], vals[i], SMOOTH_N)
                    for i in range(len(vals))]

            # 2) ラインの“存在”判定（ここでは「低いほどライン」と仮定して反転済み）
            #    値が大きいほど「白（反射高）」に近い前提でTHRESHを超えたら“白側”
            #    → すべて白側に偏っていればライン喪失とみなす
            is_dark_like = [ (v < THRESH) for v in vals ]
            line_present = any(is_dark_like)

            # 3) 重心を計算（-1..1）
            center = compute_center(vals)  # 0に近いほど中央

            # 4) 速度合成
            if args.monitor or not line_present:
                left, right = 0.0, 0.0
                loss_cnt += 1
            else:
                loss_cnt = 0
                left, right = mix_to_motors(args.speed, center, args.gain)

            # 5) 出力
            #    喪失が続いたら完全停止（安全）
            if loss_cnt >= LOSS_COUNT_LIMIT:
                left, right = 0.0, 0.0

            robot.value = (left, right)

            # 6) コンソール表示（見やすいように丸め）
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
