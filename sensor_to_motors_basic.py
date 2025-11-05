#!/usr/bin/env python3
# sensor_to_motors_basic.py
# フォトリフレクタ(4ch, MCP3004)のON/OFFで左右モータを単純制御するだけの最小コード
# - 左グループ(CH0,CH1) が閾値を超えたら左モータON
# - 右グループ(CH2,CH3) が閾値を超えたら右モータON
# - 方向は前進固定、速度は一定（SPEED）。停止は0。
#
# 配線（回路図に合わせて）:
#  MCP3004: CH0..CH3 ← 各フォトリフレクタ
#  SPI0: CE0(BCM8), MOSI(BCM10), MISO(BCM9), SCLK(BCM11)
#  DRV8835: 左=(BCM6,BCM5), 右=(BCM26,BCM27)
# 依存: sudo apt install python3-gpiozero python3-numpy
# 実行: python3 sensor_to_motors_basic.py   （Ctrl+Cで停止）

from gpiozero import MCP3004, Robot
import numpy as np
import time

# ===== 設定 =====
ADC_CHANNELS = [0, 1, 2, 3]       # 左→右の物理並びに合わせる
LEFT_GROUP   = [0, 1]
RIGHT_GROUP  = [2, 3]

LEFT_PINS    = (6, 5)             # 左モータ AIN1, AIN2
RIGHT_PINS   = (26, 27)           # 右モータ BIN1, BIN2

SPEED        = 0.40               # モータOの速度(0～1)
CLAMP        = 0.85              
WHITE_IS_HIGH = True              # 「白で値が大、黒で小」なら True
THRESH       = 0.60               # 閾値（WHITE_IS_HIGH=True前提の例）

PRINT_EVERY  = 0.5                # ログ制御してる用
DT           = 0.02               # ループ周期（50Hz）

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def main():
    robot = Robot(left=LEFT_PINS, right=RIGHT_PINS)
    adcs  = [MCP3004(channel=i) for i in ADC_CHANNELS]

    print("sensorToMotors: start (Ctrl+C to stop) ")
    print(f"SPEED={SPEED:.2f}  THRESH={THRESH:.2f}  WHITE_IS_HIGH={WHITE_IS_HIGH}")

    last_log = 0.0
    try:
        while True:
            vals = np.array([a.value for a in adcs])  # 0..1, 通常 白が大/黒が小
            if not WHITE_IS_HIGH:
                vals = 1.0 - vals       

            left_on  = bool((vals[LEFT_GROUP]  > THRESH).any())
            right_on = bool((vals[RIGHT_GROUP] > THRESH).any())

            l = SPEED if left_on  else 0.0
            r = SPEED if right_on else 0.0

            # クリップと出力
            l = clamp(l, -CLAMP, CLAMP)
            r = clamp(r, -CLAMP, CLAMP)
            robot.value = (l, r)

            now = time.time()
            if now - last_log > PRINT_EVERY:
                print(f"vals={[round(x,2) for x in vals]}  onL={int(left_on)} onR={int(right_on)}  LR=({l:.2f},{r:.2f})")
                last_log = now

            time.sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()
