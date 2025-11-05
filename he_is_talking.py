"""
Reflector → Motor + 状態トーク付き
フォトリフレクタの値でモータを制御しながら、
「左旋回中」「右旋回中」「前進中」などをリアルタイム表示。
"""

import time
from gpiozero import MCP3004, Robot

# --- 設定 ---
LEFT_CH = [0, 1]
RIGHT_CH = [2, 3]
WHITE_IS_HIGH = True
BASE = 0.4         # 直進時の基準速度
GAIN = 0.6         # 曲がりの強さ
TURN_THRESH = 0.15 # 左右の差がこの値を超えると旋回判定
PIN_AIN1, PIN_AIN2 = 6, 5
PIN_BIN1, PIN_BIN2 = 26, 27

# --- 準備 ---
robot = Robot(left=(PIN_AIN1, PIN_AIN2),
              right=(PIN_BIN1, PIN_BIN2), pwm=True)
sensors = [MCP3004(channel=i) for i in range(4)]

print("=== Reflector → Motor with Talk ===")
print("Ctrl+C で停止")

def norm(v):
    return v if WHITE_IS_HIGH else 1.0 - v

try:
    while True:
        vals = [norm(s.value) for s in sensors]
        left_mean = sum(vals[i] for i in LEFT_CH)/len(LEFT_CH)
        right_mean = sum(vals[i] for i in RIGHT_CH)/len(RIGHT_CH)

        diff = right_mean - left_mean
        left = BASE + diff * GAIN
        right = BASE - diff * GAIN
        left = max(-1, min(1, left))
        right = max(-1, min(1, right))

        # 状態判定
        if abs(left - right) < TURN_THRESH:
            state = "⬆ 前進中"
        elif left > right:
            state = "↩ 左旋回中"
        elif right > left:
            state = "↪ 右旋回中"
        else:
            state = "🛑 停止中"

        robot.value = (left, right)

        print(f"L={left_mean:.2f}  R={right_mean:.2f}  out=({left:.2f},{right:.2f})  {state}")
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    robot.stop()
    print("Stopped.")
