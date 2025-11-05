"""
感覚的には「左が暗い → 右に寄せる」「右が暗い → 左に寄せる」。
"""

import time
from gpiozero import MCP3004, Robot

# --- 設定 ---
LEFT_CH = [0, 1]   # 左センサ群
RIGHT_CH = [2, 3]  # 右センサ群
WHITE_IS_HIGH = True
BASE = 0.4         # 通常走行速度
GAIN = 0.6         # 偏り補正の強さ
PIN_AIN1, PIN_AIN2 = 6, 5
PIN_BIN1, PIN_BIN2 = 26, 27

# --- 準備 ---
robot = Robot(left=(PIN_AIN1, PIN_AIN2),
              right=(PIN_BIN1, PIN_BIN2), pwm=True)
sensors = [MCP3004(channel=i) for i in range(4)]

print("=== Reflector → Motor ===")
print("Ctrl+C で停止")

def norm(v):  # 白黒反転設定
    return v if WHITE_IS_HIGH else 1.0 - v

try:
    while True:
        vals = [norm(s.value) for s in sensors]
        left_mean = sum(vals[i] for i in LEFT_CH)/len(LEFT_CH)
        right_mean = sum(vals[i] for i in RIGHT_CH)/len(RIGHT_CH)

        diff = right_mean - left_mean   # 正なら右が白め（=左へ寄る）
        left = BASE + diff * GAIN
        right = BASE - diff * GAIN

        # クリップ
        left = max(-1, min(1, left))
        right = max(-1, min(1, right))

        robot.value = (left, right)

        print(f"L={left_mean:.2f}  R={right_mean:.2f}  out=({left:.2f},{right:.2f})")
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    robot.stop()
    print("Stopped.")
