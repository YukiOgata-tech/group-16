# config.py
ADC_CHANNELS = [0, 1, 2, 3]
NORM_ALPHA = 0.02
LPF_ALPHA = 0.3
READ_HZ = 50
LEFT_PINS = (6, 5)
RIGHT_PINS = (26, 27)
BASE_SPEED = 0.55    # 遅く確実な速度設定（中間発表用）
K_STEER = 0.45       # ステアリング感度を少し下げて安定化
MIN_SPEED = 0.30     # 最低速度も下げる
CLAMP = 0.70         # 最大出力も制限して安全性向上
POSITION_WEIGHTS = [-1.5, -0.5, 0.5, 1.5]
