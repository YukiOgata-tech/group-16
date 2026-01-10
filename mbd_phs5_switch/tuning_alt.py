# coding: UTF-8
"""
テスト走行向けチューニング（タイトコース想定）

Uターンや狭い区間を優先して安定性を上げた設定。
"""

# しきい値（白=1, 黒=0 の前提）
BLACK_THRESHOLD = 0.55       # 黒判定のしきい値（やや敏感）
CROSS_BLACK_COUNT = 3        # 交差点とみなす黒センサー数
CROSS_FORWARD = 0.30         # 交差点直進時の速度

# 走行速度系
BASE_SPEED = 0.16            # 基本速度（直進時）
MIN_BASE_SPEED = 0.10        # 最低速度
SPEED_SCALE = 0.85           # 全体速度スケール

# 旋回ゲイン（行列マッピング係数）
OUTER_GAIN = 1.10            # 外側センサーの効き（やや強め）
INNER_GAIN = 0.35            # 内側センサーの効き
LINE_WEIGHTS = (-1.5, -0.5, 0.5, 1.5)  # ライン位置推定の重み

# カーブ減速
TURN_SLOWDOWN = 0.75         # 旋回量に応じて減速する割合 (0-1)

# ラインロスト時の探索
LOST_BLACK_THRESHOLD = 0.20  # これ未満ならラインロスト扱い
LOST_TURN_SPEED = 0.20       # 探索時の旋回速度
LOST_FORWARD_SPEED = 0.00    # 探索時の前進速度

# 出力制限
OUTPUT_CLAMP = 1.00          # モータ指令の最大値
