# coding: UTF-8
# 実機のフォトリフレクタ反応最大値が約.45だったのでそれに合わせて補正できるように変数で管理
# 1.0までとれてもここ調整同じプログラム動かせます。
SENSOR_MIN = 0.0
SENSOR_MAX = 0.45

#（白=1,黒=0）
BLACK_THRESHOLD = 0.6       # 黒判定のしきい値
CROSS_BLACK_COUNT = 3       # 交差点判定
CROSS_FORWARD = 0.40

# 走行速度系
BASE_SPEED = 0.50
MIN_BASE_SPEED = 0.20       #ミニマム速度
SPEED_SCALE = 1.00 # 全体の速度の基準を調整できるようにしている
# 旋回制御の
OUTER_GAIN = 0.60 # 外側の効き具合
INNER_GAIN = 0.20 # 内側の
LINE_WEIGHTS = (-1.5, -0.5, 0.5, 1.5)
TURN_SLOWDOWN = 0.60        # 旋回時減速用だけど急カーブなどに対応するため変化が起きないようにしたほうが安定しました！

# ラインロスト
LOST_BLACK_THRESHOLD = 0.15 # 未満でロスト
LOST_TURN_SPEED = 0.18      # 探索時の旋回速度（最終発表後追記：ここで速度設定が弱くロスト時中身では動いているけど、重さの影響でモーターが回らなかったです。）
LOST_FORWARD_SPEED = 0.00

# 出力制限
OUTPUT_CLAMP = 1.00 # モータ指令の最大値をせいげんしている
