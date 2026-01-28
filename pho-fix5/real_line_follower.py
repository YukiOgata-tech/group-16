#!/usr/bin/python3
# coding: UTF-8
"""
photoリフレクタ正規化値考慮したP
制御基準:
    - 交差点・カーブ・ロストの判定

All rights reserved グループ16♡
"""
import numpy as np
from gpiozero import MCP3004, Robot
from signal import pause

from tuning import (
    BASE_SPEED,
    BLACK_THRESHOLD,
    CROSS_BLACK_COUNT,
    CROSS_FORWARD,
    INNER_GAIN,
    LINE_WEIGHTS,
    LOST_BLACK_THRESHOLD,
    LOST_FORWARD_SPEED,
    LOST_TURN_SPEED,
    MIN_BASE_SPEED,
    OUTER_GAIN,
    OUTPUT_CLAMP,
    SENSOR_MAX,
    SENSOR_MIN,
    SPEED_SCALE,
    TURN_SLOWDOWN,
)
# ピン位置設定
PIN_AIN1 = 6
PIN_AIN2 = 5
PIN_BIN1 = 26
PIN_BIN2 = 27

NUM_PHOTOREFS = 4

# 制御のクラス
def clamped(v):
    """値の範囲制限"""
    return max(-OUTPUT_CLAMP, min(OUTPUT_CLAMP, float(v)))

def normalize_sensor(v):
    """センサー値を実機に合わせ0-1に正規化"""
    denom = max(1e-6, SENSOR_MAX - SENSOR_MIN)
    scaled = (float(v) - SENSOR_MIN) / denom
    return max(0.0, min(1.0, scaled))


class LFController:
    def __init__(self, prs=None):
        self._prs = prs
        self._last_dir = 1  # ロスト時の探索向き(右:+1,左:-1)、なのでここで初期値は右と設定
        # すいません。プレゼン資料は胆略的に説明していますが、旋回時に旋回方向（右か左）を変数で保持し探索するため。
        # 急カーブでのラインロストは、曲がり中に飛び出してしまうから。なので急停止からの最後の方向の旋回でラインを探索する感じです

    def prs2mtrs(self):
        """4個のフォトリフレクタ値を2個のモータ指令に変換"""
        # センサ値（白=1,黒=0）
        vec_prs = np.array(
            [normalize_sensor(self._prs[idx].value) for idx in range(len(self._prs))]
        )

        # 黒さ（1→黒）と交差点判定
        black = 1.0 - vec_prs
        is_black = black > BLACK_THRESHOLD
        num_black = int(np.count_nonzero(is_black))
        black_max = float(np.max(black))

        # ライン位置
        line_pos = float(np.dot(LINE_WEIGHTS, black))
        if line_pos != 0.0:
            self._last_dir = 1 if line_pos > 0 else -1
        # 中央2本が黒, プラスで3本以上黒→交差点！直進
        if num_black >= CROSS_BLACK_COUNT and is_black[1] and is_black[2]:
            forward = CROSS_FORWARD * SPEED_SCALE
            return (clamped(forward), clamped(forward))
        # ラインロスト時は、最後の向きに旋回して探索
        if black_max < LOST_BLACK_THRESHOLD:
            turn = LOST_TURN_SPEED * SPEED_SCALE * self._last_dir
            forward = LOST_FORWARD_SPEED * SPEED_SCALE
            return (clamped(forward - turn), clamped(forward + turn))
        # 通常時の左右速度と旋回
        mat_A = np.array(
            [
                [-OUTER_GAIN, -INNER_GAIN, INNER_GAIN, OUTER_GAIN],
                [OUTER_GAIN, INNER_GAIN, -INNER_GAIN, -OUTER_GAIN],   #こっち右
            ]
        )
        turn_mag = min(1.0, abs(line_pos))
        base = BASE_SPEED * (1.0 - TURN_SLOWDOWN * turn_mag)
        base = max(base, MIN_BASE_SPEED)

        vec_mtrs = (np.dot(mat_A, vec_prs) + base) * SPEED_SCALE

        mtr_left, mtr_right = vec_mtrs[0], vec_mtrs[1]
        return (clamped(mtr_left), clamped(mtr_right))

    @property
    def photorefs(self):
        return self._prs
    @photorefs.setter
    def photorefs(self, prs):
        self._prs = prs

def main():
    """メイン"""
    print("=" * 50)
    print("実機用プログラムdisplay確認用のプリントコード")
    print("ごーごー。" * 10)
    print("初期化中...")

    # PhoR設定
    print(f"フォトリフレクタ初期化中 (MCP3004 CH0-{NUM_PHOTOREFS-1})")
    photorefs = [MCP3004(channel=idx) for idx in range(NUM_PHOTOREFS)]

    # 左右モーター設定
    print(f"初期化中...")
    print(f" 左:GPIO{PIN_AIN1}, GPIO{PIN_AIN2}")
    print(f" 右:GPIO{PIN_BIN1}, GPIO{PIN_BIN2}")
    motors = Robot(left=(PIN_AIN1, PIN_AIN2), right=(PIN_BIN1, PIN_BIN2))

    # コントローラ初期化
    print("初期化中")
    controller = LFController(prs=photorefs)

    print("=" * 50)
    print("完了！")
    print("除雪開始（機体は除雪機イメージ。）")
    print("停止 Ctrl+C")
    print("=" * 50)

    # ライントレース
    def line_follow():
        while True:
            left, right = controller.prs2mtrs()
            yield (left, right)

    # モーターに制御ジェネレータ
    motors.source = line_follow()
    try:
        pause()
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("終了しまsu")
        print("=" * 50)
        motors.stop()
if __name__ == '__main__':
    main()