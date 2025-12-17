#!/usr/bin/python3
# coding: UTF-8
"""
ラインフォロワー実機用プログラム（mbd_phs5アルゴリズム実装）

mbd_phs5 のシミュレーターで検証した制御アルゴリズムを
Raspberry Pi 実機用に移植したバージョンです。

制御方式:
    - 固定行列マッピング（パラメータ調整不要）
    - 交差点検出機能
    - シンプルで安定した動作

ハードウェア要件:
    - Raspberry Pi Zero W
    - MCP3004 ADC (4ch)
    - DRV8835 モータードライバー
    - LBR-127HLD フォトリフレクタ × 4

実行方法:
    $ python3 real_line_follower.py

停止方法:
    Ctrl+C

All rights reserved 2019-2025 (c) Shogo MURAMATSU / EicDesignLab contributors
"""
import numpy as np
from gpiozero import MCP3004, Robot
from signal import pause


# ========================================
# ピン設定
# ========================================
PIN_AIN1 = 6   # 左モーター正転
PIN_AIN2 = 5   # 左モーター逆転
PIN_BIN1 = 26  # 右モーター正転
PIN_BIN2 = 27  # 右モーター逆転

NUM_PHOTOREFS = 4  # フォトリフレクタ数


# ========================================
# 制御クラス（mbd_phs5 と同じロジック）
# ========================================
def clamped(v):
    """値の範囲を [-1, 1] に制限する"""
    return max(-1.0, min(1.0, float(v)))


class LFController:
    """ラインフォロワ用シンプル制御クラス

    mbd_phs5 のシミュレーターと同じ制御ロジックを実装。
    固定の行列マッピングで左右モータ指令を生成します。

    - 入力: フォトリフレクタ値 [0,1]×4（白=1, 黒=0）
    - 出力: モータ指令       [-1,1]×2（左,右）
    """

    def __init__(self, prs=None):
        self._prs = prs

    def prs2mtrs(self):
        """4個のフォトリフレクタ値を2個のモータ指令に変換する"""

        # センサ値ベクトル（白=1, 黒=0）
        vec_prs = np.array([self._prs[idx].value for idx in range(len(self._prs))])

        # 黒さ（1に近いほど黒）と簡単な交差点判定
        black = 1.0 - vec_prs
        is_black = black > 0.6
        num_black = int(np.count_nonzero(is_black))

        # 条件: 中央2本がしっかり黒, かつ 3本以上黒 → 交差点中央とみなして直進優先
        if num_black >= 3 and is_black[1] and is_black[2]:
            forward = 0.35
            return (clamped(forward), clamped(forward))

        # 通常は行列マッピングで左右の速度と旋回を決定
        # Left <- 0 1 2 3 -> Right
        mat_A = np.array(
            [
                [-1.0, -0.2, 0.2, 1.0],   # 左モーター
                [1.0, 0.2, -0.2, -1.0],   # 右モーター
            ]
        )
        vec_mtrs = np.dot(mat_A, vec_prs) + 0.2

        mtr_left, mtr_right = vec_mtrs[0], vec_mtrs[1]
        return (clamped(mtr_left), clamped(mtr_right))

    @property
    def photorefs(self):
        return self._prs

    @photorefs.setter
    def photorefs(self, prs):
        self._prs = prs


# ========================================
# メイン処理
# ========================================
def main():
    """メイン関数"""
    print("=" * 50)
    print("ラインフォロワー実機用プログラム (mbd_phs5)")
    print("=" * 50)
    print("初期化中...")

    # フォトリフレクタ（複数）設定（A/D変換）
    print(f"フォトリフレクタ初期化中... (MCP3004 CH0-{NUM_PHOTOREFS-1})")
    photorefs = [MCP3004(channel=idx) for idx in range(NUM_PHOTOREFS)]

    # 左右モーター設定（PWM）
    print(f"モータードライバー初期化中...")
    print(f"  左モーター: GPIO{PIN_AIN1}, GPIO{PIN_AIN2}")
    print(f"  右モーター: GPIO{PIN_BIN1}, GPIO{PIN_BIN2}")
    motors = Robot(left=(PIN_AIN1, PIN_AIN2), right=(PIN_BIN1, PIN_BIN2))

    # コントローラ初期化
    print("制御アルゴリズム初期化中...")
    controller = LFController(prs=photorefs)

    print("=" * 50)
    print("初期化完了！")
    print("ライントレース開始...")
    print("停止するには Ctrl+C を押してください")
    print("=" * 50)

    # ライントレース処理（ジェネレータ）
    def line_follow():
        """制御ループ（無限ジェネレータ）"""
        while True:
            # センサー値からモーター指令を計算
            left, right = controller.prs2mtrs()

            # デバッグ出力（必要に応じてコメント解除）
            # print(f"Motors: L={left:+.2f}, R={right:+.2f}")

            yield (left, right)

    # モーターに制御ジェネレータを接続
    motors.source = line_follow()

    # 停止(Ctrl+C)まで待機
    try:
        pause()
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("ライントレース停止")
        print("=" * 50)
        motors.stop()


if __name__ == '__main__':
    main()
