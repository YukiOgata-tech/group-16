#!/usr/bin/python3
# coding: UTF-8
"""
ラインフォロワー実機用プログラム（タクトスイッチ制御付き）

mbd_phs5 のシミュレーターで検証した制御アルゴリズムを
Raspberry Pi 実機用に移植し、タクトスイッチで制御できるようにしたバージョンです。

タクトスイッチ操作:
    - 短押し（1秒未満）: 一時停止/再開の切り替え
    - 長押し（3秒以上）: シャットダウン

制御方式:
    - 固定行列マッピング（パラメータ調整不要）
    - 交差点検出機能
    - シンプルで安定した動作

ハードウェア要件:
    - Raspberry Pi Zero W
    - MCP3004 ADC (4ch)
    - DRV8835 モータードライバー
    - LBR-127HLD フォトリフレクタ × 4
    - タクトスイッチ × 1（GPIO 3 に接続）

実行方法:
    $ python3 real_line_follower_with_button.py

停止方法:
    - タクトスイッチ短押し（一時停止）
    - タクトスイッチ長押し（シャットダウン）
    - Ctrl+C（プログラム終了）

All rights reserved 2019-2025 (c) Shogo MURAMATSU / EicDesignLab contributors
"""
import numpy as np
from gpiozero import MCP3004, Robot, Button, LED
from subprocess import check_call
from signal import pause
from time import sleep


# ========================================
# ピン設定
# ========================================
PIN_AIN1 = 6   # 左モーター正転
PIN_AIN2 = 5   # 左モーター逆転
PIN_BIN1 = 26  # 右モーター正転
PIN_BIN2 = 27  # 右モーター逆転

PIN_BUTTON = 3  # タクトスイッチ
PIN_LED = 23    # ステータスLED（オプション）

NUM_PHOTOREFS = 4  # フォトリフレクタ数


# ========================================
# 状態管理クラス
# ========================================
class RunningState:
    """走行状態を管理するクラス"""

    def __init__(self):
        self.is_running = False  # 走行中かどうか
        self.motors = None
        self.led = None

    def toggle(self):
        """走行/停止を切り替える"""
        self.is_running = not self.is_running

        if self.is_running:
            print("\n>>> 走行開始")
            if self.led:
                self.led.on()  # LED点灯
        else:
            print("\n>>> 一時停止")
            if self.motors:
                self.motors.stop()  # モーター停止
            if self.led:
                self.led.off()  # LED消灯

    def start(self):
        """走行開始"""
        if not self.is_running:
            self.toggle()

    def stop(self):
        """一時停止"""
        if self.is_running:
            self.toggle()


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
# コールバック関数
# ========================================
def on_button_pressed(state):
    """ボタン短押し時のコールバック（一時停止/再開）"""
    state.toggle()


def on_button_held():
    """ボタン長押し時のコールバック（シャットダウン）"""
    print("\n" + "=" * 50)
    print("シャットダウン開始...")
    print("=" * 50)
    # シャットダウンメッセージ通知
    check_call(['sudo', 'wall', 'Line Follower: Shutting down...'])
    sleep(1)
    # シャットダウン
    check_call(['sudo', 'poweroff'])


# ========================================
# メイン処理
# ========================================
def main():
    """メイン関数"""
    print("=" * 50)
    print("ラインフォロワー実機用プログラム (タクトスイッチ制御)")
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

    # 状態管理オブジェクト初期化
    state = RunningState()
    state.motors = motors

    # ステータスLED初期化（オプション）
    try:
        led = LED(PIN_LED)
        state.led = led
        print(f"ステータスLED初期化完了 (GPIO{PIN_LED})")
    except Exception as e:
        print(f"ステータスLED初期化スキップ: {e}")
        state.led = None

    # タクトスイッチ初期化
    print(f"タクトスイッチ初期化中... (GPIO{PIN_BUTTON})")
    button = Button(PIN_BUTTON, hold_time=3)  # 3秒長押しでシャットダウン

    # コールバック設定
    button.when_pressed = lambda: on_button_pressed(state)
    button.when_held = on_button_held

    print("=" * 50)
    print("初期化完了！")
    print()
    print("操作方法:")
    print(f"  - タクトスイッチ短押し: 走行/停止の切り替え")
    print(f"  - タクトスイッチ長押し (3秒): シャットダウン")
    print(f"  - Ctrl+C: プログラム終了")
    print()
    print("※ タクトスイッチを短押しして走行を開始してください")
    print("=" * 50)

    # ライントレース処理（ジェネレータ）
    def line_follow():
        """制御ループ（無限ジェネレータ）"""
        while True:
            if state.is_running:
                # 走行中: センサー値からモーター指令を計算
                left, right = controller.prs2mtrs()

                # デバッグ出力（必要に応じてコメント解除）
                # print(f"Motors: L={left:+.2f}, R={right:+.2f}")

                yield (left, right)
            else:
                # 停止中: モーター停止
                yield (0, 0)

    # モーターに制御ジェネレータを接続
    motors.source = line_follow()

    # 停止(Ctrl+C)まで待機
    try:
        pause()
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("プログラム終了")
        print("=" * 50)
        motors.stop()
        if state.led:
            state.led.off()


if __name__ == '__main__':
    main()
