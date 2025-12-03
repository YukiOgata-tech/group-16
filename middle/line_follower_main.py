#!/usr/bin/env python3
"""
line_follower_main.py
中間発表用ラインフォロワーメインプログラム

4つのフォトリフレクタ（MCP3004経由）で黒線を検出し、
DRV8835モータドライバーで2つのDCモータを制御して楕円コースを走行する。

使用方法:
    python3 line_follower_main.py

停止:
    Ctrl+C で安全に停止

設定変更:
    config.py で速度・ステアリング感度などを調整可能
"""

from sensors import PhotoArray
from motors import MotorDriver
from controller import compute_steer, mixer
import time
import sys

def main():
    """メインループ"""
    print("=" * 60)
    print("ラインフォロワー起動中...")
    print("=" * 60)

    # センサーとモータの初期化
    try:
        sensors = PhotoArray()
        motors = MotorDriver()
        print("[OK] センサー・モータの初期化完了")
    except Exception as e:
        print(f"[ERROR] 初期化失敗: {e}")
        print("SPI が有効になっているか確認してください:")
        print("  sudo raspi-config -> Interface Options -> SPI -> Enable")
        sys.exit(1)

    # 起動時のセンサーテスト
    print("\n起動時センサーチェック...")
    try:
        for i in range(3):
            raw_values = sensors.read_raw()
            print(f"  センサー値: {[f'{v:.3f}' for v in raw_values]}")
            time.sleep(0.2)
        print("[OK] センサー正常")
    except Exception as e:
        print(f"[ERROR] センサーエラー: {e}")
        motors.stop()
        sys.exit(1)

    # メインループ開始
    print("\n" + "=" * 60)
    print("ライントレース開始！ (Ctrl+C で停止)")
    print("=" * 60)

    loop_count = 0
    start_time = time.time()

    try:
        while True:
            # センサー読み取り（黒線の強さ 0-1）
            blackness = sensors.read_blackness()

            # ライン位置を計算（-1.5〜+1.5の範囲）
            position = compute_steer(blackness)

            # 左右モータ速度を計算
            left_speed, right_speed = mixer(position)

            # モータ制御
            motors.drive(left_speed, right_speed)

            # 定期的にステータス表示（5秒ごと）
            loop_count += 1
            if loop_count % 250 == 0:  # 50Hz × 5秒 = 250ループ
                elapsed = time.time() - start_time
                print(f"[{elapsed:6.1f}s] センサー: {[f'{b:.2f}' for b in blackness]} "
                      f"| 位置: {position:+.2f} | モータ: L={left_speed:.2f}, R={right_speed:.2f}")

            # 50Hzで動作
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("停止信号を受信しました")
        print("=" * 60)
    except Exception as e:
        print(f"\n[ERROR] 実行中エラー: {e}")
    finally:
        # 終了処理
        motors.stop()
        elapsed = time.time() - start_time
        print(f"\nモータ停止完了")
        print(f"総走行時間: {elapsed:.1f}秒")
        print("プログラム終了\n")

if __name__ == "__main__":
    main()
