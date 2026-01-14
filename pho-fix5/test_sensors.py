#!/usr/bin/python3
# coding: UTF-8
"""
フォトリフレクタ動作確認スクリプト

実機のセンサー値をリアルタイム表示します。
ライントレース実行前に、センサーが正しく動作しているか確認してください。

実行方法:
    $ python3 test_sensors.py

停止方法:
    Ctrl+C

確認項目:
    - 白い紙の上: 0.35〜0.45 付近の値（組み立て都合の想定）
    - 黒い線の上: 0.0〜0.15 付近の値
    - 4つのセンサーすべてが反応するか
"""
from gpiozero import MCP3004
from time import sleep
from tuning import SENSOR_MAX, SENSOR_MIN

# A/D変換チャネル数
NUM_CH = 4

def main():
    print("=" * 60)
    print("フォトリフレクタ動作確認")
    print("=" * 60)
    print("センサー値をリアルタイム表示します")
    print("停止するには Ctrl+C を押してください")
    print("=" * 60)
    print()

    # フォトリフレクタ（複数）設定（A/D変換）
    photorefs = [MCP3004(channel=idx) for idx in range(NUM_CH)]

    try:
        while True:
            # センサー値の読み取りと表示（raw と正規化の両方）
            values = [pr.value for pr in photorefs]
            denom = max(1e-6, SENSOR_MAX - SENSOR_MIN)
            norm_values = [max(0.0, min(1.0, (v - SENSOR_MIN) / denom)) for v in values]

            # 数値表示
            print("RAW:  ", end="")
            for idx, v in enumerate(values):
                print(f"CH{idx}:{v:4.2f}  ", end="")
            print(" | NORM: ", end="")
            for idx, v in enumerate(norm_values):
                print(f"CH{idx}:{v:4.2f}  ", end="")

            # バーグラフ表示
            print("| ", end="")
            for v in norm_values:
                bar_len = int(v * 20)
                print("█" * bar_len + "░" * (20 - bar_len) + " ", end="")

            print("\r", end="", flush=True)

            # 0.1秒待機
            sleep(0.1)

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("動作確認終了")
        print("=" * 60)

if __name__ == '__main__':
    main()
