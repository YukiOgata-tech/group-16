#!/usr/bin/python3
# coding: UTF-8
"""
パターン:
    1. 前進（両モーター正転）
    2. 後退（両モーター逆転）
    3. 左旋回（左モーター逆転、右モーター正転）
    4. 右旋回（左モーター正転、右モーター逆転）
    5. 停止

"""
from gpiozero import Robot
from time import sleep

# モータードライバ接続ピン
PIN_AIN1 = 6
PIN_AIN2 = 5
PIN_BIN1 = 26
PIN_BIN2 = 27

def main():
    print("=" * 60)
    print("モーター動作確認")
    print("=" * 60)
    print("左右モーター設定:")
    print(f"  左モーター: GPIO{PIN_AIN1}, GPIO{PIN_AIN2}")
    print(f"  右モーター: GPIO{PIN_BIN1}, GPIO{PIN_BIN2}")
    print("=" * 60)
    print()

    # 左右モーター設定(PWM)
    motors = Robot(left=(PIN_AIN1, PIN_AIN2), right=(PIN_BIN1, PIN_BIN2))

    try:
        # テスト1: 前進
        print("テスト1: 前進（速度0.5）")
        motors.forward(0.5)
        sleep(2)
        motors.stop()
        sleep(1)

        # テスト2: 後退
        print("テスト2: 後退（速度0.5）")
        motors.backward(0.5)
        sleep(2)
        motors.stop()
        sleep(1)

        # テスト3: 左旋回
        print("テスト3: 左旋回（速度0.5）")
        motors.left(0.5)
        sleep(2)
        motors.stop()
        sleep(1)

        # テスト4: 右旋回
        print("テスト4: 右旋回（速度0.5）")
        motors.right(0.5)
        sleep(2)
        motors.stop()
        sleep(1)

        # テスト5: 速度変化
        print("テスト5: 速度変化（0.3 → 0.7）")
        motors.forward(0.3)
        sleep(1)
        motors.forward(0.7)
        sleep(1)
        motors.stop()

        print()
        print("=" * 60)
        print("テスト完了")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\nテスト中断")
        motors.stop()

if __name__ == '__main__':
    main()
