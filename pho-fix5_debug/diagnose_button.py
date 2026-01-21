#!/usr/bin/python3
# coding: UTF-8
"""
タクトスイッチ診断ログ

ボタンの押下イベントやGPIO環境をログに残します。
ディスプレイ接続での原因特定用に、コンソールとログファイルへ出力します。
"""
from __future__ import annotations

import logging
import os
import platform
import sys
import time
from datetime import datetime
from pathlib import Path

from gpiozero import Button, Device


# ==============================
# 設定（必要に応じて編集）
# ==============================
BUTTON_PIN = 3            # GPIO番号（BCM）
PULL_UP = True            # GPIO3-GND 配線なら True
BOUNCE_TIME = 0.05        # チャタリング抑制
HOLD_TIME = 1.5           # 長押し判定
LONG_HOLD_TIME = 4.0      # 超長押し判定
SAMPLE_INTERVAL = 1.0     # 状態ログ間隔（秒）
INACTIVITY_WARN = 15.0    # 反応が無いときの警告間隔（秒）


def setup_logger() -> logging.Logger:
    log_dir = Path(__file__).with_name("logs")
    log_dir.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"button_diag_{ts}.log"

    logger = logging.getLogger("button_diag")
    logger.setLevel(logging.INFO)

    fmt = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(fmt)
    logger.addHandler(file_handler)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(fmt)
    logger.addHandler(console_handler)

    logger.info("ログファイル: %s", log_path)
    return logger


def log_environment(logger: logging.Logger) -> None:
    logger.info("Python: %s", sys.version.replace("\n", " "))
    logger.info("Platform: %s", platform.platform())
    logger.info("GPIOZERO_PIN_FACTORY: %s", os.environ.get("GPIOZERO_PIN_FACTORY"))
    logger.info("Pin factory: %s", Device.pin_factory)
    logger.info("Has /dev/gpiomem: %s", os.path.exists("/dev/gpiomem"))
    logger.info("Has /dev/gpiochip0: %s", os.path.exists("/dev/gpiochip0"))

    # /dev/gpiomem のアクセス確認
    try:
        with open("/dev/gpiomem", "rb"):
            pass
        logger.info("/dev/gpiomem: read OK")
    except Exception as exc:
        logger.warning("/dev/gpiomem: read NG (%s)", exc)


def main() -> None:
    logger = setup_logger()
    logger.info("ボタン診断開始 (GPIO%s, pull_up=%s)", BUTTON_PIN, PULL_UP)
    log_environment(logger)

    last_event = time.monotonic()
    last_sample = 0.0
    last_press = None

    try:
        button = Button(
            BUTTON_PIN,
            pull_up=PULL_UP,
            bounce_time=BOUNCE_TIME,
            hold_time=HOLD_TIME,
        )
    except Exception as exc:
        logger.error("Button 初期化失敗: %s", exc)
        logger.error("配線/権限/環境を確認してください")
        return

    def on_pressed() -> None:
        nonlocal last_event, last_press
        last_event = time.monotonic()
        last_press = last_event
        logger.info("pressed (value=%.3f)", button.value)

    def on_released() -> None:
        nonlocal last_event, last_press
        last_event = time.monotonic()
        duration = None
        if last_press is not None:
            duration = last_event - last_press
        logger.info("released (duration=%.3fs, value=%.3f)", duration or 0.0, button.value)
        if duration is not None:
            if duration >= LONG_HOLD_TIME:
                logger.info("判定: 超長押し(>=%.1fs)", LONG_HOLD_TIME)
            elif duration >= HOLD_TIME:
                logger.info("判定: 長押し(>=%.1fs)", HOLD_TIME)
            else:
                logger.info("判定: 短押し")

    def on_held() -> None:
        nonlocal last_event
        last_event = time.monotonic()
        logger.info("held (>=%.1fs)", HOLD_TIME)

    button.when_pressed = on_pressed
    button.when_released = on_released
    button.when_held = on_held

    logger.info("イベント待機中... ボタンを押してログを確認してください")

    while True:
        now = time.monotonic()

        if now - last_sample >= SAMPLE_INTERVAL:
            last_sample = now
            logger.info("state: is_pressed=%s, value=%.3f", button.is_pressed, button.value)

        if now - last_event >= INACTIVITY_WARN:
            logger.warning("一定時間イベント無し（配線/ピン/プル設定を確認）")
            last_event = now  # 繰り返し間隔を抑える

        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n終了しました")
