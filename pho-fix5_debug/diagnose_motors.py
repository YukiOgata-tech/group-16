#!/usr/bin/python3
# coding: UTF-8
"""
モーター診断ログ

低速でモーターを動かし、GPIO制御が動作するかをログで確認します。
"""
from __future__ import annotations

import logging
import sys
import time
from datetime import datetime
from pathlib import Path

from gpiozero import Robot


# ==============================
# 設定（必要に応じて編集）
# ==============================
PIN_AIN1 = 6   # 左モーター正転
PIN_AIN2 = 5   # 左モーター逆転
PIN_BIN1 = 26  # 右モーター正転
PIN_BIN2 = 27  # 右モーター逆転

SPEED = 0.2    # 低速テスト
STEP_SEC = 1.0


def setup_logger() -> logging.Logger:
    log_dir = Path(__file__).with_name("logs")
    log_dir.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"motor_diag_{ts}.log"

    logger = logging.getLogger("motor_diag")
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


def main() -> None:
    logger = setup_logger()
    logger.info("モーター診断開始 (speed=%.2f)", SPEED)

    try:
        motors = Robot(left=(PIN_AIN1, PIN_AIN2), right=(PIN_BIN1, PIN_BIN2))
    except Exception as exc:
        logger.error("Robot 初期化失敗: %s", exc)
        return

    try:
        logger.info("前進")
        motors.forward(SPEED)
        time.sleep(STEP_SEC)

        logger.info("停止")
        motors.stop()
        time.sleep(0.5)

        logger.info("左旋回")
        motors.left(SPEED)
        time.sleep(STEP_SEC)

        logger.info("停止")
        motors.stop()
        time.sleep(0.5)

        logger.info("右旋回")
        motors.right(SPEED)
        time.sleep(STEP_SEC)

        logger.info("停止")
        motors.stop()
    except KeyboardInterrupt:
        logger.info("中断されました")
    finally:
        motors.stop()
        logger.info("診断終了")


if __name__ == "__main__":
    main()
