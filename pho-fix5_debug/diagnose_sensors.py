#!/usr/bin/python3
# coding: UTF-8
"""
センサー診断ログ

RAW と正規化値を一定間隔でログ出力します。
"""
from __future__ import annotations

import logging
import sys
import time
from datetime import datetime
from pathlib import Path

from gpiozero import MCP3004

from tuning import SENSOR_MAX, SENSOR_MIN


# ==============================
# 設定（必要に応じて編集）
# ==============================
NUM_CH = 4
INTERVAL_SEC = 0.2


def setup_logger() -> logging.Logger:
    log_dir = Path(__file__).with_name("logs")
    log_dir.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"sensor_diag_{ts}.log"

    logger = logging.getLogger("sensor_diag")
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
    logger.info("センサー診断開始 (SENSOR_MIN=%.3f, SENSOR_MAX=%.3f)", SENSOR_MIN, SENSOR_MAX)

    photorefs = [MCP3004(channel=idx) for idx in range(NUM_CH)]
    denom = max(1e-6, SENSOR_MAX - SENSOR_MIN)

    try:
        while True:
            raw = [pr.value for pr in photorefs]
            norm = [max(0.0, min(1.0, (v - SENSOR_MIN) / denom)) for v in raw]
            logger.info("RAW=%s NORM=%s", [f\"{v:.2f}\" for v in raw], [f\"{v:.2f}\" for v in norm])
            time.sleep(INTERVAL_SEC)
    except KeyboardInterrupt:
        logger.info("診断終了")


if __name__ == "__main__":
    main()
