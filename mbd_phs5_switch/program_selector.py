#!/usr/bin/python3
# coding: UTF-8
"""
タクトスイッチで複数の走行プログラムを切り替えるランチャー

操作:
  - 短押し: 未起動時は候補切替 / 走行中は停止
  - 長押し(1.5秒以上): 未起動時は起動 / 走行中は停止
  - 超長押し(4.0秒以上): シャットダウン
"""
import os
import signal
import subprocess
import sys
import threading
import time

from gpiozero import Button, LED


# ========================================
# ピン設定
# ========================================
PIN_BUTTON = 3  # タクトスイッチ
PIN_LED = 23    # ステータスLED（オプション）


# ========================================
# プログラム候補
# ========================================
PROGRAMS = [
    {"name": "base", "script": "real_line_follower.py"},
    {"name": "tight", "script": "real_line_follower_alt.py"},
]

HOLD_TIME = 1.5
SHUTDOWN_TIME = 4.0


class ProgramManager:
    """走行プログラムの起動/停止を管理するクラス"""

    def __init__(self, programs, base_dir, led=None):
        self.programs = programs
        self.base_dir = base_dir
        self.led = led
        self.index = 0
        self.proc = None
        self.lock = threading.Lock()
        self._stop_event = threading.Event()
        self._monitor_thread = threading.Thread(target=self._monitor, daemon=True)
        self._monitor_thread.start()

    def _monitor(self):
        while not self._stop_event.is_set():
            proc = None
            with self.lock:
                proc = self.proc
            if proc and proc.poll() is not None:
                code = proc.returncode
                print(f"[INFO] プログラム終了: {self.current()['name']} (code={code})")
                self._clear_proc()
            time.sleep(0.2)

    def _clear_proc(self):
        with self.lock:
            self.proc = None
        if self.led:
            self.led.off()

    def current(self):
        return self.programs[self.index]

    def is_running(self):
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def cycle(self):
        if self.is_running():
            print("[WARN] 走行中は候補切替できません")
            return
        self.index = (self.index + 1) % len(self.programs)
        print(f"[INFO] 選択: {self.current()['name']} ({self.current()['script']})")

    def start(self):
        if self.is_running():
            print("[WARN] すでに走行中です")
            return

        script = self.current()["script"]
        script_path = os.path.join(self.base_dir, script)
        if not os.path.isfile(script_path):
            print(f"[ERROR] スクリプトが見つかりません: {script_path}")
            return

        cmd = [sys.executable, script_path]
        print(f"[INFO] 起動: {self.current()['name']} ({script})")
        proc = subprocess.Popen(cmd, cwd=self.base_dir)
        with self.lock:
            self.proc = proc
        if self.led:
            self.led.on()

    def stop(self):
        with self.lock:
            proc = self.proc

        if not proc:
            print("[INFO] 停止対象がありません")
            return

        print(f"[INFO] 停止: {self.current()['name']}")
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

        self._clear_proc()

    def shutdown(self):
        self._stop_event.set()
        if self.is_running():
            self.stop()

    def shutdown_system(self):
        """走行停止後にシャットダウンする"""
        print("[INFO] シャットダウン開始")
        self.stop()
        try:
            subprocess.check_call(["sudo", "wall", "Line Follower: Shutting down..."])
        except Exception as e:
            print(f"[WARN] wall 送信失敗: {e}")
        try:
            subprocess.check_call(["sudo", "poweroff"])
        except Exception as e:
            print(f"[ERROR] poweroff 実行失敗: {e}")


class ButtonHandler:
    """短押し/長押し/超長押しを判定して操作を分ける"""

    def __init__(self, button, manager, hold_time=HOLD_TIME, shutdown_time=SHUTDOWN_TIME):
        self.button = button
        self.manager = manager
        self.hold_time = hold_time
        self.shutdown_time = shutdown_time
        self.press_time = None

        self.button.when_pressed = self.on_pressed
        self.button.when_released = self.on_released

    def on_pressed(self):
        self.press_time = time.monotonic()

    def on_released(self):
        if self.press_time is None:
            return

        duration = time.monotonic() - self.press_time
        self.press_time = None

        if duration >= self.shutdown_time:
            self.manager.shutdown_system()
            return

        if duration >= self.hold_time:
            if self.manager.is_running():
                self.manager.stop()
            else:
                self.manager.start()
            return

        if self.manager.is_running():
            self.manager.stop()
        else:
            self.manager.cycle()


def main():
    print("=" * 60)
    print("タクトスイッチ プログラム切替ランチャー")
    print("=" * 60)
    print("短押し: 候補切替 / 走行中は停止")
    print("長押し(1.5秒以上): 走行開始 / 走行中は停止")
    print("超長押し(4.0秒以上): シャットダウン")
    print("=" * 60)

    # LED 初期化（オプション）
    led = None
    try:
        led = LED(PIN_LED)
        led.off()
    except Exception as e:
        print(f"[INFO] LED を使用しません: {e}")

    base_dir = os.path.dirname(os.path.abspath(__file__))
    manager = ProgramManager(PROGRAMS, base_dir, led=led)

    print(f"[INFO] 選択: {manager.current()['name']} ({manager.current()['script']})")

    button = Button(PIN_BUTTON, bounce_time=0.05)
    ButtonHandler(button, manager, hold_time=HOLD_TIME, shutdown_time=SHUTDOWN_TIME)

    try:
        signal.pause()
    except KeyboardInterrupt:
        print("\n[INFO] 終了します")
    finally:
        manager.shutdown()
        if led:
            led.off()


if __name__ == "__main__":
    main()
