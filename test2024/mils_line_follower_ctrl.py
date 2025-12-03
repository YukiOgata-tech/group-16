#!/usr/bin/python3
# coding: UTF-8

import numpy as np

def clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


class LFController:
    """
    フォトリフレクタ4個 → モータ出力2個
    揺れを抑えるためにできるだけシンプルな P 制御 + モード制御にしている。
    """

    def __init__(self, prs=None):
        self._prs = prs

        # フォトリフレクタ左右位置（mils_line_follower_body の LF_MOUNT_POS_PRF と対応）
        # ((120,-60), (100,-20), (100,20), (120,60)) の y 成分だけ使用
        self.sensor_lat = np.array([-60.0, -20.0, 20.0, 60.0], dtype=float)
        self.lat_norm = float(np.max(np.abs(self.sensor_lat))) or 1.0

        # --- 制御ゲイン（かなり抑えめ） ---
        self.Kp_follow = 1.0       # 通常時の P ゲイン
        self.Kp_cross  = 0.7       # 交差モード時の P ゲイン（弱め）

        # --- 速度設定 ---
        self.v_base   = 0.48       # 通常時ベース速度
        self.v_cross  = 0.40       # 交差モードの速度
        self.v_min    = 0.20       # 最低速度

        # 旋回量に応じた減速（大きく曲がるときは減速）
        self.turn_slow_gain = 1.4

        # 小さい誤差を無視するデッドゾーン（直線でのビビり防止）
        self.dead_band = 0.06

        # LOST 判定
        self.lost_th = 0.10
        self.lost_frames_max = 7
        self.omega_lost = 0.8

        # センサ「黒さ」しきい値（2値化用）
        self.on_th = 0.45

        # センサ値ローパス（振動抑制）
        self.alpha = 0.6
        self.filt = np.zeros(4, dtype=float)

        # モード制御
        self.mode = "FOLLOW"
        self.cross_hold_frames = 10
        self.cross_timer = 0

        # 状態
        self.prev_e = 0.0   # いまはほぼ使わないが残しておく
        self.last_good_e = 0.0
        self.lost_count = 0

    # --------------------------------------------------
    # 内部：センサ読み取り
    # --------------------------------------------------
    def _read_sensors(self):
        # 白=1, 黒=0 → line=1-raw で黒さ
        raw = np.array([prs.value for prs in self._prs], dtype=float)
        line = 1.0 - raw

        # ローパス
        self.filt = self.alpha * self.filt + (1.0 - self.alpha) * line
        line = self.filt.copy()

        s = float(line.sum())
        bits = (line >= self.on_th).astype(int)
        on_count = int(bits.sum())
        return line, bits, s, on_count

    # --------------------------------------------------
    # 内部：モード更新
    # --------------------------------------------------
    def _update_mode(self, s, on_count, bits):
        b0, b1, b2, b3 = bits

        # LOST 優先
        if s < self.lost_th:
            self.lost_count += 1
        else:
            self.lost_count = 0
        if self.lost_count >= self.lost_frames_max:
            self.mode = "LOST"
            self.cross_timer = 0
            return

        # CROSS モードの継続
        if self.cross_timer > 0:
            self.cross_timer -= 1
            self.mode = "CROSS"
            return

        # 新しく CROSS へ入る条件：3本以上 ON かつ中央センサを含む
        if on_count >= 3 and (b1 == 1 or b2 == 1):
            self.mode = "CROSS"
            self.cross_timer = self.cross_hold_frames
            return

        # 通常
        self.mode = "FOLLOW"

    # --------------------------------------------------
    # メイン：フォトリフレクタ → モータ出力
    # --------------------------------------------------
    def prs2mtrs(self):
        line, bits, s, on_count = self._read_sensors()
        self._update_mode(s, on_count, bits)

        # LOST モード
        if self.mode == "LOST":
            d = np.sign(self.last_good_e) or 1.0
            v = self.v_min
            o = d * self.omega_lost
            return clamp(v - o), clamp(v + o)

        # 誤差 e の計算（横方向重心）
        if self.mode == "CROSS":
            # 交差中は中央2本を主に見つつ、直前の方向を濃く残す
            c_vals = np.array([line[1], line[2]])
            c_lat  = np.array([self.sensor_lat[1], self.sensor_lat[2]])
            s_c = float(c_vals.sum())
            if s_c > 1e-6:
                lat = float(c_lat @ c_vals) / s_c
                e_now = lat / self.lat_norm
            else:
                e_now = 0.0

            # 直前の方向とのブレンド（急に方向が変わらないように）
            e = 0.5 * e_now + 0.5 * self.last_good_e
        else:
            # FOLLOW：4本全部で重心
            s_all = float(line.sum())
            if s_all > 1e-6:
                lat = float(self.sensor_lat @ line) / s_all
                e = lat / self.lat_norm
            else:
                e = 0.0

        # 正規化 & デッドゾーン
        e = max(-1.0, min(1.0, e))
        if abs(e) < self.dead_band:
            e = 0.0

        self.last_good_e = e

        # 制御ゲインと速度の選択
        if self.mode == "CROSS":
            Kp = self.Kp_cross
            v_base = self.v_cross
        else:
            Kp = self.Kp_follow
            v_base = self.v_base

        # シンプル P 制御
        u = Kp * e
        u = clamp(u)

        # 旋回量に応じた減速
        turn_mag = abs(u)
        speed_scale = 1.0 / (1.0 + self.turn_slow_gain * turn_mag)
        v = self.v_min + (v_base - self.v_min) * speed_scale

        o = u
        left  = clamp(v - o)
        right = clamp(v + o)
        return left, right

    # --------------------------------------------------
    # プロパティ
    # --------------------------------------------------
    @property
    def photorefs(self):
        return self._prs

    @photorefs.setter
    def photorefs(self, prs):
        self._prs = prs
