#!/usr/bin/python3
# coding: UTF-8
"""
ラインフォロワ用シンプル制御クラス（mbd_phs2 と同一ロジック）

4 個のフォトリフレクタの出力から、固定の行列マッピングで左右モータ
指令を計算する。PD のようにゲイン調整がシビアでないため、course2024
でも揺れが少なく安定しやすい。

All rights revserved 2019-2023 (c) Shogo MURAMATSU
"""
import numpy as np


def clamped(v):
    """値の範囲を [-1, 1] に制限する。"""
    return max(-1.0, min(1.0, float(v)))


class LFController:
    """ラインフォロワ用シンプル制御クラス

    - 入力: フォトリフレクタ値 [0,1]×4（白=1, 黒=0）
    - 出力: モータ指令       [-1,1]×2（左,右）

    mbd_phs2 と同じ行列 `mat_A` を用いて、センサ値から左右モータの
    基本速度と旋回量を同時に決める。
    """

    def __init__(self, prs=None):
        self._prs = prs

    def prs2mtrs(self):
        """4 個のフォトリフレクタ値を 2 個のモータ指令に変換する。"""

        # センサ値ベクトル（白=1, 黒=0）
        vec_prs = np.array([self._prs[idx].value for idx in range(len(self._prs))])

        # 黒さ（1 に近いほど黒）と簡単な交差点判定
        black = 1.0 - vec_prs
        is_black = black > 0.6
        num_black = int(np.count_nonzero(is_black))

        # 条件: 中央2本がしっかり黒, かつ 3 本以上黒 → 交差点中央とみなして直進優先
        if num_black >= 3 and is_black[1] and is_black[2]:
            forward = 0.35
            return (clamped(forward), clamped(forward))

        # 通常は行列マッピングで左右の速度と旋回を決定
        # Left <- 0 1 2 3 -> Right
        mat_A = np.array(
            [
                [-1.0, -0.2, 0.2, 1.0],
                [1.0, 0.2, -0.2, -1.0],
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
