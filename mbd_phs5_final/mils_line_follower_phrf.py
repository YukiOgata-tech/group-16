#!/usr/bin/python3
# coding: UTF-8
"""
フォトリフレクタクラス

「電子情報通信設計製図」新潟大学工学部工学科電子情報通信プログラム

All rights revserved 2019-2023 (c) Shogo MURAMATSU
"""
import pygame

class LFPhotoReflector:
    """ フォトリフレクタクラス 
    
        フォトリフレクタの応答を模擬しています。

        ノイズを加えたり応答をスケールするなど、
        実機のフォトリフレクタに合わせた調整は、
        この部分で行うとよいでしょう。
    
    """
    ACTIVE_WHITE = True # 白で1，黒で0．Falseのときは逆

    def __init__(self,course,value = 0.0):
        self._course = course
        self._value = value
        self._pos_px = [0.0, 0.0]
    
    @property
    def value(self):
        return self.measurement()
    
    @value.setter
    def value(self,value):
        self._value = value

    @property
    def pos_px(self):
        return self._pos_px

    @pos_px.setter
    def pos_px(self,pos_px):
        self._pos_px = pos_px

    def measurement(self):
        # センサ位置周辺の値をリターン
        x_px = int(self._pos_px[0]+0.5)
        y_px = int(self._pos_px[1]+0.5)     
        if 1 < y_px and y_px < self._course.height-1 and \
            1 < x_px and x_px < self._course.width-1:
            pxarray = pygame.PixelArray(self._course.image) 
            # 3x3 領域の平均を出力
            acc = 0.0
            for row in range(-1,2):
                for col in range(-1,2):
                    acc = acc + float(pxarray[x_px+col][y_px+row] > 0)
            if LFPhotoReflector.ACTIVE_WHITE:
                value = acc/9.0 # 平均値
            else:
                value = 1.0 - acc/9.0 # 平均値
        else:
            value = 0.5

        return value
