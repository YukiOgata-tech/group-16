#!/usr/bin/env python3
"""
楕円コース画像生成スクリプト（pygame版）

中間発表用の楕円コース画像を生成します。
pygame を使用するため、追加のインストールは不要です。
"""

import pygame
import numpy as np

def generate_ellipse_course(width=640, height=360, line_width=30, margin=60):
    """
    楕円コース画像を生成

    Args:
        width: 画像幅（ピクセル）
        height: 画像高さ（ピクセル）
        line_width: 黒線の幅（ピクセル）
        margin: 画像端からのマージン（ピクセル）

    Returns:
        pygame Surface オブジェクト
    """
    # pygame 初期化
    pygame.init()

    # サーフェスを作成（白背景）
    surface = pygame.Surface((width, height))
    surface.fill((255, 255, 255))

    # 楕円の境界ボックス
    bbox_outer = pygame.Rect(margin, margin, width - 2*margin, height - 2*margin)
    bbox_inner = pygame.Rect(
        margin + line_width,
        margin + line_width,
        width - 2*margin - 2*line_width,
        height - 2*margin - 2*line_width
    )

    # 外側の楕円（黒）
    pygame.draw.ellipse(surface, (0, 0, 0), bbox_outer)

    # 内側の楕円（白で抜く）
    pygame.draw.ellipse(surface, (255, 255, 255), bbox_inner)

    return surface

def main():
    """メイン関数"""
    print("楕円コース画像を生成中...")

    # 楕円コース画像を生成
    course_surface = generate_ellipse_course(
        width=640,
        height=360,
        line_width=30,
        margin=60
    )

    # 保存
    output_path = "ellipse_course.png"
    pygame.image.save(course_surface, output_path)
    print(f"保存完了: {output_path}")
    print(f"画像サイズ: {course_surface.get_size()}")

    pygame.quit()

if __name__ == "__main__":
    main()
