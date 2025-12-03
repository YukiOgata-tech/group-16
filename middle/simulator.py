#!/usr/bin/env python3
# coding: UTF-8
"""
中間発表用ラインフォロワーシミュレーター

middle フォルダの実際のコントローラー（controller.py, sensors.py）を使って
楕円コースをシミュレーション

使用方法:
    python3 simulator.py

操作:
    マウス: 車体の位置と角度を設定
    クリック: シミュレーション開始/停止
    SPACE: 開始
    ESC: 停止
    q: 終了

依存パッケージ:
    pygame, numpy, scipy
    インストール: pip install pygame numpy scipy
"""

import pygame
import numpy as np
from scipy.integrate import odeint
import sys
import math

# middle フォルダのモジュールをインポート
from controller import compute_steer, mixer
import config

# 定数
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 360
FPS = 20

# 車体パラメータ（mbd_phs2と同じ）
SHAFT_LENGTH = 50  # mm
TIRE_DIAMETER = 58  # mm
CAR_WEIGHT = 360  # g

# センサーマウント位置 [mm] (車体中心からの相対座標)
SENSOR_POSITIONS = [
    (100, -60),   # CH0: 左端
    (100, -20),   # CH1: 左中
    (100, 20),    # CH2: 右中
    (100, 60)     # CH3: 右端
]

# 色定義
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 128, 0)
RED = (255, 0, 0)

def rotate_pos(pos, center, angle):
    """座標位置の回転"""
    rotmtx = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    return rotmtx.dot(pos - center) + center

class VirtualPhotoSensor:
    """仮想フォトリフレクタ"""

    def __init__(self, course_surface):
        self.course_surface = course_surface
        self.pos_px = [0, 0]
        self._value = 0.0

    def read_blackness(self):
        """黒線の強さを読み取り（0: 白, 1: 黒）"""
        x_px = int(self.pos_px[0] + 0.5)
        y_px = int(self.pos_px[1] + 0.5)

        width = self.course_surface.get_width()
        height = self.course_surface.get_height()

        if 1 < x_px < width - 1 and 1 < y_px < height - 1:
            # 3x3 領域の平均を計算
            total = 0.0
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    # ピクセルのRGB値を取得
                    color = self.course_surface.get_at((x_px + dx, y_px + dy))
                    # 明るさを計算（RGB平均）
                    brightness = (color[0] + color[1] + color[2]) / 3.0
                    # 黒さに変換（0: 白=255, 1: 黒=0）
                    blackness = 1.0 - (brightness / 255.0)
                    total += blackness
            return total / 9.0
        else:
            return 0.5  # 範囲外は中間値

class LineFollowerSimulator:
    """ラインフォロワーシミュレーター"""

    def __init__(self, course_image_path="lfcourse.png", resolution_mm_per_pixel=2.5):
        """
        初期化

        Args:
            course_image_path: コース画像のパス
            resolution_mm_per_pixel: 解像度（mm/pixel）
        """
        pygame.init()

        # コース画像読み込み
        try:
            image = pygame.image.load(course_image_path)
            self.course_surface = pygame.transform.scale(image, (WINDOW_WIDTH, WINDOW_HEIGHT))
        except:
            print(f"警告: {course_image_path} が見つかりません。デフォルトコースを生成します。")
            self.course_surface = self._generate_default_course()

        # 画面設定
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('中間発表用ラインフォロワーシミュレーター')
        self.clock = pygame.time.Clock()

        # パラメータ
        self.resolution = resolution_mm_per_pixel  # mm/pixel

        # 車体状態
        self.x_mm = 100.0  # mm
        self.y_mm = 100.0  # mm
        self.angle_rad = 0.0  # rad
        self.v_mm_s = 0.0  # mm/s
        self.w_rad_s = 0.0  # rad/s

        # センサー
        self.sensors = [VirtualPhotoSensor(self.course_surface) for _ in range(4)]

        # シミュレーション状態
        self.state = 'locate'  # locate, rotate, wait, run
        self.drag_flag = False
        self.rot_flag = False
        self.elapsed_time = 0.0

    def _generate_default_course(self):
        """デフォルトの楕円コースを生成"""
        surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
        surface.fill(WHITE)

        # 楕円を描画
        margin = 60
        line_width = 30
        bbox_outer = pygame.Rect(margin, margin,
                                 WINDOW_WIDTH - 2*margin,
                                 WINDOW_HEIGHT - 2*margin)
        bbox_inner = pygame.Rect(margin + line_width,
                                margin + line_width,
                                WINDOW_WIDTH - 2*margin - 2*line_width,
                                WINDOW_HEIGHT - 2*margin - 2*line_width)

        pygame.draw.ellipse(surface, BLACK, bbox_outer)
        pygame.draw.ellipse(surface, WHITE, bbox_inner)

        return surface

    def get_center_px(self):
        """車体中心座標（ピクセル）"""
        return [self.x_mm / self.resolution, self.y_mm / self.resolution]

    def get_rect_px(self):
        """車体の矩形（ピクセル）"""
        center_px = self.get_center_px()
        car_width_px = (2 * SHAFT_LENGTH) / self.resolution
        car_length_px = (2 * SHAFT_LENGTH + 60) / self.resolution

        return [
            center_px[0] - car_length_px / 2,
            center_px[1] - car_width_px / 2,
            car_length_px,
            car_width_px
        ]

    def update_sensor_positions(self):
        """センサー位置を更新"""
        center_px = np.array(self.get_center_px())

        for i, sensor_pos_mm in enumerate(SENSOR_POSITIONS):
            # センサー位置をピクセルに変換
            offset_px = np.array(sensor_pos_mm) / self.resolution
            # 回転を適用
            pos_px = rotate_pos(center_px + offset_px, center_px, self.angle_rad)
            self.sensors[i].pos_px = pos_px

    def read_sensors(self):
        """センサー値を読み取り"""
        self.update_sensor_positions()
        return [sensor.read_blackness() for sensor in self.sensors]

    def simple_physics(self, left_motor, right_motor, dt):
        """
        簡単な物理モデル（速度に直接変換）

        Args:
            left_motor: 左モータ信号 [-1, 1]
            right_motor: 右モータ信号 [-1, 1]
            dt: 時間ステップ（秒）
        """
        # モータ信号を速度に変換（簡易モデル）
        max_speed_mm_s = 200.0  # 最大速度 200 mm/s

        # 左右の速度
        v_left = left_motor * max_speed_mm_s
        v_right = right_motor * max_speed_mm_s

        # 差動駆動モデル
        v_linear = (v_left + v_right) / 2.0  # 直進速度
        wheel_base = 2 * SHAFT_LENGTH  # ホイールベース
        w_angular = (v_right - v_left) / wheel_base  # 角速度

        # 位置・角度を更新（オイラー法）
        self.x_mm += v_linear * np.cos(self.angle_rad) * dt
        self.y_mm += v_linear * np.sin(self.angle_rad) * dt
        self.angle_rad += w_angular * dt

        # 速度を保存
        self.v_mm_s = v_linear
        self.w_rad_s = w_angular

    def draw_car(self):
        """車体を描画"""
        center_px = np.array(self.get_center_px())
        rect = self.get_rect_px()

        # 車体の四隅
        corners = [
            np.array([rect[0], rect[1]]),
            np.array([rect[0] + rect[2], rect[1]]),
            np.array([rect[0] + rect[2], rect[1] + rect[3]]),
            np.array([rect[0], rect[1] + rect[3]])
        ]

        # 回転を適用
        rotated_corners = [rotate_pos(c, center_px, self.angle_rad) for c in corners]

        # 車体を描画
        pygame.draw.polygon(self.screen, YELLOW, rotated_corners, 0)

        # タイヤを描画
        tire_length = TIRE_DIAMETER / self.resolution
        tire_width = 12 / self.resolution
        shaft_offset = SHAFT_LENGTH / self.resolution

        # 左タイヤ
        left_center = center_px + rotate_pos(
            np.array([0, -shaft_offset]), np.array([0, 0]), self.angle_rad
        )
        left_front = rotate_pos(
            left_center + np.array([tire_length/2, 0]), left_center, self.angle_rad
        )
        left_rear = rotate_pos(
            left_center - np.array([tire_length/2, 0]), left_center, self.angle_rad
        )
        pygame.draw.line(self.screen, BLACK, left_front, left_rear, int(tire_width))

        # 右タイヤ
        right_center = center_px + rotate_pos(
            np.array([0, shaft_offset]), np.array([0, 0]), self.angle_rad
        )
        right_front = rotate_pos(
            right_center + np.array([tire_length/2, 0]), right_center, self.angle_rad
        )
        right_rear = rotate_pos(
            right_center - np.array([tire_length/2, 0]), right_center, self.angle_rad
        )
        pygame.draw.line(self.screen, BLACK, right_front, right_rear, int(tire_width))

        # センサーを描画
        for i, sensor in enumerate(self.sensors):
            pos = sensor.pos_px
            # 黒さに応じて赤色の強度を変える
            blackness = sensor.read_blackness()
            color = (int(blackness * 255), 0, 0)
            pygame.draw.circle(self.screen, color, (int(pos[0]), int(pos[1])), 4)

    def run(self):
        """メインループ"""
        font20 = pygame.font.Font(None, 20)
        font40 = pygame.font.Font(None, 40)

        running = True
        while running:
            # イベント処理
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # マウス・キー入力
            mouseX, mouseY = pygame.mouse.get_pos()
            mBtn1, mBtn2, mBtn3 = pygame.mouse.get_pressed()
            key = pygame.key.get_pressed()

            # 背景描画
            self.screen.blit(self.course_surface, [0, 0])

            # 状態遷移
            msg = ""

            if self.state == 'locate':
                msg = 'マウスで車体を配置してください'
                # 車体をドラッグ
                rect = self.get_rect_px()
                center_px = self.get_center_px()
                is_mouse_in = (rect[0] < mouseX < rect[0] + rect[2] and
                              rect[1] < mouseY < rect[1] + rect[3])

                if (self.drag_flag or is_mouse_in) and mBtn1 == 1:
                    if not self.drag_flag:
                        self.prev_mouse = (mouseX, mouseY)
                        self.drag_flag = True
                    dx_px = mouseX - self.prev_mouse[0]
                    dy_px = mouseY - self.prev_mouse[1]
                    self.x_mm += dx_px * self.resolution
                    self.y_mm += dy_px * self.resolution
                    self.prev_mouse = (mouseX, mouseY)
                elif self.drag_flag and mBtn1 == 0:
                    self.drag_flag = False
                    self.state = 'rotate'

            elif self.state == 'rotate':
                msg = 'マウスで車体の向きを設定してください'
                if mBtn1 == 1:
                    if not self.rot_flag:
                        self.rot_flag = True
                    center_px = self.get_center_px()
                    dx = mouseX - center_px[0]
                    dy = mouseY - center_px[1]
                    self.angle_rad = math.atan2(dy, dx)
                elif self.rot_flag and mBtn1 == 0:
                    self.rot_flag = False
                    self.state = 'wait'

            elif self.state == 'wait':
                msg = 'クリックでスタート'
                self.elapsed_time = 0.0
                if mBtn1 == 1 and not self.drag_flag:
                    self.drag_flag = True
                elif self.drag_flag and mBtn1 == 0:
                    self.drag_flag = False
                    self.state = 'run'

            elif self.state == 'run':
                msg = 'クリックで停止'
                if mBtn1 == 1:
                    self.state = 'locate'
                    self.v_mm_s = 0.0
                    self.w_rad_s = 0.0
                else:
                    # センサー読み取り
                    blackness_values = self.read_sensors()

                    # 制御計算（実際のコントローラーを使用）
                    position = compute_steer(blackness_values)
                    left_motor, right_motor = mixer(position)

                    # 物理シミュレーション
                    dt = 1.0 / FPS
                    self.simple_physics(left_motor, right_motor, dt)

                    self.elapsed_time += dt

            # キーボード入力
            if key[pygame.K_ESCAPE]:
                self.state = 'locate'
                self.v_mm_s = 0.0
                self.w_rad_s = 0.0
            if key[pygame.K_SPACE]:
                if self.state == 'wait':
                    self.state = 'run'
            if key[pygame.K_q]:
                running = False

            # 描画
            self.draw_car()

            # メッセージ表示
            msg_surface = font20.render(msg, True, BLUE)
            self.screen.blit(msg_surface, [10, WINDOW_HEIGHT - 20])

            # センサー値のデバッグ表示
            if self.state in ['run', 'wait']:
                sensor_values = self.read_sensors()
                sensor_text = f"Sensors: [{sensor_values[0]:.2f}, {sensor_values[1]:.2f}, {sensor_values[2]:.2f}, {sensor_values[3]:.2f}]"
                sensor_surface = font20.render(sensor_text, True, RED)
                self.screen.blit(sensor_surface, [10, 10])

            # 経過時間表示
            smin = int(self.elapsed_time / 60) % 60
            ssec = int(self.elapsed_time) % 60
            smsc = int(100 * self.elapsed_time) % 100
            time_str = f"{smin:02d}'{ssec:02d}\"{smsc:02d}"
            time_surface = font40.render(time_str, True, GREEN)
            self.screen.blit(time_surface, [WINDOW_WIDTH - 120, WINDOW_HEIGHT - 30])

            pygame.display.update()
            self.clock.tick(FPS)

        pygame.quit()

def main():
    """メイン関数"""
    print("=" * 60)
    print("中間発表用ラインフォロワーシミュレーター")
    print("=" * 60)
    print(f"設定:")
    print(f"  BASE_SPEED: {config.BASE_SPEED}")
    print(f"  K_STEER: {config.K_STEER}")
    print(f"  MIN_SPEED: {config.MIN_SPEED}")
    print("=" * 60)

    # シミュレーター起動
    sim = LineFollowerSimulator("lfcourse.png")
    sim.run()

if __name__ == "__main__":
    main()
