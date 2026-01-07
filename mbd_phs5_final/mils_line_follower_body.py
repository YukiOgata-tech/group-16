#!/usr/bin/python3
# coding: UTF-8
"""
ラインフォロワ車両の物理モデル（mbd_phs2 と同一機体）

mbd_phs2 で使用している詳細な動力学モデルと同じパラメータ・描画
ロジックを用いることで、機体を統一しつつ course2024 での挙動を
シミュレーションする。

※元の実装は mbd_phs2/mils_line_follower_body.py を参照。

All rights revserved 2019-2023 (c) Shogo MURAMATSU
"""
from mils_line_follower_ctrl import LFController
from mils_line_follower_phrf import LFPhotoReflector
from scipy.integrate import odeint
import numpy as np
import pygame

# 車両パラメータ（mbd_phs2 と同一）
LF_MOUNT_POS_PRF = ((100, -40), (100, -20), (100, 20), (100, 40))  # mm
LF_WEIGHT = 360  # g
SHAFT_LENGTH = 50  # mm
TIRE_DIAMETER = 58  # mm

COEF_K_P = 3.0

PARAMS_MU_C = 1e-3
PARAMS_IOTA_C = 1e-4
PARAMS_ELL_C = 30e-3
PARAMS_N = 58.2
PARAMS_L_C = 2 * SHAFT_LENGTH * 1e-3
PARAMS_R_W = (1 / 2) * TIRE_DIAMETER * 1e-3
PARAMS_K_T = 2e-3
PARAMS_K_B = 2e-3
PARAMS_R_A = 2.0
PARAMS_J_C = (1 / 3) * LF_WEIGHT * 1e-3 * ((160e-3 / 2) ** 2 + (60e-3 / 2) ** 2)
PARAMS_J_M = (1 / 2) * (5e-3) * ((5e-3) ** 2)
PARAMS_J_G = 0
PARAMS_IOTA_M = 1e-6
PARAMS_IOTA_G = 0

NUM_PHOTOREFS = 4

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 128, 0)


def rotate_pos(pos, center, angle):
    rotmtx = np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return rotmtx.dot(pos - center) + center


class LFPhysicalModel:
    """ラインフォロワ車両の物理モデル（mbd_phs2 と同一仕様）"""

    def __init__(self, course, weight=LF_WEIGHT, mntposprs=LF_MOUNT_POS_PRF):
        self._course = course
        self._weight = weight
        self._mntposprs = mntposprs
        self._x_mm = SHAFT_LENGTH + 10
        self._y_mm = SHAFT_LENGTH + 10
        self._angle_rad = 0.0

        self.reset()

        self._controller = LFController()
        self._prs = [LFPhotoReflector(self._course) for _ in range(NUM_PHOTOREFS)]
        for idx in range(NUM_PHOTOREFS):
            self._prs[idx].value = 0.0
        self._controller.photorefs = self._prs

    def mtrs2twist(self, mtrs, v0, w0, fps):
        h = 1 / fps
        u_r = mtrs[1]
        u_l = mtrs[0]
        u_lin = (u_r + u_l) / 2.0
        u_rot = (u_r - u_l) / 2.0

        t = np.linspace(0, h, 2)
        y0 = [0.0 for _ in range(2)]
        y0[0] = v0
        y0[1] = w0
        y1 = odeint(self._odedydt, y0, t, args=(u_lin, u_rot))
        v1 = y1[-1][0]
        w1 = y1[-1][1]

        twist = {"linear": {"x": v1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": w1}}
        return twist

    def _odedydt(self, y, t, u_lin, u_rot):
        v = y[0]
        w = y[1]

        mc_kg = 1e-3 * self._weight

        n = PARAMS_N
        r_w = PARAMS_R_W
        k_t = PARAMS_K_T
        k_b = PARAMS_K_B
        R_a = PARAMS_R_A
        J_m = PARAMS_J_M
        J_g = PARAMS_J_G
        iota_m = PARAMS_IOTA_M
        iota_g = PARAMS_IOTA_G

        iota_bar_m = iota_m + k_t * k_b / R_a
        iota_bar_g = iota_g + (n ** 2) * iota_bar_m
        J_bar_g = J_g + (n ** 2) * J_m
        zeta = n * k_t / R_a

        mu_c = PARAMS_MU_C
        D_lin = iota_bar_g + 0.5 * mu_c * (r_w ** 2)
        J_lin = J_bar_g + 0.5 * mc_kg * (r_w ** 2)
        T_lin = J_lin / D_lin
        K_lin = COEF_K_P * (zeta * r_w) / D_lin
        N_lin = 0.5 * mc_kg * (r_w ** 2)

        iota_c = PARAMS_IOTA_C
        l_c = PARAMS_ELL_C
        L_c = PARAMS_L_C
        J_c = PARAMS_J_C
        r_c = (2 * r_w / L_c)
        D_rot = iota_bar_g + 0.5 * (mu_c * (l_c ** 2) + iota_c) * (r_c ** 2)
        J_rot = J_bar_g + 0.5 * (J_c + mc_kg * (l_c ** 2)) * (r_c ** 2)
        T_rot = J_rot / D_rot
        K_rot = COEF_K_P * (zeta * r_c) / D_rot
        N_rot = 0.5 * mc_kg * (r_c ** 2)

        dydt0 = (1 / T_lin) * (-l_c * (N_lin / D_lin) * (-w * w) - v + K_lin * u_lin)
        dydt1 = (1 / T_rot) * (-l_c * (N_rot / D_rot) * (v * w) - w + K_rot * u_rot)

        return [dydt0, dydt1]

    def drive(self, fps):
        self._sense()
        mtrs = np.asarray(self._controller.prs2mtrs())
        self.updatestate(mtrs, fps)

    def updatestate(self, mtrs, fps):
        v0_m_s = 1e-3 * self._v_mm_s
        w0_rad_s = self._w_rad_s
        twist = self.mtrs2twist(mtrs, v0_m_s, w0_rad_s, fps)
        v1_m_s = twist["linear"]["x"]
        w1_rad_s = twist["angular"]["z"]

        t = np.linspace(0, 1 / fps, 2)
        pos = [0.0 for _ in range(3)]
        pos[0] = 1e-3 * self._x_mm
        pos[1] = 1e-3 * self._y_mm
        pos[2] = self._angle_rad
        p = odeint(self._odefun, pos, t, args=(v1_m_s, w1_rad_s))
        pos = p[-1]

        self._v_mm_s = 1e3 * v1_m_s
        self._w_rad_s = w1_rad_s
        self._x_mm = 1e3 * pos[0]
        self._y_mm = 1e3 * pos[1]
        self._angle_rad = pos[2]

    def _odefun(self, pos, t, v, w):
        phi = pos[2]
        return [np.cos(phi) * v, np.sin(phi) * v, w]

    @property
    def course(self):
        return self._course

    @property
    def angle(self):
        return self._angle_rad

    def reset(self):
        self._v_mm_s = 0.0
        self._w_rad_s = 0.0

    def set_position_mm(self, x, y):
        self._x_mm = x
        self._y_mm = y

    def move_px(self, dx_px, dy_px):
        res = self._course.resolution
        self._x_mm = self._x_mm + dx_px * res
        self._y_mm = self._y_mm + dy_px * res

    def rotate(self, angle):
        self._angle_rad = angle

    def set_interval(self, interval):
        self._interval = interval

    def draw_body(self, screen):
        rect = np.asarray(self.get_rect_px())
        center = np.asarray(self.get_center_px())

        apos00 = np.dot([[1, 0, 0, 0], [0, 1, 0, 0]], rect)
        apos10 = np.dot([[1, 0, 0, 0], [0, 1, 0, 1]], rect)
        apos01 = np.dot([[1, 0, 1, 0], [0, 1, 0, 0]], rect)
        apos11 = np.dot([[1, 0, 1, 0], [0, 1, 0, 1]], rect)

        angle = self._angle_rad
        apos00 = rotate_pos(apos00, center, angle)
        apos10 = rotate_pos(apos10, center, angle)
        apos01 = rotate_pos(apos01, center, angle)
        apos11 = rotate_pos(apos11, center, angle)

        pos00 = apos00.tolist()
        pos10 = apos10.tolist()
        pos01 = apos01.tolist()
        pos11 = apos11.tolist()

        pygame.draw.polygon(screen, YELLOW, [pos00, pos01, pos11, pos10], 0)

        res = self._course.resolution

        pos_ltf = center + np.asarray([TIRE_DIAMETER / 2, -SHAFT_LENGTH]) / res
        pos_ltr = center + np.asarray([-TIRE_DIAMETER / 2, -SHAFT_LENGTH]) / res
        pos_ltf = (rotate_pos(pos_ltf, center, angle) + 0.5).astype(np.int32).tolist()
        pos_ltr = (rotate_pos(pos_ltr, center, angle) + 0.5).astype(np.int32).tolist()
        pygame.draw.line(screen, BLACK, pos_ltf, pos_ltr, int(12 / res))
        pygame.draw.circle(screen, BLACK, pos_ltf, int(6 / res))
        pygame.draw.circle(screen, BLACK, pos_ltr, int(6 / res))

        pos_rtf = center + np.asarray([TIRE_DIAMETER / 2, SHAFT_LENGTH]) / res
        pos_rtr = center + np.asarray([-TIRE_DIAMETER / 2, SHAFT_LENGTH]) / res
        pos_rtf = (rotate_pos(pos_rtf, center, angle) + 0.5).astype(np.int32).tolist()
        pos_rtr = (rotate_pos(pos_rtr, center, angle) + 0.5).astype(np.int32).tolist()
        pygame.draw.line(screen, BLACK, pos_rtf, pos_rtr, int(12 / res))
        pygame.draw.circle(screen, BLACK, pos_rtf, int(6 / res))
        pygame.draw.circle(screen, BLACK, pos_rtr, int(6 / res))

        for idx in range(NUM_PHOTOREFS):
            pos = center + np.asarray(self._mntposprs[idx]) / res
            pos = (rotate_pos(pos, center, angle) + 0.5).astype(np.int32).tolist()
            if LFPhotoReflector.ACTIVE_WHITE:
                red = (int(self._prs[idx].value * 255.0), 0, 0)
            else:
                red = (int((1.0 - self._prs[idx].value) * 255.0), 0, 0)
            pygame.draw.circle(screen, red, pos, 4)

    def get_rect_px(self):
        res = self._course.resolution
        pos_cx_mm = self._x_mm
        pos_cy_mm = self._y_mm
        car_width_mm = 2 * SHAFT_LENGTH
        car_length_mm = car_width_mm + 60
        pos_topleft_x_px = int((pos_cx_mm - 0.7 * SHAFT_LENGTH) / res + 0.5)
        pos_topleft_y_px = int((pos_cy_mm - 0.7 * SHAFT_LENGTH) / res + 0.5)
        car_width_px = (car_width_mm - 0.6 * SHAFT_LENGTH) / res
        car_length_px = (car_length_mm - 0.6 * SHAFT_LENGTH) / res
        rect = [pos_topleft_x_px, pos_topleft_y_px, car_length_px, car_width_px]
        return rect

    def get_center_px(self):
        res = self._course.resolution
        cx_mm = self._x_mm
        cy_mm = self._y_mm
        cx_px = cx_mm / res
        cy_px = cy_mm / res
        return [cx_px, cy_px]

    def _sense(self):
        res = self._course.resolution
        center_px = self.get_center_px()
        angle = self._angle_rad
        for idx in range(NUM_PHOTOREFS):
            dx, dy = self._mntposprs[idx]
            pos = np.asarray([center_px[0] + dx / res, center_px[1] + dy / res])
            pos = rotate_pos(pos, center_px, angle)
            self._prs[idx].pos_px = pos.tolist()

