#!/usr/bin/python3
# coding: UTF-8

import numpy as np

def clamp(v):
    return max(-1.0,min(1.0,v))

class LFController:
    def __init__(self, prs=None):
        self._prs = prs
        self.sensor_lat = np.array([-50,-20,20,50],dtype=float)
        self.lat_norm = float(np.max(np.abs(self.sensor_lat)))

        self.Kp = 1.3
        self.Kd = 0.45

        self.v_base = 0.55
        self.v_min = 0.18
        self.v_hard = 0.17

        self.sharp_th = 0.70
        self.turn_slow_gain = 1.1

        self.lost_th = 0.09
        self.lost_frames_max = 6
        self.omega_lost = 0.9

        self.on_th = 0.45
        self.alpha = 0.45
        self.filt = np.zeros(4)
        self.prev_e = 0.0
        self.last_good_e = 0.0
        self.lost_count = 0

    def read(self):
        raw = np.array([p.value for p in self._prs],dtype=float)
        l = 1.0-raw
        self.filt = self.alpha*self.filt + (1.0-self.alpha)*l
        l = self.filt.copy()
        bits = (l>=self.on_th).astype(int)
        return l,bits,float(l.sum()),int(bits.sum())

    def prs2mtrs(self):
        l,b,s,on = self.read()

        if s < self.lost_th:
            self.lost_count+=1
        else:
            self.lost_count=0
        if self.lost_count>=self.lost_frames_max:
            d = np.sign(self.last_good_e) or 1.0
            v=self.v_min; o=d*self.omega_lost
            return clamp(v-o), clamp(v+o)

        b0,b1,b2,b3=b

        # 強制ハードターン
        if b0==1 and b1==0 and b2==0 and b3==0:
            self.last_good_e=-1
            return clamp(self.v_hard-1.0), clamp(self.v_hard+1.0)
        if b3==1 and b2==0 and b1==0 and b0==0:
            self.last_good_e=1
            return clamp(self.v_hard+1.0), clamp(self.v_hard-1.0)

        # 交差点：中央優先（外側の誤判定を排除）
        is_cross = (on>=3 and (b1==1 or b2==1))

        if is_cross:
            c = np.array([l[1],l[2]])
            cpos = np.array([self.sensor_lat[1],self.sensor_lat[2]])
            s2=float(c.sum())
            if s2>1e-6:
                lat=float(cpos@c)/s2
            else:
                lat=0.0
            e=lat/self.lat_norm
            e=max(-1.0,min(1.0,e))
        else:
            s_all=float(l.sum())
            if s_all>1e-6:
                lat=float(self.sensor_lat@l)/s_all
            else:
                lat=0.0
            e=lat/self.lat_norm

        self.last_good_e=e

        de=e-self.prev_e
        self.prev_e=e

        u=self.Kp*e + self.Kd*de
        u=clamp(u)

        sharp = (abs(e)>self.sharp_th and s>self.lost_th)

        if sharp:
            v=self.v_hard
            o=u
            return clamp(v-o),clamp(v+o)

        turn_mag=abs(u)
        speed_scale=1.0/(1.0+self.turn_slow_gain*turn_mag)
        v=self.v_min+(self.v_base-self.v_min)*speed_scale

        if is_cross:
            u*=0.5

        o=u
        return clamp(v-o), clamp(v+o)

    @property
    def photorefs(self):
        return self._prs

    @photorefs.setter
    def photorefs(self, prs):
        self._prs = prs
