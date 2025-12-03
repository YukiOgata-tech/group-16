# sensors.py
from gpiozero import MCP3004
import numpy as np

class PhotoArray:
    def __init__(self, channels=[0,1,2,3]):
        self.adcs = [MCP3004(channel=c) for c in channels]
        n = len(self.adcs)
        self.rmin = np.full(n, 0.15)
        self.rmax = np.full(n, 0.95)
        self.prev = np.zeros(n)
    def read_raw(self):
        return np.array([a.value for a in self.adcs])
    def read_norm(self):
        raw = self.read_raw()
        self.rmin = 0.98*self.rmin + 0.02*np.minimum(self.rmin, raw)
        self.rmax = 0.98*self.rmax + 0.02*np.maximum(self.rmax, raw)
        norm = np.clip((raw-self.rmin)/(self.rmax-self.rmin+1e-3),0,1)
        out = 0.7*self.prev + 0.3*norm
        self.prev = out
        return out
    def read_blackness(self):
        return 1.0 - self.read_norm()
