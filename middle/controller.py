# controller.py
import numpy as np
from config import BASE_SPEED,K_STEER,MIN_SPEED,POSITION_WEIGHTS
def compute_steer(b):
    w=np.array(POSITION_WEIGHTS,float);b=np.array(b,float)
    pos=float((w@b)/(b.sum()+1e-6));return pos
def mixer(pos):
    steer=K_STEER*pos
    l=max(BASE_SPEED-steer,MIN_SPEED)
    r=max(BASE_SPEED+steer,MIN_SPEED)
    return l,r
