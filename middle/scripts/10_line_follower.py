from sensors import PhotoArray
from motors import MotorDriver
from controller import compute_steer,mixer
import time
s=PhotoArray();m=MotorDriver()
try:
    while True:
        b=s.read_blackness()
        pos=compute_steer(b)
        l,r=mixer(pos)
        m.drive(l,r)
        time.sleep(0.02)
except KeyboardInterrupt:
    m.stop()
