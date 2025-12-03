# motors.py
from gpiozero import Robot
class MotorDriver:
    def __init__(self,left=(6,5),right=(26,27)):
        self.robot = Robot(left=left,right=right)
    def drive(self,l,r):
        self.robot.value=(max(-1,min(1,l)),max(-1,min(1,r)))
    def stop(self):
        self.robot.stop()
    def demo_sequence(self):
        import time
        seq=[("forward",(0.4,0.4)),("left",(-0.2,0.4)),("right",(0.4,-0.2)),("backward",(-0.4,-0.4)),("stop",(0,0))]
        for n,v in seq:
            print(n,v);self.drive(*v);time.sleep(0.8)
        self.stop()
