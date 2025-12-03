from sensors import PhotoArray
import time
pa=PhotoArray()
print("Press Ctrl+C to quit.")
while True:
    n=pa.read_norm();b=1-n
    print(["{:.2f}".format(x) for x in n],["{:.2f}".format(x) for x in b])
    time.sleep(0.05)
