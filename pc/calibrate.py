# recorder.py
# Calibrate the 6DOF IMU
# Author: Yanglei Zhao
#############################

import time
import traceback

from gibbon import *

def loop(sc):
    timeout = 0
    print 'Started...'

    while True:
        raw_input()
        (id, t, raw) = sc.read()
        if raw is None:
            # timeout
            timeout += 1
            if timeout == 3:
                break
        else:
            print raw[0], raw[1], raw[2], '',
            print raw[3], raw[4], raw[5], '',
            print raw[6], raw[7], raw[8]

def main():
    try:
        sc = SerialCom(mode=1)
        loop(sc)
    except (KeyboardInterrupt, Exception), e:
        traceback.print_exc()

if __name__ == '__main__':
    main()
