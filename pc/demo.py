# demo.py
# Get Arduino serial data of IMU and visualize it.
# Author: Yanglei Zhao
#############################

import traceback

from imu import *

def loop(sc, nr, kalman, vi):
    timeout = 0
    prev_t = 0
    print 'Started...'
    while True:
        (id, t, raw) = sc.read()
        if raw is None:
            # timeout
            timeout += 1
            if timeout == 3:
                raise Exception('Serial connection timed out.')
        else:
            data = nr.normalize(raw)
            #kalman.naive_update(data[3:6], t / 1000000.0 - prev_t)
            kalman.time_update(data[3:6], t / 1000000.0 - prev_t)
            kalman.measurement_update(data[0:3])
            vi.show(kalman.quat)
            
            prev_t = t / 1000000.0
            

def main():
    try:
        nr = Normalizer()
        kalman = EKalman()
        vi = Visualizer(kalman.quat)
        sc = SerialCom()
        loop(sc, nr, kalman, vi)
    except (KeyboardInterrupt, Exception), e:
        traceback.print_exc()

if __name__ == '__main__':
    main()
