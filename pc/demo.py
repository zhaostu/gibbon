# demo.py
# Get Arduino serial data of IMU and visualize it.
# Author: Yanglei Zhao
#############################

import traceback
import sys

from gibbon import *

gesture = Gesture()
ud = Gesture()

gesture_mode = False
prev_x = None
sum_x = None
num = 0

def main():
    db = Database('ud2.gib')
    ud_data = db.read_all_data()

    for i, row in enumerate(ud_data):
        if row['time'] >= 0.5 * 1000000 and row['time'] <= 1.5 * 1000000 and i % 15 == 0:
            ud.append((row['gx'], row['gy'], row['gz']))
    
    if len(sys.argv) > 1:
        db = sys.argv[1]
    else:
        db = None
    try:
        demo = Demo(database=db, visualizer=False)
        demo.register(iteration)
        print 'Started...'
        demo.run()
    except (KeyboardInterrupt, Exception), e:
        traceback.print_exc()

def iteration(i, gyro, acc, mag, t, x):
    global gesture_mode, prev_x, sum_x, num
    if i % 15 == 0:
        gesture.append(gyro)
        if len(gesture) > len(ud):
            gesture.pop()

    if not gesture_mode:
        if i % 30 == 0:
            x = ud.dtw_distance(gesture, 5)
            if x < 0.6:
                print 'Pointing gesture detected.'
                print x, t
                gesture_mode = True
    else:
        if norm3(gyro) < 0.30:
            # Stable
            if prev_x is None:
                prev_x = x
                sum_x = x
                num = 1
            else:
                if norm3(prev_x - x) < 0.15:
                    sum_x += x
                    prev_x = x
                    num += 1
                else:
                    sum_x = x
                    prev_x = x
                    num = 1

        elif prev_x is not None:
            if num < 100:
                # Clear
                sum_x = None
                prev_x = None
                num = 0
            else:
                print sum_x / num
                gesture_mode = False
                sum_x = None
                prev_x = None
                num = 0

if __name__ == '__main__':
    main()
