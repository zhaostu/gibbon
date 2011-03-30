# demo.py
# Get Arduino serial data of IMU and visualize it.
# Author: Yanglei Zhao
#############################

import traceback
import sys

from gibbon import *

gesture = Gesture()
ud = Gesture()

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
    if i % 15 == 0:
        gesture.append(gyro)
        if len(gesture) > len(ud):
            gesture.pop()
        x = gesture.dtw_distance(ud, 10)
        if x < 0.60:
            print x, t

if __name__ == '__main__':
    main()
