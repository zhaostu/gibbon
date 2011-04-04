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

class Demo(object):
    def __init__(self, database=None, visualizer=True):
        if database is not None:
            # Read from database
            db = Database(database)
            self.data = db.read_all_data()
            self.db_mode = True
        else:
            # Read from serial port
            self.serial = SerialCom(mode=1)
            self.db_mode = False

        self.kalman = EKalman()
        self.nm = Normalizer()

        self.vi_mode = bool(visualizer)
        if self.vi_mode:
            self.vi = Visualizer()

        self.call_back = None
        self.reinit()

    def reinit(self):
        self.t = 0
        self.mag_prev = (0, 0, 0)

    def run(self):
        if self.db_mode:
            # Read from database.
            for i, row in enumerate(self.data):
                self._iteration(i,
                                (row['gx'], row['gy'], row['gz']),
                                (row['ax'], row['ay'], row['az']),
                                (row['mx'], row['my'], row['mz']),
                                row['time'])
                if self.call_back:
                    self.call_back(i, (row['gx'], row['gy'], row['gz']),
                                   (row['ax'], row['ay'], row['az']), (row['mx'], row['my'], row['mz']),
                                   row['time'], self.kalman.quat.RM.T.A[0])
                time.sleep(0.001)
        else:
            # Using serial port to read data.
            i = 0
            timeout = 0
            while True:
                (id, t, raw) = self.serial.read()
                if raw is None:
                    # timeout
                    timeout += 1
                    if timeout == 3:
                        raise Exception('Serial connection timed out.')
                else:
                    data = self.nm.normalize(raw)
                    self._iteration(i, data[3:6], data[:3], data[6:], t)
                    if self.call_back:
                        self.call_back(i, data[3:6], data[:3], data[6:], t, self.kalman.quat.RM.T.A[0])
                    i += 1

    def _iteration(self, i, gyro, acc, mag, t):
        GYRO_ERR_COV = Parameters.GYRO_ERR_COV_001

        total_acc = norm3(acc)
        if total_acc > 1.1 or total_acc < 0.9 or norm3(gyro) > 0.4:
            ACC_ERR_COV = Parameters.ACC_ERR_COV_D_100
        else:
            ACC_ERR_COV = Parameters.ACC_ERR_COV_D

        MAG_ERR_COV = Parameters.MAG_ERR_COV_D_100

        self.kalman.time_update(gyro, t / 1000000.0 - self.t,
                          GYRO_ERR_COV)

        rotation = self.kalman.x.RM.A
        x_mag = self.nm.get_horizontal_mag(mag, rotation[2])

        if self.mag_prev != mag:
            self.kalman.measurement_update(x_mag,
                MAG_ERR_COV, Parameters.mag_h, Parameters.mag_H)
            
            self.mag_prev = mag
        else:
            self.kalman.measurement_update(acc,
                ACC_ERR_COV, Parameters.acc_h, Parameters.acc_H)
        
        self.t = t / 1000000.0
        
        if self.vi_mode:
            self.vi.show(self.kalman.quat)

    def register(self, call_back):
        self.call_back = call_back

def main():
    db = Database('point.gib')
    ud_data = db.read_all_data()

    for i, row in enumerate(ud_data):
        if row['time'] >= 0.7 * 1000000 and row['time'] <= 1.4 * 1000000 and i % 15 == 0:
            ud.append((row['gx'], row['gy'], row['gz']))
    
    if len(sys.argv) > 1:
        db = sys.argv[1]
    else:
        db = None
    try:
        demo = Demo(database=db, visualizer=True)
        #demo.register(iteration)
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
        if i % 15 == 0:
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
