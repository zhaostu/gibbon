# demo.py
# Get Arduino serial data of IMU and visualize it.
# Author: Yanglei Zhao
#############################

import traceback
import sys

from gibbon import *

class Demo(object):
    def __init__(self, database=None, gesture=None, visualizer=True,
                 gesture_sample_interval = 0.05):
        if database is not None:
            # Read from database
            self.data = database.read_all_data()
            self.db_mode = True
        else:
            # Read from serial port
            self.serial = SerialCom(mode=1)
            self.db_mode = False

        self.vi_mode = bool(visualizer)
        if self.vi_mode:
            self.vi = Visualizer()

        self.nm = Normalizer()

        self.t = 0

        # EKalman.
        self.kalman = EKalman(Q=Parameters.GYRO_ERR_COV_D * 1000)
        self.mag_prev = (0, 0, 0)

        # Gesture
        if gesture is None:
            self.gesture_mode = False
        else:
            self.gesture_mode = True
            self.gesture = gesture
            self.gyro_seq = Gesture()

            self.gesture_sample_interval = gesture_sample_interval
            self.prev_sample_t = 0

            self.pointing_started = None
            self.start_x = None
            self.sum_x = None
            self.num_x = 0

    def run(self):
        if self.db_mode:
            # Read from database.
            for row in self.data:
                self.iteration((row['gx'], row['gy'], row['gz']),
                                (row['ax'], row['ay'], row['az']),
                                (row['mx'], row['my'], row['mz']),
                                row['time'])
        else:
            # Using serial port to read data.
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
                    self.iteration(data[3:6], data[:3], data[6:], t)

    def iteration(self, gyro, acc, mag, t):
        t = t * 0.000001
        self.kalman_iter(gyro, acc, mag, t)
        if self.gesture_mode:
            self.gesture_iter(gyro, acc, mag, t)

        if self.vi_mode:
            self.vi.show(self.kalman.quat)

        self.t = t

    def kalman_iter(self, gyro, acc, mag, t):
        if t <= 1.5:
            GYRO_ERR_COV = Parameters.GYRO_ERR_COV_D
        else:
            GYRO_ERR_COV = Parameters.GYRO_ERR_COV_S_S

        total_acc = norm3(acc)
        total_gyro = norm3(gyro)
        if total_acc > 1.1 or total_acc < 0.9 or total_gyro > 0.4:
            ACC_ERR_COV = Parameters.ACC_ERR_COV_D_L
        else:
            ACC_ERR_COV = Parameters.ACC_ERR_COV_D

        if gyro < 0.2:
            MAG_ERR_COV = Parameters.MAG_ERR_COV_D
        else:
            MAG_ERR_COV = Parameters.MAG_ERR_COV_D_L

        self.kalman.time_update(gyro, t - self.t,
                          GYRO_ERR_COV)

        rotation = self.kalman.xminus.RM.A
        # Get only the x axis of the magnetic field.
        x_mag = self.nm.get_x_mag(mag, rotation[2])

        if self.mag_prev != mag:
            self.kalman.measurement_update(x_mag,
                MAG_ERR_COV, Parameters.mag_h, Parameters.mag_H)
            
            self.mag_prev = mag
        else:
            self.kalman.measurement_update(acc,
                ACC_ERR_COV, Parameters.acc_h, Parameters.acc_H)

    def gesture_iter(self, gyro, acc, mag, t):
        if t - self.prev_sample_t >= self.gesture_sample_interval:
            self.gyro_seq.append(gyro)
            self.gyro_seq.trim_head()
            self.prev_sample_t = t

            if len(self.gyro_seq) > len(self.gesture) * 1.5:
                self.gyro_seq.pop_head()

            if self.pointing_started is None:
                score = self.gesture.dtw_distance(self.gyro_seq.copy.trim_tail(), 10)
                if score < 0.5:
                    print 'Pointing gesture detected.', score, t
                    self.pointing_started = t

        if self.pointing_started is not None:
            # Pointing started

            if t - self.pointing_started >= 2:
                # Already pointed for two seconds.
                print self.sum_x / self.num_x
                self.pointing_started = None
                self.start_x = None
                self.sum_x = None
                self.num_x = 0

            elif norm3(gyro) < Gesture.STATIC_THRESHOLD:
                # Stable
                x = self.kalman.quat.RM.T.A[0]
                if self.start_x is None or norm3(self.start_x - x) >= 0.15:
                    self.pointing_started = t
                    self.start_x = np.array(x)
                    self.sum_x = x
                    self.num_x = 1
                else:
                    self.sum_x += x
                    self.num_x += 1
            else:
                self.pointing_started = t
                self.start_x = None
                self.sum_x = None
                self.num_x = 0

def main():
    si = 0.05
    if len(sys.argv) > 1:
        db = Database(sys.argv[1])
    else:
        db = None

    point_db = Database('point.gib')
    point = Gesture.from_db(point_db, sample_interval=si)

    try:
        demo = Demo(database=db, gesture=point, visualizer=True,
                    gesture_sample_interval = si)
        print 'Started...'
        demo.run()
    except (KeyboardInterrupt, Exception), e:
        traceback.print_exc()


if __name__ == '__main__':
    main()
