import sys
import traceback

import time
from math import *
import random

from gibbon import *

class Emulator():
    FREQ = 50.0 # Hz
    FREQ_MAG = 10.0
    ROTATE_DURATION = 2.0 # seconds
    POINT_DURATION = 10.0 # seconds
    MAG_ANGLE = 1.0472 # radian
    RAW_PADDING = (0, 0, 0, 0, 0, 0, 0, 0, 0)

    ACC_ERROR_D = 0.05
    ACC_ERROR_S = 0.00001
    GYRO_ERROR_D = 0.02
    GYRO_ERROR_S = 0.0004
    MAG_ERROR = 0.0028
    
    def __init__(self, database=None, gesture=None):
        self.state = Quaternion()
        self.id = 0
        self.t = 0
        self.mag_prev = None
        self.mag_prev_t = -1

        self.database = database
        self.gesture = gesture

    def run(self):
        while True:
            # Generate a pointing gesture first.
            for row in self.gesture.s:
                q = Quaternion.from_gyro((row[0], row[1], row[2]), 1 / self.FREQ)
                self.state *= q
                self.store_data(q)

            print 'Gesture finished at', self.t * 1.0 / 10**6
            # Generate a rotation
            # v: unit rotation vector
            # theta: rotation angle in radian
            # of a random rotation quaternion for one pointing gesture
            (v, theta) = Quaternion.random().rotation_vector

            # Split up for a certain duration
            theta = theta * 1.0 / self.FREQ / self.ROTATE_DURATION

            q = Quaternion.from_rotation_vector(v, theta)

            for i in xrange(int(self.FREQ * self.ROTATE_DURATION)):
                self.state *= q
                self.store_data(q)

            for i in xrange(int(self.FREQ * self.POINT_DURATION)):
                self.store_data(Quaternion(), stable=True)

            print self.state.RM.T.A[0]

    def store_data(self, q, stable=False):
        """
        Emulate data generation of sensors given the true state.
        """

        # Generate gyroscope data
        gyro, theta = q.rotation_vector
        # Generate accelerometer/magnetometer data
        acc = Parameters.acc_h(self.state.M).A1
        mag = Parameters.mag_h(self.state.M).A1
        mag = acc * sin(self.MAG_ANGLE) + mag * cos(self.MAG_ANGLE)

        acc = acc.tolist()
        mag = (mag * 0.65).tolist()

        for i in range(len(gyro)):
            gyro[i] *= theta * self.FREQ
            # Adding random noise to gyroscope data
            if stable:
                gyro[i] += random.gauss(0, sqrt(self.GYRO_ERROR_S))
            else:
                gyro[i] += random.gauss(0, sqrt(self.GYRO_ERROR_D))

        # Adding random noise to accelerometer data
        for i in range(len(acc)):
            if stable:
                acc[i] += random.normalvariate(0, sqrt(self.ACC_ERROR_S))
            else:
                acc[i] += random.normalvariate(0, sqrt(self.ACC_ERROR_D))

        # Adding random noise to magnetometer data
        for i in range(len(mag)):
            mag[i] += random.normalvariate(0, sqrt(self.MAG_ERROR))

        if self.t - self.mag_prev_t >= 1 / self.FREQ_MAG:
            self.mag_prev_t = self.t
            self.mag_prev = mag
        else:
            mag = self.mag_prev

        data = []
        data.extend(acc)
        data.extend(gyro)
        data.extend(mag)
        self.database.write_data(self.id, self.t, self.RAW_PADDING, data)

        self.id += 1
        self.t += int(1.0 / self.FREQ * 10**6)

def main():
    try:
        if len(sys.argv) > 1:
            db = Database(sys.argv[1] + '.gib')
        else:
            db = Database()

        point_db = Database('point.gib')

        emu = Emulator(database=db, gesture=Gesture.from_db(point_db, sample_interval=0.02))

        emu.run()
    except KeyboardInterrupt, e:
        print 'Data recorded in %s.' % db.filename
        del db
    except Exception, e:
        traceback.print_exc()

if __name__ == '__main__':
    main()
