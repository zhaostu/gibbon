from math import *
import random
from gibbon import *
import time


class Emulator():
    FREQ = 50.0 # Hz
    FREQ_MAG = 10.0
    DURATION = 2.0 # seconds

    GYRO_ERROR = 0.2
    ACC_ERROR = 0.4
    MAG_ERROR = 0.2
    
    def __init__(self, database=None):
        self.state = Quaternion()
        self.id = 0
        self.t = 0

        self.database = database

    def run(self):
        while True:
            # v: unit rotation vector
            # theta: rotation angle in radian
            # of a random rotation quaternion for one pointing gesture
            (v, theta) = Quaternion.random().rotation_vector

            # Split up for a certain duration
            theta = theta * 1.0 / self.FREQ / self.DURATION

            q = Quaternion.from_rotation_vector(v, theta)

            for i in xrange(int(self.FREQ * self.DURATION)):
                self.state *= q
                self.generate_data(q)

    def generate_data(self, q):
        """
        Emulate data generation of sensors given the true state.
        """

        # Generate gyroscope data
        gyro, theta = q.rotation_vector
        for i in range(len(gyro)):
            gyro[i] *= theta * self.FREQ
            # Adding random noise to gyro data
            gyro[i] += random.normalvariate(0, self.GYRO_ERROR)
        
        # Generate accelerometer data
        acc = Parameters.acc_h(self.state.M).A1.tolist()
        # Adding random noise to gyro data
        for i in range(len(acc)):
            acc[i] += random.normalvariate(0, self.ACC_ERROR)

        # Generate magnetometer data
        mag = Parameters.mag_h(self.state.M).A1.tolist()
        # Adding random noise to gyro data
        for i in range(len(mag)):
            mag[i] += random.normalvariate(0, self.MAG_ERROR)

def main():
    try:
        emu = Emulator()
        emu.run()
    except KeyboardInterrupt as e:
        pass

if __name__ == '__main__':
    main()
