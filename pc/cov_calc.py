# analyzer.py
# The data analyzer of the IMU
# Author: Yanglei Zhao
#############################

import sys
from imu import *
import numpy as np

def main():
    acc = []
    gyro = []
    for fname in sys.argv[1:]:
        db = Database(fname)
        data = db.read_data()

        acc_current = []
        gyro_current = []

        for row in data:
            # 9-11 is the accelerometer data.
            acc_current.append(row[9:12])
            q = Quaternion.from_gyro(row[12:15], row[2])
            gyro_current.append(q.A)

        ave_acc = np.average(acc_current, axis=0)
        for row in acc_current:
            acc.append(np.subtract(row, ave_acc))
        
        ave_gyro = np.average(gyro_current, axis=0)
        for row in gyro_current:
            gyro.append(np.subtract(row, ave_gyro))


    acc_arr = np.array(acc).T
    gyro_arr = np.array(gyro).T

    print 'Covariance matrix for Accelerometer'
    print np.cov(acc_arr)
    print 'Covariance matrix for Gyroscope'
    print np.cov(gyro_arr)
    
if __name__ == '__main__':
    main()
