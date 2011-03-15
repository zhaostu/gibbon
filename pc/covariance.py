# covariance.py
# Calculate covariences for the EKalman filter.
# Author: Yanglei Zhao
#############################

import sys
from gibbon import *
import numpy as np

def main():
    acc = []
    gyro = []
    mag = []
    
    for fname in sys.argv[1:]:
        db = Database(fname)
        data = db.read_data()

        acc_current = []
        gyro_current = []
        mag_current = []

        prev_m = (0, 0, 0)

        for row in data:
            acc_current.append((row['ax'], row['ay'], row['az']))
            q = Quaternion.from_gyro((row['gx'], row['gy'], row['gz']), row[2])
            gyro_current.append(q.A)

            m = (row['mx'], row['my'], row['mz'])
            if m != prev_m:
                mag_current.append(m)

        ave_acc = np.average(acc_current, axis=0)
        for row in acc_current:
            acc.append(np.subtract(row, ave_acc))
        
        ave_gyro = np.average(gyro_current, axis=0)
        for row in gyro_current:
            gyro.append(np.subtract(row, ave_gyro))

        ave_mag = np.average(mag_current, axis=0)
        for row in mag_current:
            mag.append(np.subtract(row, ave_mag))

    acc_arr = np.array(acc).T
    gyro_arr = np.array(gyro).T
    mag_arr = np.array(mag).T

    print 'Covariance matrix for Accelerometer'
    print np.cov(acc_arr)
    print 'Covariance matrix for Gyroscope'
    print np.cov(gyro_arr)
    print 'Covariance matrix for Magnetometer'
    print np.cov(mag_arr)
    
if __name__ == '__main__':
    main()
