# replay.py
# replay the rotation recorded by record.py
# Author: Yanglei Zhao
#############################

import sys
from gibbon import *

def main():
    db = Database(sys.argv[1])
    kalman = EKalman()
    nm = Normalizer()
    vi = Visualizer(kalman.quat)
    
    data = db.read_data()
    t = 0

    prev_m = (0, 0, 0)
    for row in data:
        #kalman.naive_update((row['gx'], row['gy'], row['gz']), row['time'] / 1000000.0 - t)
        kalman.time_update((row['gx'], row['gy'], row['gz']), row['time'] / 1000000.0 - t,
                           Parameters.GYRO_ERR_COV)

        if prev_m == (row['mx'], row['my'], row['mz']):
            kalman.measurement_update((row['ax'], row['ay'], row['az']),
                                      Parameters.ACC_ERR_COV,
                                      Parameters.acc_h,
                                      Parameters.acc_H)
        else:
            kalman.measurement_update(nm.get_h_mag((row['mx'], row['my'], row['mz']), kalman.x.RM.A[2]),
                                      Parameters.MAG_ERR_COV,
                                      Parameters.mag_h,
                                      Parameters.mag_H)
            prev_m = (row['mx'], row['my'], row['mz'])
            
        t = row['time'] / 1000000.0
        vi.show(kalman.quat)
    
if __name__ == '__main__':
    main()
