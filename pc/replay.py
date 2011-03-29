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

    FIX_INTERVAL = 1

    prev_m = (0, 0, 0)
    mag_err_count = 0
    for i, row in enumerate(data):
        if i % FIX_INTERVAL == 0:
            if np.linalg.norm((row['gx'], row['gy'], row['gz'])) > 0.5:
                GYRO_ERR_COV = Parameters.GYRO_ERR_COV_D
                ACC_ERR_COV = Parameters.ACC_ERR_COV_D
                MAG_ERR_COV = Parameters.MAG_ERR_COV_D
            else:
                GYRO_ERR_COV = Parameters.GYRO_ERR_COV
                ACC_ERR_COV = Parameters.ACC_ERR_COV
                MAG_ERR_COV = Parameters.MAG_ERR_COV
                
            kalman.time_update((row['gx'], row['gy'], row['gz']), row['time'] / 1000000.0 - t,
                               GYRO_ERR_COV)

            if prev_m == (row['mx'], row['my'], row['mz']):
                kalman.measurement_update((row['ax'], row['ay'], row['az']),
                                          ACC_ERR_COV,
                                          Parameters.acc_h,
                                          Parameters.acc_H)
            else:
                rotation = kalman.x.RM.A
                x_mag = nm.get_horizontal_mag((row['mx'], row['my'], row['mz']), rotation[2])
                if np.dot(rotation[0], x_mag) >= 0 or mag_err_count > 2:
                    kalman.measurement_update(x_mag,
                                              MAG_ERR_COV,
                                              Parameters.mag_h,
                                              Parameters.mag_H)
                    mag_err_count = 0
                else:
                    mag_err_count += 1
                prev_m = (row['mx'], row['my'], row['mz'])
        else:
            kalman.naive_time_update((row['gx'], row['gy'], row['gz']), row['time'] / 1000000.0 - t)
        
        t = row['time'] / 1000000.0
        vi.show(kalman.quat)
    
if __name__ == '__main__':
    main()
