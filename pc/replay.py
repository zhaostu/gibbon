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
    for row in data:
        #kalman.naive_update(row[12:15], row[2] / 1000000.0 - t)
        kalman.time_update((row['gx'], row['gy'], row['gz']), row['time'] / 1000000.0 - t,
                           Parameters.GYRO_ERR_COV_S)
        
        kalman.measurement_update((row['ax'], row['ay'], row['az']),
                                  Parameters.ACC_ERR_COV_S,
                                  Parameters.acc_h,
                                  Paramegers.acc_H)
        t = row['time'] / 1000000.0
        vi.show(kalman.quat)
    
if __name__ == '__main__':
    main()
