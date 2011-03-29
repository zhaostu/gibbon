# gyro_calibrate.py
# Calibrate the gyro data by integrate the gyro data on one axis
# Author: Yanglei Zhao
#############################

'''
Usage:

gyro_calibrate.py <database> <axis>
'''

import sys
from imu import *

amap = {'x': 3, 'y': 4, 'z': 5}

def main():
    db = Database(sys.argv[1])
    axis = amap[sys.argv[2]]
    nm = Normalizer()
    
    data = db.read_all_data()
    t = 0
    total = 0
    for row in data:
        total += nm.balance((row['raw_ax'], row['raw_ay'], row['raw_az'],
                             row['raw_gx'], row['raw_gy'], row['raw_gz']
                             row['raw_mx'], row['raw_my'], row['raw_mz']))[axis] * (row['time'] / 1000000.0 - t)
        t = row['time'] / 1000000.0

    print total
    
if __name__ == '__main__':
    main()
