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
    
    data = db.read_data()
    t = 0
    total = 0
    for row in data:
        total += nm.balance(row[3:9])[axis] * (row[2] / 1000000.0 - t)
        t = row[2] / 1000000.0

    print total        
    
if __name__ == '__main__':
    main()
