# analyzer.py
# The data analyzer of the IMU
# Author: Yanglei Zhao
#############################

import sys
from imu import *

def main():
    pl = Plotter()
    db = Database(sys.argv[1])
    data = db.read_data()
    # Raw accelerometer data
    #pl.show_plot(data, 'time', ('raw_ax', 'raw_ay', 'raw_az'), title='Raw Accelerometer')
    # Normalized accelerometer data
    pl.show_plot(data, 'time', ('ax', 'ay', 'az'), title='Accelerometer')

    # Raw gyroscope data
    #pl.show_plot(data, 'time', ('raw_gx', 'raw_gy', 'raw_gz'), title='Raw Gyroscope')
    # Normalized gyroscope data
    pl.show_plot(data, 'time', ('gx', 'gy', 'gz'), title='Gyroscope')

    # Raw magnetometer data
    #pl.show_plot(data, 'time', ('raw_mx', 'raw_my', 'raw_mz'), title='Raw Magnetometer')
    # Normalized gyroscope data
    pl.show_plot(data, 'time', ('mx', 'my', 'mz'), title='Magnetometer')

if __name__ == '__main__':
    main()
