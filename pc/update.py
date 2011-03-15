# updater.py
# Update the database values for 6DOF IMU
# Author: Yanglei Zhao
#############################

from grad import *
import sys

def main():
    nr = Normalizer()
    db = Database(sys.argv[1])

    result = db.read_data()
    for row in result:
        data = nr.normalize(row[3:9])
        db.update_data(row[0], data)
if __name__ == '__main__':
    main()
