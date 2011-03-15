# recorder.py
# Record Arduino serial data of IMU into Sqlite database for later usage.
# Author: Yanglei Zhao
#############################

import traceback

from imu import *

def loop(sc, nr, db):
    timeout = 0
    print 'Started...'
    while True:
        (id, t, raw) = sc.read()
        if raw is None:
            # timeout
            timeout += 1
            if timeout == 3:
                raise Exception('Serial connection timed out.')
        else:
            data = nr.normalize(raw)
            db.write_data(id, t, raw, data)

def main():
    try:
        nr = Normalizer()
        db = Database()
        sc = SerialCom()
        loop(sc, nr, db)
    except KeyboardInterrupt, e:
        print 'Data recorded in %s.' % db.filename
        del db
    except Exception, e:
        traceback.print_exc()

if __name__ == '__main__':
    main()
