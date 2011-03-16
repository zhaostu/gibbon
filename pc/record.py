# recorder.py
# Record Arduino serial data of IMU into Sqlite database for later usage.
# Author: Yanglei Zhao
#############################

import traceback
import sys

from gibbon import *

def loop(sc, nr, db):
    timeout = 0
    print 'Started...'

    if len(sys.argv) > 2:
        total = int(sys.argv[2])
    else:
        total = -1
    i = 0
    while True:
        if i == total:
            return
        i += 1
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
        if len(sys.argv) > 1:
            db = Database(sys.argv[1] + '.gib')
        else:
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
