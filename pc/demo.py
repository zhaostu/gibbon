# demo.py
# Get Arduino serial data of IMU and visualize it.
# Author: Yanglei Zhao
#############################

import traceback
import sys

from gibbon import *

def main():
    if len(sys.argv) > 1:
        db = sys.argv[1]
    else:
        db = None
    try:
        demo = Demo(db)
        print 'Started...'
        demo.run()
    except (KeyboardInterrupt, Exception), e:
        traceback.print_exc()

if __name__ == '__main__':
    main()
