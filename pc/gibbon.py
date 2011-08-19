# imu.py
# The supporting library for the sensors on Arduino board.
# Author: Yanglei Zhao
#############################

import struct
import time
import math
import sqlite3
import sys

try:
    import serial
except:
    sys.stderr.write('Warning: pySerial not installed, will not connect Arduino.\n')

import numpy as np
from matplotlib import pyplot as plt
import visual

BUF_LEN = 26
RAW_LEN = 9
HEAD_LEN = 8
COM = 'COM6'

class Parameters(object):
    # Calibrated rest level.
    SENSOR_ZERO_LEVEL = [514, 509, 516, 386, 384, 382, 0, 0, 0]
    # Calibrated sensitivity. (Because the z-axis gyro is on a different chip, the
    # data for z-axis gyro is slightly different from the x/y-axis gyro.
    SENSOR_SENSITIVITY = [104, 105, 102, 15.9142, 15.8326, 14.2891, 1300, 1300, 1300]

    # This is the covariance matrix for the noise in the gyroscope,
    # represented by quaternions. Should be used as Q matrix in the EKalman.
    GYRO_ERR_COV_S = np.matrix(
        [[ 0.09617226, 0.00035709, 0.00120697, 0.00094805],
         [ 0.00035709, 0.00563692, 0.00351737, 0.00295389],
         [ 0.00120697, 0.00351737, 0.01479248, 0.00977058],
         [ 0.00094805, 0.00295389, 0.00977058, 0.0132765 ]])

    GYRO_ERR_COV_S_S = GYRO_ERR_COV_S / 50

    # Dynamic
    GYRO_ERR_COV_D = np.matrix(
        [[ 0.50547552,  0.00170386,  0.00103366,  0.00061697],
         [ 0.00170386,  0.17675834, -0.03223344, -0.00790452],
         [ 0.00103366, -0.03223344,  0.17435359, -0.01433586],
         [ 0.00061697, -0.00790452, -0.01433586,  0.14327618]])

    # The covariance matrix for the noise in the accelerometer. Should be used
    # as R matrix in the EKalman.
    ACC_ERR_COV_S = np.matrix(
        [[ 1.25592175e-05,  5.02785656e-07, -1.48793605e-06],
         [ 5.02785656e-07,  1.49101810e-05, -8.28079731e-06],
         [-1.48793605e-06, -8.28079731e-06,  2.36853045e-05]])

    # Dynamic
    ACC_ERR_COV_D = np.matrix(
        [[ 0.04902105, 0.00640971, 0.00189323],
         [ 0.00640971, 0.03728613, 0.00115823],
         [ 0.00189323, 0.00115823, 0.06454173]])

    ACC_ERR_COV_D_L = ACC_ERR_COV_D * 50

    # The covarance matrix for the noise in the magnetometer. Should be used
    # as R matrix in the EKalman.
    MAG_ERR_COV_S = np.matrix(
        [[ 2.43683824e-03, -1.30620637e-03,  5.74294645e-05],
         [-1.30620637e-03,  8.00758180e-04, -1.24840836e-04],
         [ 5.74294645e-05, -1.24840836e-04,  1.08079276e-04]])

    # Dynamic
    MAG_ERR_COV_D = np.matrix(
        [[ 0.00211615, -0.00071693,  0.00028416],
         [-0.00071693,  0.0038208 , -0.00086872],
         [ 0.00028416, -0.00086872,  0.00254654]])

    MAG_ERR_COV_D_L = MAG_ERR_COV_D * 50

    @classmethod
    def acc_h(cls, x):
        q = x.A1
        return np.matrix([[2*(q[1]*q[3]-q[2]*q[0])],
                          [2*(q[2]*q[3]+q[1]*q[0])],
                          [1-2*(q[1]*q[1]+q[2]*q[2])]])

    @classmethod
    def acc_H(cls, x):
        q = x.A1
        return np.matrix([[-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
                          [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
                          [0, -4*q[1], -4*q[2], 0]])

    @classmethod
    def mag_h(cls, x):
        q = x.A1
        return np.matrix([[1-2*(q[2]*q[2]+q[3]*q[3])],
                          [2*(q[1]*q[2]-q[0]*q[3])],
                          [2*(q[1]*q[3]+q[0]*q[2])]])

    @classmethod
    def mag_H(cls, x):
        q = x.A1
        return np.matrix([[0, 0, -4*q[2], -4*q[3]],
                          [-2*q[3], 2*q[2], 2*q[1], -2*q[0]],
                          [2*q[2], 2*q[3], 2*q[0], 2*q[1]]])

#################### Data processing ####################
class SerialCom(object):
    def __init__(self, mode=0):
        '''
        mode=0: the auto mode, the Arduino will sequencially send data to PC.
        mode=1: the step mode, the Arduino wait for a byte of serial data
        from PC, then send data to PC.
        '''
        if mode != 0 and mode != 1:
            raise Exception('parameter "mode" should be 0 or 1.')
        self.mode = mode
        
        self.ser = serial.Serial(COM, 57600, timeout=10)

        r = self.ser.read()
        
        if self.mode == 0:
            self._write('\x00')
        else:
            self._write('\x01')

    def _write(self, byte):
        self.ser.write(byte)

    def read(self):
        '''
        -> (id, t, raw)
        id: the id of the data.
        t: in microsecond, time since the board start up.
        raw: the raw imu data.
        '''
        if self.mode == 1:
            self._write('\xf0')

        s = self.ser.read(BUF_LEN)
        if len(s) < BUF_LEN:
            # Timeout
            return (None, None, None)

        data = struct.unpack('<LLhhhhhhhhh', s)

        id = data[0]
        t = data[1]

        raw = data[2:]
        return (id, t, raw)

    def __del__(self):
        if 'ser' in dir(self):
            self.ser.close()

class Normalizer(object):
    def __init__(self):
        self.zero_level = Parameters.SENSOR_ZERO_LEVEL
        self.sensitivity = Parameters.SENSOR_SENSITIVITY

    def normalize(self, raw):
        '''
        -> data: the normalized IMU data.
        raw: the raw IMU data.
        '''
        data = []
        for i in range(RAW_LEN):
            data.append((raw[i] - self.zero_level[i]) * 1.0 / self.sensitivity[i])
        return self.align_axis(data)

    def balance(self, raw):
        '''
        Balance the raw data by subtract the raw data by self.zero_level.
        '''
        data = []
        for i in range(RAW_LEN):
            data.append(raw[i] - self.zero_level[i])

        return data

    def scale(self, raw):
        '''
        Scale the raw data by divide the raw data by sensitivity.
        '''
        data = []
        for i in range(RAW_LEN):
            data.append(raw[i] * 1.0 / self.sensitivity[i])

        return data

    def align_axis(self, data):
        '''
        The device's axis are not aligned. This function helps align the data.
        '''
        # For this device
        # x = -ax, y = -ay, z =  gz
        # x =  gx, y =  gy, z =  gz
        # x = -mx, y =  my, z = -mz
        # We need to change the accelerometer data to align the axis.
        data[0] = -data[0]
        data[1] = -data[1]
        
        # As well as the magnetometer
        data[6] = -data[6]
        data[8] = -data[8]
        return data

    def get_x_mag(self, mag, gravity):
        '''
        Get the horizontal projection of the magnetic field vector by removing
        component toward gravity.
        '''
        mag = np.array(mag)
        gravity = np.array(gravity)
        return mag - gravity / np.dot(gravity, gravity) * np.dot(gravity, mag)

class Database(object):
    def __init__(self, file_name=None):
        self.SQL_CREATE_TABLE = '''CREATE TABLE data
        (id INTEGER PRIMARY KEY AUTOINCREMENT,
        raw_id INTEGER, time INTEGER,
        raw_ax INTEGER, raw_ay INTEGER, raw_az INTEGER,
        raw_gx INTEGER, raw_gy INTEGER, raw_gz INTEGER,
        raw_mx INTEGER, raw_my INTEGER, raw_mz INTEGER,
        ax REAL, ay REAL, az REAL,
        gx REAL, gy REAL, gz REAL,
        mx REAL, my REAL, mz REAL);'''

        self.SQL_INSERT_DATA = '''INSERT INTO data
        (raw_id, time,
        raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz,
        raw_mx, raw_my, raw_mz, ax, ay, az, gx, gy, gz,
        mx, my, mz) VALUES
        (?, ?, ?, ?, ?, ?,
        ?, ?, ?, ?, ?, ?,
        ?, ?, ?, ?, ?, ?, ?, ?);'''

        self.SQL_SELECT_DATA = '''SELECT * FROM data;'''

        if file_name == None:
            fname = time.strftime('%H%M%S-%b%d%Y') + '.gib'
        else:
            fname = file_name

        self._filename = fname
        self.conn = sqlite3.connect(fname)

        try:
            self.conn.execute(self.SQL_CREATE_TABLE)
        except:
            pass

        self.conn.row_factory = sqlite3.Row
        self.cur = self.conn.cursor()

    @property
    def filename(self):
        return self._filename

    def write_data(self, id, t, raw, data):
        para = [id, t]
        para.extend(raw)
        para.extend(data)
        self.cur.execute(self.SQL_INSERT_DATA, para)

    def read_all_data(self):
        '''
        Get all data from the database.
        '''
        self.cur.execute(self.SQL_SELECT_DATA)
        return self.cur.fetchall()

    def read(self):
        '''
        Get next row from the database.
        '''
        return self.cur.fetchone()

    def __del__(self):
        if 'conn' in dir(self):
            self.conn.commit()
            self.conn.close()

#################### The model ####################
class Quaternion(object):
    SUB_GRID = np.ix_([1,2,3], [1,2,3])
    def __init__(self, array=[1, 0, 0, 0]):
        '''
        array: a length 4 array, default to a zero-rotation
        '''
        if type(array) in (np.matrix, np.ndarray):
            if array.shape == (1, 4):
                self.q = np.matrix(array).T
            elif array.shape == (4, 1) or array.shape == (4, ):
                self.q = np.matrix(array)
            else:
                raise Exception('Shape of the matrix or ndarray is not valid')

        elif type(array) in (list, tuple):
            if len(array) == 4:
                self.q = np.matrix(array).T
            else:
                raise Exception('Length of the array is not valid')

        else:
            raise Exception('Not valid parameter "array", type %s' % type(array))

        self.unitize()

    @classmethod
    def from_gyro(cls, gyro, dt):
        theta = norm3(gyro)

        if theta == 0:
            v = (1, 0, 0)
        else:
            v = (gyro[0] / theta, gyro[1] / theta, gyro[2] / theta)
        
        theta = theta * dt
        return cls.from_rotation_vector(v, theta)
        
    @classmethod
    def from_rotation_vector(cls, v, theta):
        if theta == 0:
            return cls([1, 0, 0, 0])
        
        q0 = np.cos(theta / 2)
        q1 = v[0] * np.sin(theta / 2)
        q2 = v[1] * np.sin(theta / 2)
        q3 = v[2] * np.sin(theta / 2)
        return cls([q0, q1, q2, q3])

    @classmethod
    def random(cls):
        """
        This algorithm comes from http://planning.cs.uiuc.edu/node198.html
        """
        u1, u2, u3 = np.random.random(3)

        q0 = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
        q1 = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)    
        q2 = math.sqrt(u1) * math.sin(2 * math.pi * u3)
        q3 = math.sqrt(u1) * math.cos(2 * math.pi * u3)

        return cls((q0, q1, q2, q3))

    @property
    def rotation_vector(self):

        abs_q = math.sqrt(self.q[1] * self.q[1] + self.q[2] * self.q[2]\
                      + self.q[3] * self.q[3])

        v = (self.q[1:] / abs_q).A1.tolist()
        theta = 2 * math.acos(self.q[0])
        return (v, theta)

    @property
    def matrix_repr(self):
        qa = self.q.A1 # array
        qm = np.matrix([[qa[0], -qa[1], -qa[2], -qa[3]],
                         [qa[1], qa[0], -qa[3], qa[2]],
                         [qa[2], qa[3], qa[0], -qa[1]],
                         [qa[3], -qa[2], qa[1], qa[0]]])
        return qm

    @property
    def neg_matrix_repr(self):
        qa = self.q.A1
        qm = np.matrix([[qa[0], -qa[1], -qa[2], -qa[3]],
                         [qa[1], qa[0], qa[3], -qa[2]],
                         [qa[2], -qa[3], qa[0], qa[1]],
                         [qa[3], qa[2], -qa[1], qa[0]]])

        return qm

    @property
    def M(self):
        '''
        To matrix form.
        '''
        return self.q

    @property
    def A(self):
        '''
        To flattened array form.
        '''
        return self.q.A1

    @property
    def RM(self):
        '''
        -> The corresponding rotation matrix of the quaternion.
        '''
        return (self.neg_matrix_repr.T * self.matrix_repr)[self.SUB_GRID]

    def __mul__(self, q2):
        '''
        Quaternion multiplication.
        '''
        return Quaternion(self.matrix_repr * q2.M)

    def __neg__(self):
        '''
        Negative of the Quaternion.
        '''
        a = self.A
        a[1] = -a[1]
        a[2] = -a[2]
        a[3] = -a[3]
        return Quaternion(a)

    def __repr__(self):
        return '<Quaternion q0=%s, q1=%s, q2=%s, q3=%s>' % tuple(self.q.A1)

    def unitize(self):
        self.q = self.q / norm4(self.q)

class Gesture(object):
    INF = 32767.0
    STATIC_THRESHOLD = 0.40

    @classmethod
    def from_db(cls, db, sample_interval=0.05):
        gesture = cls()
        prev_sample_t = 0

        data = db.read_all_data()

        for row in data:
            t = row['time'] * 0.000001
            if t - prev_sample_t < sample_interval:
                continue
            prev_sample_t = t
            gesture.append((row['gx'], row['gy'], row['gz']))

        gesture.trim_head()
        gesture.trim_tail()
        return gesture

    def __init__(self, array=[]):
        self.s = [np.array(row) for row in array]

    def __len__(self):
        return len(self.s)

    def append(self, data):
        self.s.append(np.array(data))

        for j in xrange(len(self.s[-1])):
            if self.s[-1][j] >= 2:
                self.s[-1][j] = 1.6
            elif self.s[-1][j] <= -2:
                self.s[-1][j] = -1.6
            elif self.s[-1][j] >= 1:
                self.s[-1][j] = (self.s[-1][j] - 1) / 2 + 1
            elif self.s[-1][j] <= -1:
                self.s[-1][j] = (self.s[-1][j] + 1) / 2 - 1

    @property
    def copy(self):
        return Gesture(self.s)

    def pop_head(self):
        if len(self.s) > 0:
            del self.s[0]
        return self

    def trim_head(self):
        while len(self.s) > 0 and norm3(self.s[0]) <= self.STATIC_THRESHOLD:
            del self.s[0]
        return self

    def trim_tail(self):
        while len(self.s) > 0 and norm3(self.s[-1]) <= self.STATIC_THRESHOLD:
            self.s.pop()
        return self

    def dtw_distance(self, seq, w):
        len_a = len(self.s)
        len_b = len(seq.s)
        dtw = np.empty((len_a+1, len_b+1))

        dtw.fill(self.INF)

        dtw[0][0] = 0
        for i in xrange(len_a):
            for j in xrange(max(0, i-w), min(len_b, i+w)):
                dtw[i+1][j+1] = norm3(self.s[i]-seq.s[j]) +\
                    min(dtw[i][j+1], dtw[i][j], dtw[i+1][j])

        return dtw[len_a][len_b] * 1.0 / (len_a + len_b)

class EKalman(object):
    '''
    The Kalman Filter class for the IMU.
    '''
    def __init__(self, array=[1, 0, 0, 0], Q=Parameters.GYRO_ERR_COV_D):
        self.x = Quaternion(array)
        self.p = Q

    def unitize(self, data):
        v = np.array(data)
        return v / norm3(v)

    def naive_time_update(self, gyro, dt):
        q = Quaternion.from_gyro(gyro, dt)
        self.x = self.x * q

    def time_update(self, gyro, dt, Q):
        '''
        The time update phase of the EKF.
        '''
        q = Quaternion.from_gyro(gyro, dt)
        # The rotation represented by quaternion
        # q1 * q2 means apply q2 first about XYZ,
        # then apply q1 about XYZ, which is equavalent to
        # apply q1 first about XYZ, then q2 about xyz.
        self.xminus = self.x * q
        # q1 * q2 = q1.matrix_repr * q2 = q2.neg_matrix_repr * q1
        A = q.neg_matrix_repr
        self.pminus = A * self.p * A.T + Q

    def measurement_update(self, data, R, h_func, H_func):
        '''
        The measurement update phase of the EKF.
        '''
        I = np.identity(4)
        z = np.matrix(self.unitize(data)).T

        H = H_func(self.xminus.M)

        K = self.pminus * H.T * (H * self.pminus * H.T + R).I
        self.x = Quaternion(self.xminus.M + K * (z - h_func(self.xminus.M)))
        self.p = (I - K * H) * self.pminus
    
    @property
    def quat(self):
        return self.x

#################### Visualizing ####################
class Plotter(object):
    def __init__(self):
        self.COLORS = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w']

    def show_plot(self, data, t_idx, data_idx, title=None, text=None):
        '''
        Show the plot of the data.
        '''
        fig = plt.figure()
        if title is not None:
            fig.suptitle(title)
        
        sub = fig.add_subplot(111)

        t_array = np.array([row[t_idx] for row in data]) / 1000000.0

        for i, data_i in enumerate(data_idx):
            d_array = np.array([row[data_i] for row in data])
            sub.plot(t_array, d_array, c=self.COLORS[i], marker='-')

        plt.show()

class Visualizer(object):
    def __init__(self):
        # For converting
        self.rt_mx = np.matrix([[1, 0, 0],
                                [0, 0, 1],
                                [0, -1, 0]])

        self.x = visual.arrow(color=(1,0,0))
        self.y = visual.arrow(color=(0,1,0))
        self.z = visual.arrow(color=(0,0,1))

    def _cnvrt_axis(self, rm):
        return self.rt_mx * rm
    
    def show(self, quat):
        rm = self._cnvrt_axis(quat.RM).T.A
        self.x.pos = rm[0] * 0.05
        self.x.axis = rm[0]
        self.x.up = rm[1]
        self.y.pos = rm[1] * 0.05
        self.y.axis = rm[1]
        self.y.up = rm[2]
        self.z.pos = rm[2] * 0.05
        self.z.axis = rm[2]
        self.z.up = rm[0]

#################### Utils ####################

def norm3(a):
    return math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])

def norm4(a):
    return math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3])
