# imu.py
# The supporting library for the sensors on Arduino board.
# Author: Yanglei Zhao
#############################

import struct
import serial
import time
import sqlite3
import numpy as np
import scipy
from matplotlib import pyplot as plt
import visual

BUF_LEN = 26
RAW_LEN = 9
HEAD_LEN = 8
COM = 'COM6'

class Parameters(object):
    # Calibrated rest level.
    SENSOR_ZERO_LEVEL = [505, 504, 507, 386, 384, 382, 0, 0, 0]
    # Calibrated sensitivity. (Because the z-axis gyro is on a different chip, the
    # data for z-axis gyro is slightly different from the x/y-axis gyro.
    SENSOR_SENSITIVITY = [102, 103, 102, 15.9142, 15.8326, 14.2891, 1300, 1300, 1300]

    # This is the covariance matrix for the noise in the gyroscope,
    # represented by quaternions. Should be used as Q matrix in the EKalman.
    # Static
    GYRO_ERR_COV_S = np.matrix(
        [[ 4.80574003e-01,  2.77505891e-03, -8.69992105e-04, -4.03247783e-04],
         [ 2.77505891e-03,  9.19282667e-02,  5.28045224e-05, -2.45922986e-03],
         [-8.69992105e-04,  5.28045224e-05,  8.54215839e-02,  5.30840124e-03],
         [-4.03247783e-04, -2.45922986e-03,  5.30840124e-03,  6.06963705e-02]])

    # Dynamic
    GYRO_ERR_COV_D = np.matrix(
        [[ 4.98022904e-01, -4.73826515e-04,  1.52772269e-04,  1.35944826e-03],
         [-4.73826515e-04,  1.73284439e-01, -5.02642964e-03,  2.74398575e-02],
         [ 1.52772269e-04, -5.02642964e-03,  1.59915122e-01,  7.54602092e-03],
         [ 1.35944826e-03,  2.74398575e-02,  7.54602092e-03,  1.68741929e-01]])

    # The covariance matrix for the noise in the accelerometer. Should be used
    # as R matrix in the EKalman.
    # Static
    ACC_ERR_COV_S = np.matrix(
        [[ 2.08766874e-04,  3.23583653e-06, -4.37264738e-05],
         [ 3.23583653e-06,  2.31599267e-04, -5.04887598e-05],
         [-4.37264738e-05, -5.04887598e-05,  2.49027804e-04]])

    # Dynamic
    ACC_ERR_COV_D = np.matrix(
        [[ 0.08552561, -0.00511642, -0.00672479],
         [-0.00511642,  0.06799363, -0.00108495],
         [-0.00672479, -0.00108495,  0.10689232]])

    @classmethod
    def acc_h(cls, x):
        q = x.A
        return np.matrix([[2*q[1]*q[3]-2*q[2]*q[0]],
                          [2*q[2]*q[3]+2*q[1]*q[0]],
                          [1-2*q[1]*q[1]-2*q[2]*q[2]]])

    @classmethod
    def acc_H(cls, xminus):
        q = xminus.A
        return np.matrix([[-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
                          [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
                          [0, -4*q[1], -4*q[2], 0]])


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
        Scale the raw data by divide the raw data by self.sensitivity.
        '''
        data = []
        for i in range(RAW_LEN):
            data.append(raw[i] * 1.0 / self.sensitivity[i])

        return data

    def normalize(self, raw):
        '''
        -> data: the normalized IMU data.
        raw: the raw IMU data.
        '''
        data = []
        for i in range(RAW_LEN):
            data.append((raw[i] - self.zero_level[i]) * 1.0 / self.sensitivity[i])
        return self.align_axis(data)

    def align_axis(self, data):
        '''
        The device's axis are not aligned. This function helps align the data.
        '''
        # For this device
        # x =  ay, y = -ax, z =  gz
        # x =  gx, y =  gy, z =  gz
        # x = -mx, y =  my, z = -mz
        # We need to change the accelerometer data to align the axis.
        temp = data[0]
        data[0] = data[1]
        data[1] = -temp
        
        # As well as the magnetometer
        data[6] = -data[6]
        data[8] = -data[8]
        return data
        

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

    def read_data(self):
        '''
        Get all data from the database.
        '''
        self.cur.execute(self.SQL_SELECT_DATA)
        return self.cur.fetchall()

    def __del__(self):
        if 'conn' in dir(self):
            self.conn.commit()
            self.conn.close()

class Quaternion(object):
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

        self.normalize()

    @classmethod
    def from_gyro(cls, gyro, dt):
        theta = 0
        for a in gyro:
            theta += a * a
        
        theta = np.sqrt(theta)
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
        pass

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
        qa = self.q.A1
        I = np.identity(3)
        q3 = np.matrix([qa[1], qa[2], qa[3]]).T
        QQ = np.matrix([[0, -qa[3], qa[2]],
                        [qa[3], 0, -qa[1]],
                        [-qa[2], qa[1], 0]])
        
        return ((qa[0] * qa[0] - q3.T * q3).A1[0] * I + 2 * q3 * q3.T + \
               2 * qa[0] * QQ).A

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

    def normalize(self):
        q = self.q.A1
        s = 0
        for axis in q:
            s += axis * axis

        s = np.sqrt(s)
        a = []
        for axis in q:
            a.append(axis / s)
        self.q = np.matrix(a).T
        

class EKalman(object):
    '''
    The Kalman Filter class for the IMU.
    '''
    def __init__(self, array=[1, 0, 0, 0]):
        self.x = Quaternion(array)
        self.p = self.Q

    def normalize(self, data):
        s = 0
        for axis in data:
            s += axis*axis

        s = np.sqrt(s)

        a = []
        for axis in data:
            a.append(axis / s)

        return a

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
        A = q.matrix_repr
        self.pminus = A * self.p * A.T + Q

    def measurement_update(self, data, R, H):
        '''
        The measurement update phase of the EKF.
        '''
        I = np.identity(4)
        z = np.matrix(self.normalize(acc)).T
        
        K = self.pminus * H.T * ((H * self.pminus * H.T + R).I)
        self.x = Quaternion(self.xminus.M + K * (z - self.h(self.xminus)))
        self.p = (I - K * H) * self.pminus
    
    @property
    def quat(self):
        return self.x
        
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
    def __init__(self, quat):
        # For converting
        self.rt_mx = np.matrix([[1, 0, 0],
                                [0, 0, 1],
                                [0, -1, 0]])

        self.x = visual.arrow(color=(1,0,0))
        self.y = visual.arrow(color=(0,1,0))
        self.z = visual.arrow(color=(0,0,1))
        self.show(quat)

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
