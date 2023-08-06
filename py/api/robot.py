import numpy as np
import robot_interface as sdk
import time
import math
from collections import namedtuple

d = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
     'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
     'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
     'RL_0': 9, 'RL_1': 10, 'RL_2': 11}
PosStopF = math.pow(10, 9)
VelStopF = 16000.0
HIGHLEVEL = 0xee
LOWLEVEL = 0xff

class Qauternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return '{}w+{}x+{}y+{}z'.format(self.w, self.x, self.y, self.z)

class Robot():
    def __init__(self):
        self.robot = sdk.LeggedType.A1
        self.ob_dims = self.robot.ob_dims if self.robot is not None else 34 # todo make function
        self.act_dims = self.robot.act_dims if self.robot is not None else 12 # todo
        self.udp = sdk.UDP(LOWLEVEL, sdk.Basic)
        self.dt = 0.01
        self.safe = sdk.safety(self.robot)

        self.__cmd = sdk.LowCmd()
        self.__state = sdk.LowState()
        self.udp.InitCmdData(self.__cmd)

        self.motiontime = 0

        self.kp = 0
        self.kd = 0
        self.quaternion = [1, 0, 0, 0]
        self.gyroscope = [0, 0, 0]
        self.accelerometer = [0, 0, 0]
        self.position = [0 for i in range(self.act_dims)]
        self.velocity = [0 for i in range(self.act_dims)]
        # self.quaternion = Qauternion(1, 0, 0, 0)

    def init_motor(self):
        """
        init motor kp and kd
        :return:
        """
        for i in range(12):
            motor = self.__motor(i)
            motor.tau = 0
            motor.q = 0
            motor.dq = 0
        self.init_k(self.kp, self.kd)

        print('self.motor inited')

    def __motor(self,i):
        """

        :param i: index
        :return:  motor[i]
        """
        return self.__cmd.motorCmd[i]

    def connect(self):
        """
        connect to A1 with high level first
        """

        pass

    def ob_dim(self) -> int:
        return self.ob_dims

    def act_dim(self) -> int:
        return self.act_dims

    def alive(self) -> bool:
        """
        make sure whether the body is online?
        :return:
        """
        return True

    def observe(self) -> np.array:
        """
        get the info data from sensor using  imu and posi-sensor
        the order is
        1. quaternion
        2. gyroscope
        3. accelerometer
        4. position
        5. velocity
        :return: np.array([1, self.ob_dims])
        """
        self.udp.Recv()
        self.udp.GetRecv(self.__state)

        self.get_imu() # 4 + 3 + 3 = 10
        self.get_motion() # 12 * 2 = 24
        info = self.quaternion + self.gyroscope + self.accelerometer + self.position + self.velocity

        return np.array(info).astype(np.float32)

    def get_imu(self):
        """
        update imu info
        todo test the quaternion
        :return:
        """
        self.quaternion = self.__state.imu.quaternion
        self.gyroscope = self.__state.imu.gyroscope
        self.accelerometer = self.__state.imu.accelerometer
        return True

    def get_motion(self):
        """
        update motion info
        :return:
        """
        for i in range(self.act_dims):
            self.get_position(i)
            self.get_velocity(i)
        return True

    def get_velocity(self, i):
        """
        :param i: which motor velocity to get
        :return:
        """
        self.velocity[i] = self.__state.motorState[i].dq

    def get_position(self, i):
        """
        :param i: which motor position to get
        :return:
        """
        self.position[i] = self.__state.motorState[i].q

    def take_action(self, position, dq=None):
        """
        upd recv and send
        :param position : len(list) == self.act_dims
        :param dq: len(list) == self.act_dims
        """
        time.sleep(self.dt)
        assert len(position) == self.act_dims
        # if dq == None:
        #     dq = [0 for i in range(self.act_dims)]

        for i in range(self.act_dims):
            self.__motor(i).q = position[i]
            if dq is not None:
                self.__motor(i).dq = dq[i]

        self.udp.SetSend(self.__cmd)
        self.udp.Send()

    def reset(self):
        """
        how to implement
        maybe to be implement with stand
        """
        self.robot.reset()


    def init_k(self, kp, kd):
        self.kp = kp
        self.kd = kd
        for i in range(self.act_dim()):
            self.__motor(i).kp = self.kp
            self.__motor(i).kd = self.kd
        print('init motors\' kp and kd finished')

    def safe_protect(self):
        """
        using self.safe to protect the power
        :return:
        """
        self.safe.PowerProtect(self.__cmd, self.__state, 1)


