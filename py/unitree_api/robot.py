import numpy as np
import sys
sys.path.append('../')
import unitree_api.robot_interface as sdk
import time
import math
from unitree_utils.one_call import only_run_once
from unitree_utils.Waiter import Waiter, get_ms_in_s
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
        self.ob_dims = 34 # todo make function
        self.act_dims = 12 # todo
        self.udp = sdk.UDP(LOWLEVEL, sdk.Basic)
        self.udp_state = sdk.UDPState()
        self.dt = 0.01
        self.waiter = Waiter(self.dt)
        #safety
        self.safe = sdk.Safety(self.robot)
        self.torque_limit = 30
        self.back_time = 100
        self.back_position = None
        self.position_limit_up = [0.8, 4.18, -0.91] * 4
        self.position_limit_down = [-0.8, -1.04, -2.69] * 4

        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)

        self.motiontime = 0
        self.stand_gait = [0, 0.523, -1.046] * 4
        self.kp = [180, 180, 300] * 4
        self.kd = [8, 8, 15] * 4
        self.quaternion = [1, 0, 0, 0]
        self.gyroscope = [0, 0, 0]
        self.accelerometer = [0, 0, 0]
        self.position = [0 for i in range(self.act_dims)]
        self.velocity = [0 for i in range(self.act_dims)]

        # self.quaternion = Qauternion(1, 0, 0, 0)

    def update_dt(self, dt):
        self.dt = dt
        self.waiter = Waiter(dt)

    def init_motor(self, position):
        """
        init motor kp and kd
        :return:
        """
        self.observe()
        self.connection_init()
        if self.back_position is None:
            self.back_position = self.position.copy()
        for i in range(12):
            motor = self.__motor(i)
            motor.mode = 10
            motor.tau = 0
            # motor.q = self.stand_gait[i]
            motor.dq = 0
        self.init_k(self.kp, self.kd)
        print('You kp are', self.kp)
        print('You kd are', self.kd)
        print('your now position is', self.position)
        input('Are you sure to start?')
        self.take_action(position)
        print('self.motor inited')

    def quit_robot(self):
        # self.waiter.kill()
        self.back_safe()

        print('Task finished')
        sys.exit(0)

    def __motor(self,i):
        """

        :param i: index
        :return:  motor[i]
        """
        return self.cmd.motorCmd[i]

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

    @only_run_once
    def connection_init(self):
        tmp = 0
        while abs(self.observe().sum()) <= 0.02 and tmp <= 10:
            tmp += 1
            self.udp.Recv()
            self.udp.GetRecv(self.state)
            self.udp.SetSend(self.cmd)
            self.udp.Send()
        if abs(self.observe().sum()) <= 0.02:
            raise ConnectionError("Cannot read info of robot.")
        else:
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
        # time.sleep(self.dt)
        self.connection_init()
        # state = sdk.LowState()
        self.udp.Recv()
        self.udp.GetRecv(self.state)

        self.get_imu() # 4 + 3 + 3 = 10
        self.get_motion() # 12 * 2 = 24
        info = self.quaternion + self.gyroscope
        for i in range(12):
            info.append(self.position[i])
            info.append(self.velocity[i])



        """
        obs:
            1. self.quaternion     四元数 右手系,x-aix沿着头的正方向,z-up w,x,y,z 
            2. self.gyroscope      陀螺仪       角速度： rad/s
            3. self.accelerometer  加速度计     线加速度: m/s^2
            4. self.position       关节角位置   12个关节的角位置: rad
            5. self.velocity       关节角速度   12个关节的角速度: rad/s
        add-on：
            1. 12个关节的顺序: FR, FL, RR, RL
        """



        return np.array(info).astype(np.float32)

    def get_imu(self):
        """
        update imu info
        todo test the quaternion
        :return:
        """
        self.quaternion = self.state.imu.quaternion
        self.gyroscope = self.state.imu.gyroscope
        self.accelerometer = self.state.imu.accelerometer
        self.accelerometer[2] -= 9.81
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
        self.velocity[i] = self.state.motorState[i].dq

    def get_position(self, i):
        """
        :param i: which motor position to get
        :return:
        """
        self.position[i] = self.state.motorState[i].q

    def take_action(self, position, dq=None):
        """
        upd recv and send
        :param position : len(list) == self.act_dims
        :param dq: len(list) == self.act_dims
        """
        if not (isinstance(position, list)):
            raise TypeError("Please input a standard position list into the take_action function")
        if len(position) != self.act_dims:
            raise Exception("position must have the same length with self.act_dims")
        self.safe.PowerProtect(self.cmd, self.state, 1)
        self.motiontime += 1
        # print(self.motiontime)

        # if self.motiontime
        # time.sleep(self.dt)
        # print('Position going to exec:', position)
        # print('Kp:', self.kp)
        # print('kd:', self.kd)
        # input('Are you sure to go on?')
        assert len(position) == self.act_dims
        # if dq == None:
        #     dq = [0 for i in range(self.act_dims)]
        for i in range(self.act_dims):
            self.__motor(i).q = position[i]
            if dq is not None:
                self.__motor(i).dq = dq[i]
            self.torq_limit(self.__motor(i), i)
            self.posi_limit(self.__motor(i), i)
        if self.waiter.is_inited():
            self.waiter.wait()
        else:
            self.waiter.update_start()
        print(get_ms_in_s())
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def reset(self):
        """
        how to implement
        maybe to be implement with stand
        """
        self.robot.reset()


    def init_k(self, kp, kd):
        if not (isinstance(kp, list)):
            raise TypeError("Please input a standard kp and kd into the init_k function")
        if len(kp) != self.act_dims:
            raise Exception("kp and kd must have the same length with self.act_dims")
        self.kp = kp
        self.kd = kd
        for i in range(self.act_dim()):
            self.__motor(i).Kp = self.kp[i]
            self.__motor(i).Kd = self.kd[i]

        print('init motors\' kp and kd finished')

    def safe_protect(self):
        """
        using self.safe to protect the power
        :return:
        """
        self.safe.PowerProtect(self.cmd, self.state, 1)

    @only_run_once
    def pre_hold(self):
        print('Now hold on')
        self.hold_posi = self.position.copy()

    def hold_on(self):
        self.observe()
        self.pre_hold()
        # print(self.hold_posi)
        self.take_action(self.hold_posi)

    def posi_limit(self, motor, i):
        if motor.q >= self.position_limit_up[i] or motor.q <= self.position_limit_down[i]:
            raise ValueError(
                f"""motor {i} position {motor.q} has been over the position limit ({self.position_limit_up[i]},{self.position_limit_down[i]}).Close the process
                    position: {self.position}
                    imu: {self.quaternion, self.gyroscope, self.accelerometer}
                    kp: {self.kp}
                    kd: {self.kd}
                """)

    def torq_limit(self, motor, i):
        if motor.tau + motor.Kp * (motor.q - self.position[i]) + motor.Kd * (motor.dq - self.velocity[i]) > self.torque_limit:
            raise ValueError(f"""motor {i}'s torque {motor.tau + motor.Kp * (motor.q - self.position[i]) + motor.Kd * (motor.dq - self.velocity[i])} has been over the limit {self.torque_limit}, Close the process'
                    tor_info(tau, target, now_posi): {motor.tau, motor.q, self.position[i]}
                    position: {self.position}
                    imu: {self.quaternion, self.gyroscope, self.accelerometer}
                    kp: {self.kp}
                    kd: {self.kd}
                                """)

    def line_interpolating(self, begin, end, idx, rate):
        if isinstance(begin, list):
            assert isinstance(end, list)
            ans = []
            for b, e in zip(begin, end):
                ans.append(self.line_interpolating(b, e, idx, rate))
            return ans
        return end * idx / rate + (1 - idx / rate) * begin

    @only_run_once
    def record_posi(self):
        self.record_position = self.position.copy()

    def back_safe(self):
        print('Robot is going back to safe position')
        self.record_posi()
        for i in range(self.back_time):
            self.observe()
            self.take_action(self.line_interpolating(self.record_position, self.back_position, i, self.back_time))

    def go_position(self, position, timing):
        print('Robot is going to destination {}'.format(position))
        ori_posi = self.position.copy()
        for i in range(timing):
            self.observe()
            self.take_action(self.line_interpolating(ori_posi, position, i, timing))
        print('Robot has been the destination {}'.format(position))
        return True