import threading

import numpy as np
import sys
sys.path.append('../')
import unitree_api.robot_interface as sdk
import time

import math
from unitree_utils.one_call import only_run_once
from unitree_utils.Waiter import Waiter, get_ms_in_s
from unitree_deploy.angle_utils import quart_to_rpy
from collections import namedtuple
from unitree_api.filter.a1_robot_velocity_estimator import VelocityEstimator
d = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
     'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
     'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
     'RL_0': 9, 'RL_1': 10, 'RL_2': 11}
PosStopF = math.pow(10, 9)
VelStopF = 16000.0
HIGHLEVEL = 0xee
LOWLEVEL = 0xff


def analytical_leg_jacobian(leg_angles, leg_id):
    """
  Computes the analytical Jacobian.
  Args:
  ` leg_angles: a list of 3 numbers for current abduction, hip and knee angle.
    l_hip_sign: whether it's a left (1) or right(-1) leg.
  """
    l_up = 0.2
    l_low = 0.2
    l_hip = 0.08505 * (-1)**(leg_id + 1)

    t1, t2, t3 = leg_angles[0], leg_angles[1], leg_angles[2]
    l_eff = np.sqrt(l_up**2 + l_low**2 + 2 * l_up * l_low * np.cos(t3))
    t_eff = t2 + t3 / 2
    J = np.zeros((3, 3))
    J[0, 0] = 0
    J[0, 1] = -l_eff * np.cos(t_eff)
    J[0, 2] = l_low * l_up * np.sin(t3) * np.sin(
        t_eff) / l_eff - l_eff * np.cos(t_eff) / 2
    J[1, 0] = -l_hip * np.sin(t1) + l_eff * np.cos(t1) * np.cos(t_eff)
    J[1, 1] = -l_eff * np.sin(t1) * np.sin(t_eff)
    J[1, 2] = -l_low * l_up * np.sin(t1) * np.sin(t3) * np.cos(
        t_eff) / l_eff - l_eff * np.sin(t1) * np.sin(t_eff) / 2
    J[2, 0] = l_hip * np.cos(t1) + l_eff * np.sin(t1) * np.cos(t_eff)
    J[2, 1] = l_eff * np.sin(t_eff) * np.cos(t1)
    J[2, 2] = l_low * l_up * np.sin(t3) * np.cos(t1) * np.cos(
        t_eff) / l_eff + l_eff * np.sin(t_eff) * np.cos(t1) / 2
    return J



def pyb_get(quat):
    """
    this function will convert your quaternion to rotmat
    :param quat: your quaternion
    @@order: x,y,z,w
    :return: np.array (rot mat )
    """
    i = 0
    d = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]
    s = 2.0 / d
    xs = quat[0] * s
    ys = quat[1] * s
    zs = quat[2] * s
    wx = quat[3] * xs
    wy = quat[3] * ys
    wz = quat[3] * zs

    xx = quat[0] * xs
    xy = quat[0] * ys
    xz = quat[0] * zs
    yy = quat[1] * ys
    yz = quat[1] * zs
    zz = quat[2] * zs
    mat3x3= [
        1.0 - (yy + zz), xy - wz, xz + wy,
        xy + wz, 1.0 - (xx + zz), yz - wx,
        xz - wy, yz + wx, 1.0 - (xx + yy)
    ]
    mat =np.array(mat3x3)
    return mat.reshape((3,3))



class Qauternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return '{}w+{}x+{}y+{}z'.format(self.w, self.x, self.y, self.z)

class Robot():
    """
    Robot lib here
    """
    def __init__(self):
        """
        change things here
        """
        self.robot = sdk.LeggedType.A1
        self.ob_dims = 27 # todo make function
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
        self.position_limit_down = [-0.8, -1.04, -2.74] * 4

        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)

        self.motiontime = 0
        self.stand_gait = [0, 0.523, -1.046] * 4
        self.kp = [180, 180, 300] * 4
        self.kd = [8, 8, 15] * 4
        self.quaternion = [1, 0, 0, 0]
        self.gyroscope = [0, 0, 0]
        self.est_vel = [0, 0, 0]
        self.euler = None
        self.accelerometer = [0, 0, 0]
        self.position = [0 for i in range(self.act_dims)]
        self.velocity = [0 for i in range(self.act_dims)]
        self.tau = [0]  * self.act_dims

        self._robot_command_lock = threading.RLock()

        self.contact_bias = None
        self.vel_bias = None

        self.vel_estimator = VelocityEstimator(self,accelerometer_variance= 0.1, sensor_variance=0.003, moving_window_filter_size=20)


        self.__backing = False
        # self.quaternion = Qauternion(1, 0, 0, 0

    def update_dt(self, dt):
        """
        change the time_step
        :param dt: time_step
        :return: None
        """
        self.dt = dt
        self.waiter = Waiter(dt)

    def init_motor(self, position):
        """
        init motor kp and kd
        :param position: the position you want to go (using linear lerp)
        :return: None
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
        """
        call this when you need to shut down the robot manully
        It'll kill robot using linear lerp to the original position
        :return: None
        """
        self.back_safe()

        print('Task finished')
        # sys.exit(0)

    def __motor(self,i):
        """
        :param i: index
        :return:  motor[i]
        """
        return self.cmd.motorCmd[i]
    @property
    def ob_dim(self) -> int:
        return self.ob_dims

    @property
    def act_dim(self) -> int:
        return self.act_dims


    @only_run_once
    def connection_init(self):
        tmp = 0
        # print(self.observe())
        while abs(self.observe().sum() ) <= 0.02 and tmp <= 10:
            tmp += 1
            self.udp.Recv()
            self.udp.GetRecv(self.state)
            self.udp.SetSend(self.cmd)
            self.udp.Send()
        if abs(self.observe().sum() ) <= 0.02:
            raise ConnectionError("Cannot read info of robot.")
        else:
            return True

    def single_recv(self):
        """
        just recv without doing anything to establish connection
        :return:
        """
        self.udp.Recv()
        self.udp.GetRecv(self.state)


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
        self.connection_init()
        self.udp.Recv()
        self.udp.GetRecv(self.state)

        self.get_imu() # 4 + 3 + 3 = 10
        self.get_motion() # 12 * 2 = 24
        if self.euler is not None:
            self.check_angle_safe()
        info = []
        tmp =quart_to_rpy(self.quaternion)[0:2]
        if self.contact_bias is not None:
            self.get_body_vel()
        info = tmp +  self.gyroscope
        """
        todo:
         change your observation here 
        """
        for i in range(12):
            info.append(self.position[i])
            info.append(self.velocity[i])
        # info.append(0)


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



        q = np.array([info]).astype(np.float32)
        print("obs shape", q.shape)
        return q

    def get_imu(self):
        """
        update imu info
        z-acc is 9.81 replying to your location
        :return:
        """
        self.quaternion = self.state.imu.quaternion
        self.gyroscope = self.state.imu.gyroscope
        self.euler = self.state.imu.rpy
        self.accelerometer = self.state.imu.accelerometer

        return True

    def get_motion(self):
        """
        update motion info
        include positions,velocitys,taus
        all of them are in the joint order
        :return:
        """
        for i in range(self.act_dims):
            self.get_position(i)
            self.get_velocity(i)
            self.get_tau(i)
        return True



    def get_tau(self, i):
        """
        :param i: which motor tau to get
        :return: tau
        """
        self.tau[i] = self.state.motorState[i].tauEst


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
        self.safe.PowerProtect(self.cmd, self.state, 5)
        self.motiontime += 1

        assert len(position) == self.act_dims
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
        self.udp.SetSend(self.cmd)
        with self._robot_command_lock:
            self.udp.Send()

    def reset(self):
        """
        how to implement
        maybe to be implement with stand
        """
        self.robot.reset()


    def init_k(self, kp, kd):
        """
        init kp and kd
        :param kp: kp
        :param kd: kd
        related to the PID control
        :return: None
        """
        if not (isinstance(kp, list)):
            raise TypeError("Please input a standard kp and kd into the init_k function")
        if len(kp) != self.act_dims:
            raise Exception("kp and kd must have the same length with self.act_dims")
        self.kp = kp
        self.kd = kd
        for i in range(self.act_dim):
            self.__motor(i).Kp = self.kp[i]
            self.__motor(i).Kd = self.kd[i]

        print('init motors\' kp and kd finished')

    def safe_protect(self):
        """
        using self.safe to protect the power
        the third is the percentage of power limit 1~10 for 10%~100%
        :return: None
        """
        self.safe.PowerProtect(self.cmd, self.state, 1)

    @only_run_once
    def pre_hold(self):
        print('Now hold on')
        self.hold_posi = self.position.copy()

    def hold_on(self):
        """
        hold on this position
        Notice: this hold on position is only once set
        if you need to make it external called, modify self.pre_hold() and the decorator @only_run_once
        :return: None
        """
        self.observe()
        self.pre_hold()
        # print(self.hold_posi)
        self.take_action(self.hold_posi)

    def check_angle_safe(self):
        """
        check whether the angle is in the safe range
        :return:
        """
        if abs(self.euler[1]) >= 0.4 or abs(self.euler[0] ) >= 0.4:
            print('euler wrong')
            self.__backing = True
            self.quit_robot()
            sys.exit(0)


    def posi_limit(self, motor, i):
        """
        limit the motor to the position without destroy the robot base
        :param motor:
        :param i:
        :return:
        """
        if motor.q >= self.position_limit_up[i] or motor.q <= self.position_limit_down[i]:
            raise ValueError(
                f"""motor {i} position {motor.q} has been over the position limit ({self.position_limit_up[i]},{self.position_limit_down[i]}).Close the process
                    position: {self.position}
                    imu: {self.quaternion, self.gyroscope, self.accelerometer}
                    kp: {self.kp}
                    kd: {self.kd}
                """)

    def torq_limit(self, motor, i):
        """
        limit the torque
        if the limit is too low, using the command `robot.torque_limit` to set your new boundary
        :param motor:
        :param i:
        :return:
        """
        if motor.tau + motor.Kp * (motor.q - self.position[i]) + motor.Kd * (motor.dq - self.velocity[i]) > self.torque_limit:
            raise ValueError(f"""motor {i}'s torque {motor.tau + motor.Kp * (motor.q - self.position[i]) + motor.Kd * (motor.dq - self.velocity[i])} has been over the limit {self.torque_limit}, Close the process'
                    tor_info(tau, target, now_posi): {motor.tau, motor.q, self.position[i]}
                    position: {self.position}
                    imu: {self.quaternion, self.gyroscope, self.accelerometer}
                    kp: {self.kp}
                    kd: {self.kd}
                                """)

        # print(f"est tau {i}: ", motor.tau + motor.Kp * (motor.q - self.position[i]) + motor.Kd * (motor.dq - self.velocity[i]))

    def line_interpolating(self, begin, end, idx, rate):
        """
        line lerp
        :param begin:
        :param end:
        :param idx:
        :param rate:
        :return:
        """
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
        """
        back to the safe position
        :return:
        """
        print('Robot is going back to safe position')
        self.record_posi()
        for i in range(self.back_time):
            # self.observe()
            self.single_recv()
            self.take_action(self.line_interpolating(self.record_position, self.back_position, i, self.back_time))

    def go_position(self, position, timing):
        """
        go to some position
        :param position: destination
        :param timing: how long time to go (self.dt * timing is the real time)
        :return:
        """
        print('Robot is going to destination {}'.format(position))
        # ori_posi = self.position.copy()
        ori_posi = [i for i in self.position]
        for i in range(timing):
            self.observe()
            self.take_action(self.line_interpolating(ori_posi, position, i, timing))
        print('Robot has been the destination {}'.format(position))
        return True

    def ComputeJacobian(self, leg_id):
        """Compute the Jacobian for a given leg."""
        # Does not work for Minitaur which has the four bar mechanism for now.
        motor_angles = self.position[leg_id * 3:(leg_id + 1) * 3]
        return analytical_leg_jacobian(motor_angles, leg_id)

    def GetBaseAcceleration(self):
        """
        get acc
        :return: self.acc [x,y,z]
        """
        return self.accelerometer


    def GetBaseOrientation(self):
        """
        get quat
        :return: self.quat [w,x,y,z]
        """
        return self.quaternion

    def from_quaternion_to_rot(self):
        """
        :return:rot mat
        """
        return pyb_get([self.quaternion[1], self.quaternion[2], self.quaternion[3], self.quaternion[0]])

    def GetMotorVelocities(self):
        """
        motor velocities
        :return: velocity
        """
        return self.velocity

    def GetFootContacts(self):
        """
        if foot force bigger than 20, return 1 else 0
        :return: [x,x,x,x] x is 0/1
        """
        return [1 if self.GetFootForce()[i] > 20 else 0 for i in range(4)]

    def GetFootForce(self):
        """
        get foot force
        :return: [a,b,c,d] for foot force
        """
        return self.state.footForce

    def GetFootForceEst(self):
        """
        get estimated force
        :return:
        """
        return self.state.footForceEst


    def stand_up(self, timer, stand=None):
        """
        the robot could stand up the the position in timer and the stand is the position it'll go
        :param timer: how long time to go
        :param stand: the destination position
        :return: None
        """
        if stand is not None:
            self.stand_gait = stand
        self.observe()
        self.init_motor(self.position)
        ori_posi = self.position.copy()
        for idx in range(timer):
            self.observe()
            self.take_action(self.line_interpolating(ori_posi, self.stand_gait, idx, timer))
        contact = []
        vel_bias = []
        for i in range(100):
            self.observe()
            vel_bias.append(self.get_body_vel())
            contact.append(self.GetFootForce())
            self.hold_on()
        self.contact_bias = np.array(contact).mean(0)
        self.vel_bias = np.array(vel_bias).mean(0)

    def get_body_vel(self):
        """
        using estimator to estimate the body velocity (VERY  NOT  OK)
        :return:
        """
        self.vel_estimator.update(self.state.tick / 1000)
        self.est_vel = self.vel_estimator.estimated_velocity.copy()
        if self.vel_bias is not None:
            self.est_vel -= self.vel_bias
        return self.est_vel



