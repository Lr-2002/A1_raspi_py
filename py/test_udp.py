
import sys
import time
import math
import numpy as np

sys.path.append('./')
import robot_interface as sdk


def jointLinearInterpolation(initPos, targetPos, rate):
    rate = np.fmin(np.fmax(rate, 0.0), 1.0)
    p = initPos * (1 - rate) + targetPos * rate
    return p


if __name__ == '__main__':

    d = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
         'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
         'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
         'RL_0': 9, 'RL_1': 10, 'RL_2': 11}
    PosStopF = math.pow(10, 9)
    VelStopF = 16000.0
    HIGHLEVEL = 0xee
    LOWLEVEL = 0xff
    sin_mid_q = [0.0, 1.2, -2.0]
    dt = 0.002
    qInit = [0, 0, 0]
    qDes = [0, 0, 0]
    sin_count = 0
    rate_count = 0
    Kp = [0, 0, 0]
    Kd = [0, 0, 0]

    udp = sdk.UDP(LOWLEVEL,sdk.Basic)
    safe = sdk.Safety(sdk.LeggedType.A1)

    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)


    Tpi = 0
    motiontime = 0
    while True:
        time.sleep(0.02)
        motiontime += 1
        udp.Recv()
        udp.GetRecv(state)
        print(motiontime)
        for i in state.imu.quaternion:
            print(i)
        for i in range(12):
            print(state.motorState[i].q, state.motorState[i].dq)

        if (motiontime > 10):
            safe.PowerProtect(cmd, state, 1)

        udp.SetSend(cmd)
        udp.Send()
