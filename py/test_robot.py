import api.robot as rbt
import numpy as np
import time
# init the robot
import signal
import sys
from deploy.sine_generator import sine_generator

a1 = rbt.Robot()


# log
"""
1. robot 上电之后会卡卡卡响
2. 使用遥控器控制的话 会出现正常的电机运转声 但是并不会出现卡卡卡的响声
"""
def generate_line_begin_end(begin, end, idx, rate):
    if isinstance(begin, list):
        assert isinstance(end, list)
        ans = []
        for b, e in zip(begin, end):
            ans.append(generate_line_begin_end(b, e, idx, rate))
        return ans
    return end * idx/rate + (1 - idx/rate) * begin



def quit_robot(robot: rbt.Robot):
    robot.back_safe()
    print('Task finished')
    sys.exit(0)

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    quit_robot(a1)

signal.signal(signal.SIGINT, signal_handler)



# take action
# a1.take_action(posi)


if __name__=='__main__':


    # init robot p-d coefficient

    # obs = a1.observe()
    # print(obs)
    act = a1.position
    print(act)
    # print(a1.position_limit_up, a1.position_limit_down)

    a1.init_motor(act)
    # observe

    # make action
    # start_list = [-0.1552944779396057, 1.226299524307251, -2.127218246459961, 0.5057500600814819, 1.378053903579712, -2.6986260414123535, -0.14581245183944702, 1.3590056896209717, -2.6817691326141357, -0.4573284983634949, 1.2668824195861816, -2.6768805980682373]
    # e = [0, 0.67, -1.3, 0, 0.67, -1.3, 0, 0.67, -1.3, 0, 0.67, -1.3]
    e = act.copy()
    e[2] = -2
    T = 500
    # a1.dt = 0.3
    for idx in range(T):
        # print(len(generate_line_begin_end(act, e, idx, T)))
        a1.observe()
        a1.take_action(generate_line_begin_end(act, e, idx, T))

    sine_T = 80
    # ang_list = a1.stand_gait.copy()
    for idx in range(sine_T  * 10):
        a1.observe()
        a1.take_action(sine_generator(idx, sine_T, 0.15))

    while True:
        a1.hold_on()
