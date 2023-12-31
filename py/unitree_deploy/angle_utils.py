from math import sin,pi
import math
import numpy as np


def quart_to_rpy(quat):
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return [roll, pitch, yaw]

def sine_generator(idx, T, rate=1):
    if isinstance(idx, int):
        base1= 0.5233
        base3= 0.0
        base2 = -2 * base1
        ang = abs(sin( float(idx) / T  * pi)) * rate
        angle_list = [0] * 12
        idx_base = 0

        if (int(idx/T) % 2 ) == 1:
            angle_list[idx_base+1] = ang + base1
            angle_list[idx_base+2] = -2 * ang + base2

            angle_list[idx_base+4] = base1
            angle_list[idx_base+5] = base2

            angle_list[idx_base+7] = base1
            angle_list[idx_base+8] = base2
            angle_list[idx_base+10] = ang + base1
            angle_list[idx_base+11] = - 2 * ang + base2
        else:
            angle_list[idx_base+1] = base1
            angle_list[idx_base+2] = base2

            angle_list[idx_base+10] = base1
            angle_list[idx_base+11] = base2
            angle_list[idx_base+4] = ang + base1
            angle_list[idx_base+5] = -2 * ang + base2

            angle_list[idx_base+7] = ang + base1
            angle_list[idx_base+8] = -2 * ang + base2
        angle_list[idx_base+0] = base3
        angle_list[idx_base+3] = base3
        angle_list[idx_base+6] = base3
        angle_list[idx_base+9] = base3
        return np.array(angle_list)
    else:
        # tmp = []
        tmp=  [sine_generator(x, T, rate) for x in idx]
        return np.array(tmp)


if __name__=='__main__':
    # angle_list = [[0 for x in range(12)] for z in range(3)]
    x =quart_to_rpy(1, 0, 0, 0)
    print(x)