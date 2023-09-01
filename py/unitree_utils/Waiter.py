import time
import numpy as np
def get_ms():
    return int(time.time_ns()/ 1e6)

def ms_s(ms):
    return ms/1000

def get_ms_in_s():
    return ms_s(get_ms())
class Waiter:
    def __init__(self, dt):
        self.__dt = dt
        self.__start = None

    def wait(self):
        # self.__start = get_ms()
        if self.__start is None:
            raise TimeoutError('init __start time with Waiter.update_start()')

        if ms_s(get_ms() - self.__start) +0.001 < self.__dt:
            # print(self.__dt - ms_s(get_ms() - self.__start))
            time.sleep(max(self.__dt - ms_s(get_ms() - self.__start), 0.0015))
            # print('waited sleep')
        else:
            print('no need to wait ')
        self.update_start()

    def update_start(self):
        self.__start = get_ms()

    def is_inited(self):
        if self.__start is not None:
            return True
        else:
            return False

    # def kill(self):
    #     self.__start = None



if __name__ == '__main__':
    wait = Waiter(0.01)
    wait.update_start()
    for i in range(100):
        ti = np.random.random(1)/10

        print(ti)
        time.sleep(ti[0])
        wait.wait()
