import api.robot as rbt
from utils.signal_hand import quit_robot
import signal


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    quit_robot(a1)
# this one will make your robot go back to the original position in 100 * self.dt time
signal.signal(signal.SIGINT, signal_handler)


a1 = rbt.Robot()

a1.observe()
act = a1.position

a1.init_motor(act)

while True:
    obs = a1.observe()

    act = USE_YOUR_MODEL(obs)

    a1.take_action(act)

# run below command if you need  hold your posture
# while True:
#     a1.hold_on()


