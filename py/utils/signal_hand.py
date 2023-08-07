import signal
import sys
def quit_robot(robot):
    robot.back_safe()
    print('Task finished')
    sys.exit(0)



