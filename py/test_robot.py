import api.robot as rbt
# init the robot
a1 = rbt.Robot()

# init robot p-d coefficient
kp = [0 for i in range(12)]
kd = [0 for i in range(12)]
a1.init_k(kp, kd)

# observe
obs = a1.observe()

# make action
posi = [0 for i in range(12)]


# take action
a1.take_action(posi)

