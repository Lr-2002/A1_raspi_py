import robot_interface as sdk

udp = sdk.UDP()
state = sdk.LowState()
# for i in state.motorState:
#     print(i.q)
print(state.imu.quaternion)