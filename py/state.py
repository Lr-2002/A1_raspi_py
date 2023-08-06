import robot_interface as sdk

state = sdk.LowState()
# for i in state.motorState:
#     print(i.q)
print(state.imu.quaternion)