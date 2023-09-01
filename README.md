# v3.3.1

## How to use

This is the repo build to run your RL algorithm on the Unitree A1 

And now I'll tell you how to use this repo 



Before you start, you should have things below:

1. cable(connect your PC to A1)
2. A PC (Linux / Win is all ok )
3. Python(3.8 + recommend)

Now Let's start build

1. build your Unitree sdk as following command (from Info to Important parts )
2. build the python version(notice the CMAKELISTS.txt)

## Info

This repo was forked from [Maddy](https://github.com/Maddy1206/quadruped_py_control) and [Unitree](https://github.com/unitreerobotics/unitree_legged_sdk) 

And I've implemented this sdk for python

The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

## Important 

You need to update your Raspi on the A1 to arm64

### Notice
support robot: Aliengo, A1(sport_mode >= v1.19)

not support robot: Laikago, Go1.

### Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build C++
```bash
mkdir build
cd build
cmake ../
make
```

### Build for Python

```bash
cd python_wrapper
mkdir build
vim ../CMakelists.txt # change your ".so" info for your platform
cmake ..
make 
```

You'll find a repo named "robot_interface*(your platform) *.so"

### Usage

Run examples with 'sudo' for memory locking.



## How to Use Python Version

### Copy your lib

```bas	
cp where-to-your-robot_interface-.so where-to-your-destination
cd where-to-your-des
sudo python3
```

### Run this file to run

remeber to use sudo to initial UDP

```python
import robot_interface as sdk
import time
udp = sdk.UDP(0xff, sdk.Basic) # oxff is LowCmd
safe = sdk.Safety(sdk.LeggedType.A1)

cmd = sdk.LowCmd()
state = sdk.LowState()
udp.InitCmdData(cmd)

while True:
    time.sleep(0.01)
    udp.Recv()
    udp.GetRecv(state)
    
    for i in state.imu.quaternion:
        print(i)
        
    udp.SetSend(cmd)
    udp.Send()
```



## Robot Interface 

The base function of the robot has been written to the `robot.py`

```sh
cd py
sudo python3 ./test_robot.py
```

The robot usage has been written to the `test_robot.py`

## How to use Robot

```python
import api.robot as rbt
from utils.signal_hand import quit_robot
import signal

# this one will make your robot go back to the original position in 100 * self.dt time
def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    quit_robot(a1)
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

```

## How to change the file to fit your observation and action?

1. in the `robot.py` file, change the `observe()` func to fit your observation
2. in the `take_action` func, it'll change your action to command and send it to your robot 





## What's more 

1. the connection might be not stable if your cable is not very strong or stable ,It'll limit the behavior of your robot 
2. I've established a limit of torque and position 
3. change the `PowerProtect` if there is an error `Power Protection end ` when you run your model 

All these funcs are in the `robot.py` file 

