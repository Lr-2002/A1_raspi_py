# v3.3.1

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

Comming soon
